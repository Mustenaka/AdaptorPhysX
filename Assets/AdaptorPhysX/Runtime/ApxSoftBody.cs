using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Cuttable volumetric soft body. Voxelizes the (closed) mesh on Start, simulates
    /// it in FixedUpdate and deforms the rendered mesh in LateUpdate.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    [DisallowMultipleComponent]
    public sealed class ApxSoftBody : MonoBehaviour
    {
        public static readonly List<ApxSoftBody> Active = new List<ApxSoftBody>();

        [Header("Shape")]
        [Tooltip("Closed surface mesh; defaults to the MeshFilter mesh.")]
        public Mesh sourceMesh;
        [Tooltip("Lattice cells along the longest axis.")]
        [Range(4, 64)] public int resolution = 16;

        [Header("Material")]
        [Tooltip("Young's modulus in Pa.")]
        public float youngsModulus = 3.0e4f;
        [Range(0.0f, 0.49f)] public float poissonRatio = 0.45f;
        [Tooltip("kg/m^3")]
        public float density = 1000.0f;
        [Tooltip("Velocity damping per second.")]
        [Min(0.0f)] public float damping = 1.0f;
        [Tooltip("Rest-shape contraction applied at Start. Pinned tissue is pre-tensioned and incisions gape.")]
        [Range(0.0f, 0.3f)] public float preStrain;

        [Header("Solver")]
        [Range(1, 64)] public int substeps = 16;
        public bool useGravity = true;

        [Header("Collision")]
        public LayerMask collisionLayers = ~0;
        [Range(0.0f, 2.0f)] public float friction = 0.6f;
        [Tooltip("Distance kept between lattice nodes and colliders; negative lets the surface touch.")]
        public float collisionMargin;
        [Tooltip("Contact between cut-off pieces and between folded parts of the body.")]
        public bool selfCollision = true;

        [Header("Attachments")]
        [Tooltip("Nodes inside these colliders at Start follow the collider's transform.")]
        public Collider[] pinColliders = new Collider[0];

        [Header("Rendering")]
        [Tooltip("Material of cut surfaces (submesh 1).")]
        public Material cutMaterial;

        public SoftBodySimulation Simulation { get; private set; }
        public SurfaceMesh Surface { get; private set; }
        public bool IsGrabbing { get; private set; }

        private readonly Dictionary<int, ColliderShape> _previousShapes = new Dictionary<int, ColliderShape>();
        private readonly List<ColliderShape> _shapes = new List<ColliderShape>();
        private readonly Collider[] _overlap = new Collider[64];
        private float4x4[] _pinFrames;
        private float3 _grabTarget;
        private float3 _grabTargetPrevious;
        private MeshRenderer _renderer;

        /// <summary>World-space bounds of the lattice nodes (slightly larger than the rendered surface).</summary>
        public Bounds WorldBounds
        {
            get
            {
                if (Simulation == null)
                {
                    return new Bounds(transform.position, Vector3.zero);
                }
                var bounds = new Bounds();
                bounds.SetMinMax(Simulation.BoundsMin, Simulation.BoundsMax);
                return bounds;
            }
        }

        public bool IsInitialized => Simulation != null;

        private void Start() => Initialize();

        private void FixedUpdate() => Simulate(Time.fixedDeltaTime);

        private void LateUpdate() => UpdateRender();

        private void OnDestroy() => Release();

        /// <summary>Voxelizes the mesh and creates the simulation. Called by Start; callable in edit mode.</summary>
        public void Initialize()
        {
            if (Simulation != null)
            {
                return;
            }
            _renderer = GetComponent<MeshRenderer>();
            Mesh mesh = sourceMesh != null ? sourceMesh : GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null || !mesh.isReadable)
            {
                Debug.LogError("ApxSoftBody needs a readable closed mesh.", this);
                enabled = false;
                return;
            }

            Vector3 scale = transform.lossyScale;
            var positions = new List<float3>();
            var normals = new List<float3>();
            var uvs = new List<Vector2>();
            var triangles = new List<int>();
            Vector3[] meshVertices = mesh.vertices;
            Vector3[] meshNormals = mesh.normals;
            Vector2[] meshUvs = mesh.uv;
            for (int i = 0; i < meshVertices.Length; i++)
            {
                positions.Add(Vector3.Scale(meshVertices[i], scale));
                Vector3 n = meshNormals.Length == meshVertices.Length ? meshNormals[i] : Vector3.up;
                normals.Add(math.normalizesafe(new float3(n.x / scale.x, n.y / scale.y, n.z / scale.z)));
                uvs.Add(meshUvs.Length == meshVertices.Length ? meshUvs[i] : Vector2.zero);
            }
            for (int s = 0; s < mesh.subMeshCount; s++)
            {
                triangles.AddRange(mesh.GetTriangles(s));
            }

            Vector3 size = Vector3.Scale(mesh.bounds.size, scale);
            float cellSize = Mathf.Max(size.x, Mathf.Max(size.y, size.z)) / resolution;
            MeshUtil.Subdivide(positions, normals, uvs, triangles, 0.5f * cellSize);

            float3[] restPositions = positions.ToArray();
            int[] restTriangles = triangles.ToArray();
            VoxelGrid grid = Voxelizer.Voxelize(restPositions, restTriangles, cellSize);
            float4x4 restToWorld = float4x4.TRS(transform.position, transform.rotation, new float3(1.0f));
            Simulation = new SoftBodySimulation(grid, BuildParameters(), restToWorld);

            _pinFrames = new float4x4[pinColliders.Length];
            for (int p = 0; p < pinColliders.Length && p + 1 < SoftBodySimulation.MaxKinematicFrames; p++)
            {
                Collider pin = pinColliders[p];
                if (pin == null)
                {
                    continue;
                }
                _pinFrames[p] = FrameOf(pin.transform);
                var inside = new List<int>();
                for (int n = 0; n < Simulation.NodeCount; n++)
                {
                    Vector3 x = Simulation.Positions[n];
                    if ((pin.ClosestPoint(x) - x).sqrMagnitude < 1e-10f)
                    {
                        inside.Add(n);
                    }
                }
                Simulation.Attach(inside, p + 1, _pinFrames[p]);
            }

            Surface = new SurfaceMesh(Simulation, restPositions, normals.ToArray(), uvs.ToArray(), restTriangles);
            GetComponent<MeshFilter>().sharedMesh = Surface.Mesh;
            if (cutMaterial != null)
            {
                Material skin = _renderer.sharedMaterial;
                _renderer.sharedMaterials = new[] { skin, cutMaterial };
            }
            Surface.Update(transform.worldToLocalMatrix);
            Active.Add(this);
        }

        /// <summary>Advances the body by one fixed step of length <paramref name="dt"/>.</summary>
        public void Simulate(float dt)
        {
            if (Simulation == null)
            {
                return;
            }
            Simulation.CompleteStep();
            Simulation.Parameters = BuildParameters();
            GatherColliders();

            Simulation.FramesStart[SoftBodySimulation.GrabFrame] = float4x4.Translate(_grabTargetPrevious);
            Simulation.FramesEnd[SoftBodySimulation.GrabFrame] = float4x4.Translate(_grabTarget);
            _grabTargetPrevious = _grabTarget;
            for (int p = 0; p < _pinFrames.Length && p + 1 < SoftBodySimulation.MaxKinematicFrames; p++)
            {
                if (pinColliders[p] == null)
                {
                    continue;
                }
                float4x4 frame = FrameOf(pinColliders[p].transform);
                Simulation.FramesStart[p + 1] = _pinFrames[p];
                Simulation.FramesEnd[p + 1] = frame;
                _pinFrames[p] = frame;
            }

            // Scheduled only: bodies step in parallel and complete in LateUpdate (or on the next access).
            using var shapes = new NativeArray<ColliderShape>(_shapes.ToArray(), Allocator.Temp);
            Simulation.ScheduleStep(dt, substeps, shapes);
        }

        /// <summary>Deforms the rendered mesh to the current node positions.</summary>
        public void UpdateRender()
        {
            Simulation?.CompleteStep();
            Surface?.Update(transform.worldToLocalMatrix);
        }

        /// <summary>Releases native memory. Called by OnDestroy.</summary>
        public void Release()
        {
            Active.Remove(this);
            Surface?.Dispose();
            Simulation?.Dispose();
            Surface = null;
            Simulation = null;
        }

        /// <summary>Cuts along the quad swept by a blade edge from (a0,b0) to (a1,b1). Returns severed lattice edges.</summary>
        public int Cut(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1)
        {
            return Simulation == null ? 0 : Simulation.Cut(a0, b0, a1, b1);
        }

        /// <summary>Grabs the material under the ray within <paramref name="radius"/> (rest space).</summary>
        public bool BeginGrab(Ray ray, float radius, out Vector3 point)
        {
            point = default;
            if (Simulation == null)
            {
                return false;
            }
            int node = Simulation.PickNode(ray.origin, ray.direction, Simulation.CellSize);
            if (node < 0)
            {
                return false;
            }
            point = Simulation.Positions[node];
            _grabTarget = point;
            _grabTargetPrevious = point;
            Simulation.Attach(Simulation.ConnectedNodes(node, radius), SoftBodySimulation.GrabFrame,
                float4x4.Translate(point));
            IsGrabbing = true;
            return true;
        }

        public void MoveGrab(Vector3 target) => _grabTarget = target;

        public void EndGrab()
        {
            Simulation?.Detach(SoftBodySimulation.GrabFrame);
            IsGrabbing = false;
        }

        private SoftBodyParameters BuildParameters() => new SoftBodyParameters
        {
            YoungsModulus = Mathf.Max(youngsModulus, 1.0f),
            PoissonRatio = poissonRatio,
            Density = Mathf.Max(density, 1e-3f),
            Damping = damping,
            PreStrain = preStrain,
            Gravity = useGravity ? (float3)Physics.gravity : float3.zero,
            CollisionMargin = collisionMargin,
            SelfCollision = selfCollision,
        };

        private static float4x4 FrameOf(Transform t) => float4x4.TRS(t.position, t.rotation, new float3(1.0f));

        private void GatherColliders()
        {
            _shapes.Clear();
            Bounds bounds = WorldBounds;
            bounds.Expand(2.0f * Simulation.CellSize + 0.05f);
            int count = Physics.OverlapBoxNonAlloc(bounds.center, bounds.extents, _overlap, Quaternion.identity,
                collisionLayers, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < count; i++)
            {
                Collider collider = _overlap[i];
                if (collider.transform.IsChildOf(transform) || System.Array.IndexOf(pinColliders, collider) >= 0)
                {
                    continue;
                }
                if (!TryMakeShape(collider, out ColliderShape shape))
                {
                    continue;
                }
                int id = collider.GetInstanceID();
                if (_previousShapes.TryGetValue(id, out ColliderShape previous))
                {
                    shape.A0 = previous.A1;
                    shape.B0 = previous.B1;
                    shape.R0 = previous.R1;
                }
                _previousShapes[id] = shape;
                _shapes.Add(shape);
            }
        }

        private bool TryMakeShape(Collider collider, out ColliderShape shape)
        {
            Transform t = collider.transform;
            Vector3 s = t.lossyScale;
            switch (collider)
            {
                case SphereCollider sphere:
                    shape = ColliderShape.Sphere(t.TransformPoint(sphere.center),
                        sphere.radius * Mathf.Max(Mathf.Abs(s.x), Mathf.Max(Mathf.Abs(s.y), Mathf.Abs(s.z))), friction);
                    return true;
                case CapsuleCollider capsule:
                {
                    Vector3 axis = capsule.direction == 0 ? Vector3.right : capsule.direction == 1 ? Vector3.up : Vector3.forward;
                    float radialScale = capsule.direction == 0 ? Mathf.Max(Mathf.Abs(s.y), Mathf.Abs(s.z))
                        : capsule.direction == 1 ? Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.z))
                        : Mathf.Max(Mathf.Abs(s.x), Mathf.Abs(s.y));
                    float axialScale = Mathf.Abs(Vector3.Dot(s, axis));
                    float radius = capsule.radius * radialScale;
                    float half = Mathf.Max(0.0f, capsule.height * 0.5f * axialScale - radius) / Mathf.Max(axialScale, 1e-6f);
                    shape = ColliderShape.Capsule(t.TransformPoint(capsule.center + axis * half),
                        t.TransformPoint(capsule.center - axis * half), radius, friction);
                    return true;
                }
                case BoxCollider box:
                    shape = ColliderShape.Box(t.TransformPoint(box.center), t.rotation,
                        0.5f * new float3(Mathf.Abs(box.size.x * s.x), Mathf.Abs(box.size.y * s.y), Mathf.Abs(box.size.z * s.z)),
                        friction);
                    return true;
                default:
                    shape = default;
                    return false;
            }
        }
    }
}
