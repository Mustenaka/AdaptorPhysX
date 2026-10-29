using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Cuttable volumetric soft body. Voxelizes the mesh on Start, simulates
    /// it in FixedUpdate and deforms the rendered mesh in LateUpdate.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    [DisallowMultipleComponent]
    public sealed partial class ApxSoftBody : MonoBehaviour
    {
        public static readonly List<ApxSoftBody> Active = new List<ApxSoftBody>();

        [Header("Shape")]
        [Tooltip("Surface mesh, preferably closed (holes are closed in the lattice only); defaults to the MeshFilter mesh.")]
        public Mesh sourceMesh;
        [Tooltip("Lattice cells along the longest axis. A change at runtime takes effect on Rebuild().")]
        [Range(4, 64)] public int resolution = 16;
        [Tooltip("Precomputed surface and lattice (Bake Blueprint in the inspector). Used when baked for this " +
                 "mesh, resolution and scale; Start then skips subdivision and voxelization.")]
        public ApxSoftBodyBlueprint blueprint;

        [Header("Material")]
        [Tooltip("Young's modulus in Pa.")]
        public float youngsModulus = 3.0e4f;
        [Range(0.0f, 0.49f)] public float poissonRatio = 0.45f;
        [Tooltip("kg/m^3")]
        public float density = 1000.0f;
        [Tooltip("Velocity damping per second.")]
        [Min(0.0f)] public float damping = 1.0f;
        [Tooltip("Rest-shape contraction. Pinned tissue is pre-tensioned and incisions gape.")]
        [Range(0.0f, 0.3f)] public float preStrain;

        [Header("Skin layer")]
        [Tooltip("Thickness of the stiffer, pre-tensioned skin layer below the surface in metres (0 = none).")]
        [Min(0.0f)] public float skinThickness;
        [Tooltip("Young's modulus of the skin layer in Pa.")]
        public float skinYoungsModulus = 3.0e5f;
        [Tooltip("Rest-shape contraction of the skin layer; the bulk keeps preStrain.")]
        [Range(0.0f, 0.3f)] public float skinPreStrain = 0.15f;

        [Header("Plasticity")]
        [Tooltip("Elastic strain above which the rest shape flows (Frobenius norm of U - I).")]
        [Min(0.0f)] public float plasticYield = 0.1f;
        [Tooltip("Rate of plastic flow in 1/s; 0 disables plasticity.")]
        [Min(0.0f)] public float plasticCreep;
        [Tooltip("Rate in 1/s at which the plastic deformation returns to the original shape.")]
        [Min(0.0f)] public float plasticRecovery;
        [Tooltip("Largest plastic deformation (Frobenius norm of P - I); 0 = unlimited.")]
        [Min(0.0f)] public float maxPlasticDeformation = 0.5f;

        [Header("Solver")]
        [Range(1, 64)] public int substeps = 16;
        [Tooltip("Simulation steps allowed per rendered frame. When a frame takes longer than the fixed " +
                 "time step, extra FixedUpdate calls are skipped (slow motion) instead of piling up. " +
                 "A body near dynamic Rigidbodies steps in every FixedUpdate, as the physics engine does.")]
        [Range(1, 8)] public int maxStepsPerFrame = 1;
        public bool useGravity = true;
        [Tooltip("Stop stepping once the body is at rest, until a collider, pin, grab or cut disturbs it.")]
        public bool allowSleep = true;

        [Header("Collision")]
        public LayerMask collisionLayers = ~0;
        [Range(0.0f, 2.0f)] public float friction = 0.6f;
        [Tooltip("Distance kept between the surface and colliders; negative lets the surface enter them.")]
        public float collisionMargin;
        [Tooltip("Colliders also push the cells at the surface where they enter them deepest, so instruments thinner " +
                 "than a cell do not pass between the lattice nodes. Off: colliders push the nodes only.")]
        public bool surfaceContact = true;
        [Tooltip("Contact between cut-off pieces, between folded parts of the body and between the walls of an incision.")]
        public bool selfCollision = true;
        [Tooltip("Contact with other ApxSoftBody objects whose collision layers include this body's layer.")]
        public bool collideWithBodies = true;
        [Tooltip("Two-way coupling: non-kinematic Rigidbodies of the colliders the tissue touches are pushed back by it.")]
        public bool twoWayCoupling = true;

        [Header("Attachments")]
        [Tooltip("Nodes inside these colliders at Start follow the collider's transform. A change at runtime takes effect on Rebuild().")]
        public Collider[] pinColliders = new Collider[0];

        [Header("Rendering")]
        [Tooltip("Material of cut surfaces (submesh 1).")]
        public Material cutMaterial;

        /// <summary>Wall-clock duration of the stages of <see cref="Initialize"/>, in milliseconds.</summary>
        public struct InitializeTimings
        {
            /// <summary>Reading, scaling and subdividing the surface mesh.</summary>
            public float Prepare;
            public float Voxelize;
            /// <summary>Simulation arrays, initial topology and pins.</summary>
            public float Simulation;
            /// <summary>Surface embedding and the first mesh update.</summary>
            public float Surface;

            public float Total => Prepare + Voxelize + Simulation + Surface;
        }

        public SoftBodySimulation Simulation { get; private set; }
        public SurfaceMesh Surface { get; private set; }
        public InitializeTimings LastInitializeTimings { get; private set; }
        /// <summary>Whether the grab of <see cref="BeginGrab(Ray, float, out Vector3)"/> holds the tissue.</summary>
        public bool IsGrabbing => _grabs.ContainsKey(DefaultGrab);
        /// <summary>Handles of the grabs holding the tissue (0: the grab of BeginGrab(Ray, float, out Vector3)).</summary>
        public IReadOnlyCollection<int> GrabHandles => _grabs.Keys;
        /// <summary>True while the body is at rest and skips its steps.</summary>
        public bool IsSleeping => _restSteps >= SleepSteps;

        /// <summary>Steps below <see cref="SleepSpeed"/> without disturbance before the body sleeps.</summary>
        public const int SleepSteps = 30;
        /// <summary>Node speed (m/s) under which the body counts as at rest.</summary>
        public const float SleepSpeed = 2e-3f;

        private readonly Dictionary<int, ColliderShape> _previousShapes = new Dictionary<int, ColliderShape>();
        private readonly List<ColliderShape> _shapes = new List<ColliderShape>();
        private readonly Collider[] _overlap = new Collider[64];
        private float4x4[] _pinFrames;
        /// <summary>Grabs by handle (see <see cref="ApxGrab"/>); handle 0 is the grab of BeginGrab(Ray, float, out Vector3).</summary>
        private readonly SortedDictionary<int, ApxGrab> _grabs = new SortedDictionary<int, ApxGrab>();
        private int _nextGrab = DefaultGrab + 1;
        private const int DefaultGrab = 0;
        /// <summary>Targets of the dynamic attachments by handle (<see cref="AddAttachment"/>).</summary>
        private readonly SortedDictionary<int, Transform> _attachmentTargets = new SortedDictionary<int, Transform>();
        private MeshRenderer _renderer;
        private int _stepFrame = -1;
        private int _stepsThisFrame;
        private int _restSteps;
        private int _shapeCount;
        /// <summary>Mesh and resolution of the current simulation; the MeshFilter then holds the generated surface.</summary>
        private Mesh _initializedMesh;
        private int _initializedResolution;

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

        private void FixedUpdate()
        {
            if (_stepFrame != Time.frameCount)
            {
                _stepFrame = Time.frameCount;
                _stepsThisFrame = 0;
            }
            // A body touching dynamic Rigidbodies keeps pace with the physics engine, which steps them
            // anyway: a skipped step leaves them without the tissue's support for that step (a ball
            // dropped on tissue fell through during the long first frames of Play mode).
            if (_stepsThisFrame++ < maxStepsPerFrame || _stepBodies.Count > 0)
            {
                Simulate(Time.fixedDeltaTime);
            }
            else
            {
                // Rigid bodies still receive the response of the last step.
                ApplyRigidResponse();
            }
        }

        private void LateUpdate() => UpdateRender();

        private void OnValidate()
        {
            WakeUp();
            if (Simulation != null && resolution != _initializedResolution)
            {
                Debug.LogWarning($"{name}: resolution changed to {resolution}; it takes effect on Rebuild(), which discards the cuts.", this);
            }
        }

        private void OnDestroy() => Release();

        /// <summary>Voxelizes the mesh and creates the simulation. Called by Start; callable in edit mode.</summary>
        public void Initialize()
        {
            if (Simulation != null)
            {
                return;
            }
            _renderer = GetComponent<MeshRenderer>();
            Mesh mesh = sourceMesh != null ? sourceMesh : _initializedMesh != null ? _initializedMesh : GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null || !mesh.isReadable)
            {
                Debug.LogError("ApxSoftBody needs a readable mesh.", this);
                enabled = false;
                return;
            }

            var timer = System.Diagnostics.Stopwatch.StartNew();
            var timings = new InitializeTimings();
            float3[] restPositions;
            float3[] restNormals;
            Vector2[] restUvs;
            int[] restTriangles;
            VoxelGrid grid;
            if (blueprint != null && blueprint.Matches(mesh, resolution, transform.lossyScale))
            {
                blueprint.Load(out restPositions, out restNormals, out restUvs, out restTriangles, out grid);
                timings.Prepare = Lap(timer);
            }
            else
            {
                if (blueprint != null)
                {
                    Debug.LogWarning($"{name}: the blueprint was baked for other settings and is not used; bake it again.", this);
                }
                BuildGeometry(mesh, out restPositions, out restNormals, out restUvs, out restTriangles, out bool closed);
                timings.Prepare = Lap(timer);
                grid = Voxelizer.Voxelize(restPositions, restTriangles, CellSizeFor(mesh), closed);
                timings.Voxelize = Lap(timer);
            }
            float4x4 restToWorld = float4x4.TRS(transform.position, transform.rotation, new float3(1.0f));
            Simulation = new SoftBodySimulation(grid, BuildParameters(), restToWorld);
            FinishInitialize(mesh, timer, timings, restPositions, restNormals, restUvs, restTriangles);
        }

        /// <summary>
        /// Surface mesh in rest space (scaled), subdivided to at most half a cell per edge.
        /// <paramref name="closed"/> is false when the mesh has boundary edges (holes).
        /// </summary>
        private void BuildGeometry(Mesh mesh, out float3[] restPositions, out float3[] restNormals, out Vector2[] restUvs,
            out int[] restTriangles, out bool closed)
        {
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
            float3[] positionArray = positions.ToArray();
            int[] triangleArray = triangles.ToArray();
            MeshUtil.CountOpenEdges(positionArray, triangleArray, out int boundaryEdges, out _);
            closed = boundaryEdges == 0;
            if (!closed)
            {
                Debug.LogWarning($"{name}: the mesh has {boundaryEdges} boundary edges (holes). The lattice closes them " +
                                 "(inside and outside by the generalized winding number); the rendered surface keeps them.", this);
            }
            // Normals only shade the surface. A mesh without usable ones (none, or computed by the importer from
            // triangles too small for it) gets the normals of its triangles in rest space.
            float agreement = MeshUtil.NormalAgreement(positionArray, normals.ToArray(), triangleArray, out float3[] geometric);
            if (agreement < MinNormalAgreement)
            {
                Debug.LogWarning($"{name}: the mesh normals do not follow its triangles (agreement {agreement:0.00}); " +
                                 "they are recalculated from the triangles.", this);
                normals.Clear();
                normals.AddRange(geometric);
            }

            MeshUtil.Subdivide(positions, normals, uvs, triangles, 0.5f * CellSizeFor(mesh));
            restPositions = positions.ToArray();
            restNormals = normals.ToArray();
            restUvs = uvs.ToArray();
            restTriangles = triangles.ToArray();
        }

        /// <summary>Below this mean cosine between the mesh normals and its triangles' normals, the normals are recalculated.</summary>
        public const float MinNormalAgreement = 0.5f;

        private float CellSizeFor(Mesh mesh)
        {
            Vector3 size = Vector3.Scale(mesh.bounds.size, transform.lossyScale);
            return Mathf.Max(size.x, Mathf.Max(size.y, size.z)) / resolution;
        }

        private void FinishInitialize(Mesh mesh, System.Diagnostics.Stopwatch timer, InitializeTimings timings,
            float3[] restPositions, float3[] restNormals, Vector2[] restUvs, int[] restTriangles)
        {
            _pinFrames = new float4x4[pinColliders.Length];
            for (int p = 0; p < pinColliders.Length; p++)
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
            timings.Simulation = Lap(timer);

            Surface = new SurfaceMesh(Simulation, restPositions, restNormals, restUvs, restTriangles);
            GetComponent<MeshFilter>().sharedMesh = Surface.Mesh;
            if (cutMaterial != null)
            {
                Material skin = _renderer.sharedMaterial;
                _renderer.sharedMaterials = new[] { skin, cutMaterial };
            }
            Surface.Update(transform.worldToLocalMatrix);
            timings.Surface = Lap(timer);
            LastInitializeTimings = timings;
            _initializedMesh = mesh;
            _initializedResolution = resolution;
            ResetEvents();
            Active.Add(this);
        }

        private static float Lap(System.Diagnostics.Stopwatch timer)
        {
            float ms = (float)timer.Elapsed.TotalMilliseconds;
            timer.Restart();
            return ms;
        }

        /// <summary>Advances the body by one fixed step of length <paramref name="dt"/>.</summary>
        public void Simulate(float dt)
        {
            if (Simulation == null)
            {
                return;
            }
            SoftBodyWorld.BeginStep(this);
            Simulation.CompleteStep();
            RaiseStepEvents();
            ApplyRigidResponse();
            Simulation.Parameters = BuildParameters();
            bool disturbed = GatherColliders() || _grabs.Count > 0;
            disturbed |= MoveAttachments() || Simulation.StitchesDrawing;

            foreach (ApxGrab grab in _grabs.Values)
            {
                Simulation.FramesStart[grab.Frame] = grab.Previous;
                Simulation.FramesEnd[grab.Frame] = grab.Target;
                grab.Previous = grab.Target;
            }
            for (int p = 0; p < _pinFrames.Length; p++)
            {
                if (pinColliders[p] == null)
                {
                    continue;
                }
                float4x4 frame = FrameOf(pinColliders[p].transform);
                disturbed |= !frame.Equals(_pinFrames[p]);
                Simulation.FramesStart[p + 1] = _pinFrames[p];
                Simulation.FramesEnd[p + 1] = frame;
                _pinFrames[p] = frame;
            }

            _restSteps = disturbed || !allowSleep || Simulation.MaxSpeed > SleepSpeed ? 0 : _restSteps + 1;
            RaiseSleep();
            if (IsSleeping)
            {
                RaiseCuts();
                Simulation.SkipStep();
                return;
            }

            // Scheduled only: bodies step in parallel and complete in LateUpdate (or on the next access).
            using var shapes = new NativeArray<ColliderShape>(_shapes.ToArray(), Allocator.Temp);
            PackFields();
            _wrenchColliders.Clear();
            _wrenchColliders.AddRange(_shapeColliders);
            _stepBodies.Clear();
            _stepBodies.AddRange(_gatheredBodies);
            using var bodies = new NativeArray<RigidBodyState>(_gatheredStates.ToArray(), Allocator.Temp);
            Simulation.ScheduleStep(dt, substeps, shapes, _fieldBuffer, bodies);
        }

        /// <summary>Deforms the rendered mesh to the current node positions and raises the events of the step.</summary>
        public void UpdateRender()
        {
            Simulation?.CompleteStep();
            RaiseStepEvents();
            Surface?.Update(transform.worldToLocalMatrix);
        }

        /// <summary>Releases native memory. Called by OnDestroy.</summary>
        public void Release()
        {
            Active.Remove(this);
            ReleaseInteraction();
            Surface?.Dispose();
            Simulation?.Dispose();
            Surface = null;
            Simulation = null;
        }

        /// <summary>
        /// Voxelizes again with the current settings (resolution, mesh, pin colliders) and restarts
        /// from the rest shape. Cuts and the grab are discarded; the pins are taken from pinColliders again.
        /// </summary>
        public void Rebuild()
        {
            Release();
            _grabs.Clear();
            _attachmentTargets.Clear();
            _restSteps = 0;
            Initialize();
        }

        /// <summary>
        /// Cuts along the quad swept by a blade edge from (a0,b0) to (a1,b1). Returns severed lattice
        /// edges. Consecutive sweeps of one blade stroke pass the same non-zero
        /// <paramref name="stroke"/> (see <see cref="SoftBodySimulation.Cut"/>).
        /// </summary>
        public int Cut(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1, int stroke = 0)
        {
            // The simulation ends its stroke when another one cuts (SoftBodySimulation.Cut).
            EndOtherStrokes(stroke);
            int severed = Simulation == null ? 0 : Simulation.Cut(a0, b0, a1, b1, stroke);
            if (severed > 0)
            {
                WakeUp();
                RecordCut(stroke, severed);
            }
            return severed;
        }

        /// <summary>Ends a stroke of <see cref="Cut"/> (see <see cref="SoftBodySimulation.EndStroke"/>); returns the restored edges.</summary>
        public int EndStroke(int stroke)
        {
            int healed = Simulation == null ? 0 : Simulation.EndStroke(stroke);
            if (healed > 0)
            {
                WakeUp();
            }
            RecordStrokeEnd(stroke);
            return healed;
        }

        /// <summary>Resumes stepping a sleeping body.</summary>
        public void WakeUp() => _restSteps = 0;

        /// <summary>
        /// Grabs the material under the ray within <paramref name="radius"/> (rest space): the grab of
        /// <see cref="MoveGrab(Vector3)"/>, <see cref="EndGrab()"/> and <c>TryGetGrabWrench(out ApxWrench)</c>.
        /// A grab of these already held is let go first.
        /// </summary>
        public bool BeginGrab(Ray ray, float radius, out Vector3 point)
        {
            EndGrab(DefaultGrab);
            return StartGrab(DefaultGrab, SoftBodySimulation.GrabFrame, ray, radius, quaternion.identity, out point);
        }

        /// <summary>
        /// Grabs the material under the ray within <paramref name="radius"/> (rest space) with a further grab, held in a
        /// frame at the picked point turned by <paramref name="rotation"/>: the grabbed tissue follows the frame's position
        /// and rotation (<see cref="MoveGrab(int, Vector3, Quaternion)"/>). Returns the grab's handle, or -1 when the ray
        /// picks no tissue. Any number of grabs can hold a body at once; each has a kinematic frame after the pin regions'.
        /// </summary>
        public int BeginGrab(Ray ray, float radius, Quaternion rotation, out Vector3 point)
        {
            int frame = pinColliders.Length + 1;
            while (UsesFrame(frame))
            {
                frame++;
            }
            int handle = _nextGrab;
            if (!StartGrab(handle, frame, ray, radius, rotation, out point))
            {
                return -1;
            }
            _nextGrab++;
            return handle;
        }

        /// <summary>Moves the grab of <see cref="BeginGrab(Ray, float, out Vector3)"/>; its rotation stays.</summary>
        public void MoveGrab(Vector3 target)
        {
            if (_grabs.TryGetValue(DefaultGrab, out ApxGrab grab))
            {
                grab.Target = float4x4.TRS(target, new quaternion(grab.Target), new float3(1.0f));
            }
        }

        /// <summary>Moves and turns the frame of a grab; the tissue follows during the next step.</summary>
        public void MoveGrab(int grab, Vector3 position, Quaternion rotation)
        {
            if (_grabs.TryGetValue(grab, out ApxGrab state))
            {
                state.Target = float4x4.TRS(position, rotation, new float3(1.0f));
            }
        }

        /// <summary>Lets go of the grab of <see cref="BeginGrab(Ray, float, out Vector3)"/>.</summary>
        public void EndGrab() => EndGrab(DefaultGrab);

        /// <summary>Lets go of a grab; its handle is not used again.</summary>
        public void EndGrab(int grab)
        {
            if (!_grabs.TryGetValue(grab, out ApxGrab state))
            {
                return;
            }
            Simulation?.Detach(state.Frame);
            _grabs.Remove(grab);
            WakeUp();
        }

        private bool StartGrab(int handle, int frame, Ray ray, float radius, quaternion rotation, out Vector3 point)
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
            float4x4 pose = float4x4.TRS(point, rotation, new float3(1.0f));
            Simulation.Attach(Simulation.ConnectedNodes(node, radius), frame, pose);
            _grabs[handle] = new ApxGrab { Handle = handle, Frame = frame, Previous = pose, Target = pose };
            return true;
        }

        private bool UsesFrame(int frame)
        {
            foreach (ApxGrab grab in _grabs.Values)
            {
                if (grab.Frame == frame)
                {
                    return true;
                }
            }
            return false;
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
            SurfaceContact = surfaceContact,
            SelfCollision = selfCollision,
            SkinThickness = skinThickness,
            SkinStiffness = Mathf.Max(skinYoungsModulus, 1.0f) / Mathf.Max(youngsModulus, 1.0f),
            SkinPreStrain = skinPreStrain,
            PlasticYield = plasticYield,
            PlasticCreep = plasticCreep,
            PlasticRecovery = plasticRecovery,
            MaxPlasticDeformation = maxPlasticDeformation,
        };

        private static float4x4 FrameOf(Transform t) => float4x4.TRS(t.position, t.rotation, new float3(1.0f));

        /// <summary>Collects the collider shapes near the body; returns true if any appeared, left or moved.</summary>
        private bool GatherColliders()
        {
            bool changed = false;
            _shapes.Clear();
            _shapeColliders.Clear();
            _gatheredBodies.Clear();
            _gatheredStates.Clear();
            _fields.Clear();
            _fieldLength = 0;
            Bounds bounds = WorldBounds;
            bounds.Expand(2.0f * Simulation.CellSize + 0.05f);
            // The body's own physics scene: the default one, or a local one (preview and multi-scene setups).
            int count = gameObject.scene.GetPhysicsScene().OverlapBox(bounds.center, bounds.extents, _overlap,
                Quaternion.identity, collisionLayers, QueryTriggerInteraction.Ignore);
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
                shape.DynamicBody = DynamicBodyOf(collider);
                int id = collider.GetInstanceID();
                if (_previousShapes.TryGetValue(id, out ColliderShape previous))
                {
                    changed |= !previous.A1.Equals(shape.A1) || !previous.B1.Equals(shape.B1) ||
                               !previous.R1.Equals(shape.R1) || previous.Radius != shape.Radius ||
                               !previous.HalfExtents.Equals(shape.HalfExtents) || !previous.FieldScale.Equals(shape.FieldScale);
                    shape.A0 = previous.A1;
                    shape.B0 = previous.B1;
                    shape.R0 = previous.R1;
                }
                else
                {
                    changed = true;
                }
                _previousShapes[id] = shape;
                _shapes.Add(shape);
                _shapeColliders.Add(collider);
            }
            changed |= _shapes.Count != _shapeCount;
            _shapeCount = _shapes.Count;
            return changed;
        }

        private bool TryMakeShape(Collider collider, out ColliderShape shape)
        {
            if (!TryMakeContactShape(collider, out shape))
            {
                return false;
            }
            if (collider.TryGetComponent(out ApxColliderMaterial material))
            {
                shape.Friction = material.friction >= 0.0f ? material.friction : shape.Friction;
                shape.Stickiness = material.stickiness;
                shape.StickDistance = material.stickDistance;
            }
            return true;
        }

        private bool TryMakeContactShape(Collider collider, out ColliderShape shape)
        {
            Transform t = collider.transform;
            Vector3 s = t.lossyScale;
            if (TryMakeFieldShape(collider, out shape))
            {
                return true;
            }
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
