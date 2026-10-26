using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Instruments and other bodies: distance-field colliders, collider forces and two-way coupling
    /// with rigid bodies, state snapshots and blueprints.
    /// </summary>
    public sealed partial class ApxSoftBody
    {
        /// <summary>Collider of each shape gathered for the next step.</summary>
        private readonly List<Collider> _shapeColliders = new List<Collider>();
        /// <summary>Collider of each shape of the last scheduled step, for its wrenches.</summary>
        private readonly List<Collider> _wrenchColliders = new List<Collider>();
        private readonly Dictionary<Collider, ApxWrench> _wrenches = new Dictionary<Collider, ApxWrench>();
        private int _wrenchStep = -1;
        /// <summary>Dynamic rigid bodies of the gathered colliders, and of the last scheduled step.</summary>
        private readonly List<Rigidbody> _gatheredBodies = new List<Rigidbody>();
        private readonly List<RigidBodyState> _gatheredStates = new List<RigidBodyState>();
        private readonly List<Rigidbody> _stepBodies = new List<Rigidbody>();
        private int _responseStep = -1;
        /// <summary>Distance fields of the gathered shapes, in buffer order, and the buffer length.</summary>
        private readonly List<DistanceField> _fields = new List<DistanceField>();
        private int _fieldLength;
        private readonly List<DistanceField> _packedFields = new List<DistanceField>();
        private NativeArray<float> _fieldBuffer;

        /// <summary>
        /// Force and torque the tissue exerted on <paramref name="collider"/> in the last step (HAP-1).
        /// False when the collider had no contact. A sleeping body keeps the wrenches of its last step.
        /// </summary>
        public bool TryGetColliderWrench(Collider collider, out ApxWrench wrench)
        {
            CollectWrenches();
            return _wrenches.TryGetValue(collider, out wrench);
        }

        /// <summary>Wrenches of all colliders in contact during the last step.</summary>
        public IReadOnlyDictionary<Collider, ApxWrench> ColliderWrenches
        {
            get
            {
                CollectWrenches();
                return _wrenches;
            }
        }

        private void CollectWrenches()
        {
            if (Simulation == null)
            {
                return;
            }
            Simulation.CompleteStep();
            if (_wrenchStep == Simulation.StepCount)
            {
                return;
            }
            _wrenchStep = Simulation.StepCount;
            _wrenches.Clear();
            for (int s = 0; s < _wrenchColliders.Count; s++)
            {
                Collider collider = _wrenchColliders[s];
                if (collider != null && Simulation.TryGetShapeWrench(s, out ApxWrench wrench))
                {
                    _wrenches[collider] = _wrenches.TryGetValue(collider, out ApxWrench other) ? other + wrench : wrench;
                }
            }
        }

        /// <summary>
        /// Two-way coupling: adds the correction the tissue made to each dynamic rigid body during the
        /// last step (displacement and rotation about the centre of mass, velocity, angular velocity)
        /// to the body's state, which the physics engine advanced meanwhile without the tissue.
        /// Called from <see cref="Simulate"/> before the colliders are gathered; once per step.
        /// </summary>
        private void ApplyRigidResponse()
        {
            CollectWrenches();
            if (Simulation == null || _responseStep == Simulation.StepCount)
            {
                return;
            }
            _responseStep = Simulation.StepCount;
            for (int i = 0; i < _stepBodies.Count; i++)
            {
                Rigidbody body = _stepBodies[i];
                if (body == null || body.isKinematic || !Simulation.TryGetRigidBodyResponse(i, out RigidBodyState state))
                {
                    continue;
                }
                // The transform, not Rigidbody.position: the colliders are gathered from transforms
                // right after this, and Rigidbody.position reaches the transform only after the next
                // physics step.
                Transform t = body.transform;
                float3 center = t.position + t.rotation * body.centerOfMass;
                quaternion turn = state.Turn;
                float3 position = center + state.Displacement + math.mul(turn, (float3)t.position - center);
                t.SetPositionAndRotation(position, math.mul(turn, (quaternion)t.rotation));
                body.velocity += (Vector3)state.DeltaVelocity;
                body.angularVelocity += (Vector3)state.DeltaAngularVelocity;
            }
        }

        /// <summary>Dynamic rigid body of a collider for two-way coupling: index + 1 into the step's bodies, or 0.</summary>
        private int DynamicBodyOf(Collider collider)
        {
            Rigidbody body = collider.attachedRigidbody;
            if (!twoWayCoupling || body == null || body.isKinematic)
            {
                return 0;
            }
            int index = _gatheredBodies.IndexOf(body);
            if (index < 0)
            {
                index = _gatheredBodies.Count;
                _gatheredBodies.Add(body);
                // From the transform, which ApplyRigidResponse may just have moved (the Rigidbody pose
                // follows at the next physics step).
                Transform t = body.transform;
                _gatheredStates.Add(RigidBodyState.Create(body.mass, body.inertiaTensor,
                    t.rotation * body.inertiaTensorRotation, t.position + t.rotation * body.centerOfMass, body.velocity,
                    body.angularVelocity, body.useGravity ? (float3)Physics.gravity : float3.zero));
            }
            return index + 1;
        }

        /// <summary>
        /// Shape of a MeshCollider (distance field baked from its mesh) or of a collider with an
        /// <see cref="ApxColliderMaterial"/> distance field.
        /// </summary>
        private bool TryMakeFieldShape(Collider collider, out ColliderShape shape)
        {
            shape = default;
            DistanceField field = null;
            if (collider.TryGetComponent(out ApxColliderMaterial material) && material.distanceField != null)
            {
                field = material.distanceField.Field;
            }
            else if (collider is MeshCollider meshCollider)
            {
                Mesh mesh = meshCollider.sharedMesh;
                if (mesh == null || !mesh.isReadable)
                {
                    return false;
                }
                field = DistanceFieldCache.Get(mesh, material != null ? material.fieldResolution : ApxColliderMaterial.DefaultFieldResolution);
            }
            if (field == null)
            {
                return false;
            }
            int index = _fields.IndexOf(field);
            int offset = 0;
            if (index < 0)
            {
                offset = _fieldLength;
                _fields.Add(field);
                _fieldLength += field.Values.Length;
            }
            else
            {
                for (int f = 0; f < index; f++)
                {
                    offset += _fields[f].Values.Length;
                }
            }
            Transform t = collider.transform;
            Vector3 s = t.lossyScale;
            shape = ColliderShape.DistanceField(t.position, t.rotation, new float3(Mathf.Abs(s.x), Mathf.Abs(s.y), Mathf.Abs(s.z)),
                offset, field, friction);
            return true;
        }

        /// <summary>Copies the gathered distance fields into the buffer the jobs read, when they changed.</summary>
        private void PackFields()
        {
            bool same = _fields.Count == _packedFields.Count && _fieldBuffer.IsCreated;
            for (int f = 0; same && f < _fields.Count; f++)
            {
                same = ReferenceEquals(_fields[f], _packedFields[f]);
            }
            if (same)
            {
                return;
            }
            if (_fieldBuffer.IsCreated)
            {
                _fieldBuffer.Dispose();
            }
            _fieldBuffer = new NativeArray<float>(math.max(_fieldLength, 1), Allocator.Persistent);
            int offset = 0;
            foreach (DistanceField field in _fields)
            {
                NativeArray<float>.Copy(field.Values, 0, _fieldBuffer, offset, field.Values.Length);
                offset += field.Values.Length;
            }
            _packedFields.Clear();
            _packedFields.AddRange(_fields);
        }

        private void ReleaseInteraction()
        {
            Simulation?.CompleteStep();
            SoftBodyWorld.Remove(this);
            if (_fieldBuffer.IsCreated)
            {
                _fieldBuffer.Dispose();
            }
            _packedFields.Clear();
            _wrenches.Clear();
            _wrenchColliders.Clear();
            _wrenchStep = -1;
            _stepBodies.Clear();
            _responseStep = -1;
        }

        /// <summary>Copy of the body's state: simulation (with cuts), grab and sleep state.</summary>
        public ApxSoftBodyState CaptureState()
        {
            if (Simulation == null)
            {
                return null;
            }
            return new ApxSoftBodyState
            {
                Simulation = Simulation.CaptureState(),
                IsGrabbing = IsGrabbing,
                GrabTarget = _grabTarget,
                GrabTargetPrevious = _grabTargetPrevious,
                RestSteps = _restSteps,
                PinFrames = (float4x4[])_pinFrames.Clone(),
            };
        }

        /// <summary>
        /// Returns to a state captured from this body (same lattice). The rendered mesh is rebuilt on
        /// the next update; the cuts made since the capture are undone.
        /// </summary>
        public void RestoreState(ApxSoftBodyState state)
        {
            if (Simulation == null || state == null)
            {
                return;
            }
            Simulation.RestoreState(state.Simulation);
            IsGrabbing = state.IsGrabbing;
            _grabTarget = state.GrabTarget;
            _grabTargetPrevious = state.GrabTargetPrevious;
            _restSteps = state.RestSteps;
            if (state.PinFrames.Length == _pinFrames.Length)
            {
                _pinFrames = (float4x4[])state.PinFrames.Clone();
            }
            _wrenches.Clear();
            _wrenchColliders.Clear();
            _wrenchStep = -1;
            _stepBodies.Clear();
        }

        /// <summary>Whether <see cref="blueprint"/> was baked for the current mesh, resolution and scale.</summary>
        public bool BlueprintMatches() => blueprint != null && blueprint.Matches(InputMesh, resolution, transform.lossyScale);

        /// <summary>The mesh the body is built from (the MeshFilter holds the generated surface once initialized).</summary>
        private Mesh InputMesh => sourceMesh != null ? sourceMesh : _initializedMesh != null ? _initializedMesh : GetComponent<MeshFilter>().sharedMesh;

        /// <summary>Subdivides and voxelizes the mesh with the current settings into <paramref name="target"/>.</summary>
        public bool BakeBlueprint(ApxSoftBodyBlueprint target)
        {
            Mesh mesh = InputMesh;
            if (mesh == null || !mesh.isReadable)
            {
                Debug.LogError("ApxSoftBody needs a readable closed mesh to bake a blueprint.", this);
                return false;
            }
            BuildGeometry(mesh, out float3[] positions, out float3[] normals, out Vector2[] uvs, out int[] triangles);
            VoxelGrid grid = Voxelizer.Voxelize(positions, triangles, CellSizeFor(mesh));
            target.Store(mesh, resolution, transform.lossyScale, positions, normals, uvs, triangles, grid);
            return true;
        }
    }

    /// <summary>State of an <see cref="ApxSoftBody"/> (see <see cref="ApxSoftBody.CaptureState"/>).</summary>
    public sealed class ApxSoftBodyState
    {
        public SoftBodyState Simulation;
        public bool IsGrabbing;
        public float3 GrabTarget;
        public float3 GrabTargetPrevious;
        public int RestSteps;
        public float4x4[] PinFrames;
    }
}
