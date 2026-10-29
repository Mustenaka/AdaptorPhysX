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
        /// <summary>Force models of the colliders in contact during the last step, and the estimators of all gathered colliders (HAP-4).</summary>
        private readonly Dictionary<Collider, ApxForceModel> _forceModels = new Dictionary<Collider, ApxForceModel>();
        private readonly Dictionary<Collider, ApxForceModelEstimator> _estimators = new Dictionary<Collider, ApxForceModelEstimator>();
        private readonly HashSet<Collider> _modelColliders = new HashSet<Collider>();
        private readonly List<Collider> _staleColliders = new List<Collider>();
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

        /// <summary>
        /// Local linear model of the force on <paramref name="collider"/>, published after each step (HAP-4):
        /// F(x) = F0 + K (x - x0), K = -k N with N the contact normals and k from the force change along the
        /// collider's motion over the last steps. False when the collider had no contact in the last step.
        /// </summary>
        public bool TryGetColliderForceModel(Collider collider, out ApxForceModel model)
        {
            CollectWrenches();
            return _forceModels.TryGetValue(collider, out model);
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

        /// <summary>
        /// Force and torque the tissue exerted on the grab in the last step (HAP-2): the elastic forces on the
        /// grabbed nodes plus their weight, with the torque about the grab target. False when not grabbing
        /// or before the first step of the grab.
        /// </summary>
        public bool TryGetGrabWrench(out ApxWrench wrench) => TryGetGrabWrench(DefaultGrab, out wrench);

        /// <summary>
        /// Force and torque the tissue exerted on grab <paramref name="grab"/> (a handle from
        /// <see cref="BeginGrab(Ray, float, Quaternion, out Vector3)"/>) in the last step, as for the other grab.
        /// </summary>
        public bool TryGetGrabWrench(int grab, out ApxWrench wrench)
        {
            wrench = default;
            return Simulation != null && _grabs.TryGetValue(grab, out ApxGrab state) &&
                   Simulation.TryGetFrameWrench(state.Frame, out wrench);
        }

        /// <summary>
        /// Force and torque the tissue exerted on pin region <c>pinColliders[pinIndex]</c> in the last step
        /// (HAP-2), with the torque about the pin collider's position. False when the region holds no nodes.
        /// A sleeping body keeps the forces of its last step.
        /// </summary>
        public bool TryGetPinWrench(int pinIndex, out ApxWrench wrench)
        {
            wrench = default;
            return Simulation != null && pinIndex >= 0 && pinIndex < pinColliders.Length &&
                   Simulation.TryGetFrameWrench(pinIndex + 1, out wrench);
        }

        /// <summary>
        /// Adds a material point (TODO SUT-1) at a world position in the tissue, e.g. a point on the rendered surface;
        /// returns its handle, or -1 when no cell holds the position. It moves with the tissue and stays on its side of
        /// later cuts. <see cref="Rebuild"/> discards the points.
        /// </summary>
        public int AddMaterialPoint(Vector3 position) => Simulation?.AddMaterialPoint(position) ?? -1;

        /// <summary>Current world position of a material point from <see cref="AddMaterialPoint"/>.</summary>
        public Vector3 MaterialPointPosition(int point) => Simulation.MaterialPointPosition(point);

        public bool RemoveMaterialPoint(int point) => Simulation != null && Simulation.RemoveMaterialPoint(point);

        /// <summary>
        /// Holds material points (<see cref="AddMaterialPoint"/>) to <paramref name="target"/> with a dynamic attachment
        /// (TODO ECO-13): each point is pulled to where it is now in the target's frame, through a constraint of
        /// <paramref name="compliance"/> m/N (0 rigid), and lets go when its force exceeds <paramref name="breakForce"/> N.
        /// The target's position and rotation are read every step; the tissue does not push it back. Returns the handle.
        /// </summary>
        public int AddAttachment(Transform target, IReadOnlyList<int> points, float compliance = 0.0f,
            float breakForce = float.PositiveInfinity)
        {
            if (Simulation == null)
            {
                return -1;
            }
            int attachment = Simulation.AddAttachment(FrameOf(target), compliance, breakForce);
            foreach (int point in points)
            {
                Simulation.AttachPoint(attachment, point);
            }
            _attachmentTargets[attachment] = target;
            _heldPoints[attachment] = Simulation.HeldPoints(attachment);
            WakeUp();
            return attachment;
        }

        /// <summary>Lets go of all points of an attachment.</summary>
        public void RemoveAttachment(int attachment)
        {
            if (Simulation != null)
            {
                // Points that let go before the attachment is removed are still reported.
                RaiseReleases(attachment);
            }
            _heldPoints.Remove(attachment);
            Simulation?.RemoveAttachment(attachment);
            _attachmentTargets.Remove(attachment);
            WakeUp();
        }

        /// <summary>Changes the force on one point, N, above which the points of an attachment let go.</summary>
        public void SetAttachmentBreakForce(int attachment, float breakForce) =>
            Simulation?.SetAttachmentBreakForce(attachment, breakForce);

        /// <summary>Points an attachment still holds (the others let go above its break force).</summary>
        public int HeldPoints(int attachment) => Simulation == null ? 0 : Simulation.HeldPoints(attachment);

        /// <summary>
        /// Force and torque the tissue exerted on an attachment's target in the last step (HAP-2), with the torque about the
        /// target's position; false when it held no point.
        /// </summary>
        public bool TryGetAttachmentWrench(int attachment, out ApxWrench wrench)
        {
            wrench = default;
            return Simulation != null && Simulation.TryGetAttachmentWrench(attachment, out wrench);
        }

        /// <summary>
        /// Adds a stitch (TODO SUT-5) between material points (<see cref="AddMaterialPoint"/>) on the two sides of an incision:
        /// a thread that only pulls, whose length is drawn by <paramref name="speed"/> m/s from the points' distance to the
        /// length of the tissue between them without stress (<see cref="StitchNaturalLength"/>), which brings the walls
        /// together. <paramref name="compliance"/> is in m/N (0 rigid). Returns the handle.
        /// </summary>
        public int AddStitch(int pointA, int pointB, float speed, float compliance = 0.0f)
        {
            if (Simulation == null)
            {
                return -1;
            }
            int stitch = Simulation.AddStitch(pointA, pointB, speed, compliance);
            WakeUp();
            return stitch;
        }

        /// <summary>Changes the length a stitch is drawn to (its natural length at first), and optionally the speed, m/s.</summary>
        public void SetStitchTarget(int stitch, float length, float speed = -1.0f)
        {
            Simulation?.SetStitchTarget(stitch, length, speed);
            WakeUp();
        }

        /// <summary>Cuts a stitch.</summary>
        public void RemoveStitch(int stitch)
        {
            Simulation?.RemoveStitch(stitch);
            WakeUp();
        }

        /// <summary>Force of a stitch's thread in the last step, N.</summary>
        public float StitchTension(int stitch) => Simulation == null ? 0.0f : Simulation.StitchTension(stitch);

        /// <summary>Length of a stitch: its points are at most this far apart.</summary>
        public float StitchLength(int stitch) => Simulation == null ? 0.0f : Simulation.StitchLength(stitch);

        /// <summary>Length of the tissue between a stitch's points without stress (pre-strain, skin layer), its default target.</summary>
        public float StitchNaturalLength(int stitch) => Simulation == null ? 0.0f : Simulation.StitchNaturalLength(stitch);

        /// <summary>Moves the attachments to their targets for the next step; true when a target moved.</summary>
        private bool MoveAttachments()
        {
            bool moved = false;
            foreach (KeyValuePair<int, Transform> pair in _attachmentTargets)
            {
                if (pair.Value == null)
                {
                    continue;
                }
                float4x4 frame = FrameOf(pair.Value);
                moved |= !frame.Equals(Simulation.AttachmentTarget(pair.Key));
                Simulation.MoveAttachment(pair.Key, frame);
            }
            return moved;
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
            UpdateForceModels();
        }

        /// <summary>
        /// Advances the force model estimator of every collider gathered for the last step, in or out of contact
        /// (a contact starts from zero force), and publishes the models of those in contact. Called once per step.
        /// </summary>
        private void UpdateForceModels()
        {
            _forceModels.Clear();
            _modelColliders.Clear();
            for (int s = 0; s < _wrenchColliders.Count; s++)
            {
                Collider collider = _wrenchColliders[s];
                // A collider of several shapes is placed by its first one.
                if (collider == null || !_modelColliders.Add(collider))
                {
                    continue;
                }
                bool contact = _wrenches.TryGetValue(collider, out ApxWrench wrench);
                _estimators.TryGetValue(collider, out ApxForceModelEstimator estimator);
                ApxForceModel model = estimator.Update(Simulation.SimulatedTime, Simulation.ShapeMeanPosition(s),
                    contact ? wrench.Force : float3.zero, contact ? wrench.ContactNormals : float3x3.zero,
                    ForceModelMinMotion * Simulation.CellSize);
                _estimators[collider] = estimator;
                if (contact)
                {
                    _forceModels[collider] = model;
                }
            }
            // Colliders that left the body's neighbourhood start afresh when they come back.
            _staleColliders.Clear();
            foreach (Collider collider in _estimators.Keys)
            {
                if (!_modelColliders.Contains(collider))
                {
                    _staleColliders.Add(collider);
                }
            }
            foreach (Collider collider in _staleColliders)
            {
                _estimators.Remove(collider);
            }
        }

        /// <summary>Motion of a collider, in cells, before its force model's stiffness is estimated again.</summary>
        private const float ForceModelMinMotion = 0.01f;

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
            _forceModels.Clear();
            _estimators.Clear();
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
                Grabs = CloneGrabs(_grabs.Values),
                NextGrab = _nextGrab,
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
            _grabs.Clear();
            foreach (ApxGrab grab in CloneGrabs(state.Grabs))
            {
                _grabs[grab.Handle] = grab;
            }
            _nextGrab = state.NextGrab;
            _restSteps = state.RestSteps;
            if (state.PinFrames.Length == _pinFrames.Length)
            {
                _pinFrames = (float4x4[])state.PinFrames.Clone();
            }
            _wrenches.Clear();
            _wrenchColliders.Clear();
            _wrenchStep = -1;
            _forceModels.Clear();
            _estimators.Clear();
            _stepBodies.Clear();
            // The events start from the restored state; the cuts since the capture are undone.
            _pendingCuts.Clear();
            _contacts.Clear();
            _attachmentHandles.Clear();
            _attachmentHandles.AddRange(_heldPoints.Keys);
            foreach (int attachment in _attachmentHandles)
            {
                _heldPoints[attachment] = Simulation.HeldPoints(attachment);
            }
        }

        private static ApxGrab[] CloneGrabs(IEnumerable<ApxGrab> grabs)
        {
            var copies = new List<ApxGrab>();
            foreach (ApxGrab grab in grabs)
            {
                copies.Add(new ApxGrab { Handle = grab.Handle, Frame = grab.Frame, Previous = grab.Previous, Target = grab.Target });
            }
            return copies.ToArray();
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
                Debug.LogError("ApxSoftBody needs a readable mesh to bake a blueprint.", this);
                return false;
            }
            BuildGeometry(mesh, out float3[] positions, out float3[] normals, out Vector2[] uvs, out int[] triangles, out bool closed);
            VoxelGrid grid = Voxelizer.Voxelize(positions, triangles, CellSizeFor(mesh), closed);
            target.Store(mesh, resolution, transform.lossyScale, positions, normals, uvs, triangles, grid);
            return true;
        }
    }

    /// <summary>State of an <see cref="ApxSoftBody"/> (see <see cref="ApxSoftBody.CaptureState"/>).</summary>
    public sealed class ApxSoftBodyState
    {
        public SoftBodyState Simulation;
        public ApxGrab[] Grabs;
        public int NextGrab;
        public int RestSteps;
        public float4x4[] PinFrames;
    }

    /// <summary>
    /// A grab of an <see cref="ApxSoftBody"/>: its handle, its kinematic frame, and the frame's pose at the start and end of
    /// the next step.
    /// </summary>
    public sealed class ApxGrab
    {
        public int Handle;
        public int Frame;
        public float4x4 Previous;
        public float4x4 Target;
    }
}
