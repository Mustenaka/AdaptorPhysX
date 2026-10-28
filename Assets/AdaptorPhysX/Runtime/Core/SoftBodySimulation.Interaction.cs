using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.NotBurstCompatible;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Interaction with instruments and other bodies: distance-field colliders, collider forces
    /// (wrenches), contact with other soft bodies, adhesion, plasticity and state snapshots.
    /// </summary>
    public sealed partial class SoftBodySimulation
    {
        /// <summary>Nodes per block of the node job; collider impulses are summed per block.</summary>
        public const int NodeBatch = 256;

        /// <summary>Stand-ins for the optional arrays of the node job, which must always be valid.</summary>
        private NativeArray<float> _noFloats;
        private NativeArray<int> _noInts;
        private NativeArray<ColliderImpulse> _noImpulses;

        // Collider impulses of the last step.
        private NativeArray<ColliderImpulse> _impulsePartial;
        private NativeArray<ColliderImpulse> _impulseTotal;
        private float _impulseDt;
        private int _impulseSubsteps;

        // Forces on the kinematic frames (HAP-2): constraint impulses per (slice, frame) and their sums.
        private NativeArray<ColliderImpulse> _framePartial;
        private NativeArray<ColliderImpulse> _frameTotal;
        private NativeArray<float3> _frameOrigins;
        /// <summary>Per frame: nodes, and mass and first mass moment in frame coordinates (see KinematicFramesJob).</summary>
        private NativeArray<int> _frameNodes;
        private NativeArray<float4> _frameMass;
        /// <summary>The kinematic nodes changed since _frameNodes and _frameMass were counted.</summary>
        private bool _framesDirty = true;
        private bool _framesActive;
        /// <summary>Per frame: its nodes in the last step (0 when it took no part), and their weight and its torque about the origin.</summary>
        private readonly int[] _frameStepNodes = new int[MaxKinematicFrames];
        private readonly float3[] _frameWeight = new float3[MaxKinematicFrames];
        private readonly float3[] _frameWeightTorque = new float3[MaxKinematicFrames];

        // Dynamic rigid bodies of the colliders (two-way coupling) and the shapes' start poses.
        private NativeArray<RigidBodyState> _rigidBodies;
        private NativeArray<ColliderShape> _shapeStart;
        private NativeArray<ColliderImpulse> _shapeConsumed;

        // Other soft bodies near this one (see BodyContacts).
        /// <summary>SDF of the grid points for the contact between bodies (see ContactSdfJob).</summary>
        private NativeArray<float> _contactSdf;
        private NativeArray<float> _nodeSdf;
        /// <summary>Nodes held by a sticky collider (see NodeStepJob.Stuck).</summary>
        private NativeArray<byte> _stuck;
        /// <summary>Overlap with other bodies left to them during the step (see NodeStepJob.PartnerShare).</summary>
        private NativeArray<float> _partnerShare;
        private NativeList<PartnerCell> _partnerCells;
        private NativeList<float3> _partnerCorner;
        private NativeList<float3> _partnerVelocity;
        private NativeList<float> _partnerSdf;
        private NativeList<float> _partnerWeight;
        /// <summary>Partner node of each copied corner, to refresh the copies between substeps.</summary>
        private NativeList<int> _partnerNode;
        /// <summary>Copied cells of each partner, in the order they were added.</summary>
        private readonly List<PartnerRange> _partnerRanges = new List<PartnerRange>();
        private float _partnerSpeed;
        private float _partnerRadius;
        private NativeArray<int> _partnerBucketStart;
        private NativeArray<int> _partnerEntries;
        private NativeArray<int> _partnerCandidate;
        private NativeArray<float> _partnerCandidateGap;
        private NativeArray<int> _partnerCandidateCount;

        private struct PartnerRange
        {
            public SoftBodySimulation Body;
            public int FirstCell;
            public int CellCount;
            /// <summary>The partner steps with this body this round, so its copy is refreshed between substeps.</summary>
            public bool Steps;
        }

        /// <summary>
        /// Bodies in contact that step together in the current round (see <see cref="SubstepGroup"/>);
        /// null when the body steps alone. Set by the pairing at the start of each round.
        /// </summary>
        internal SubstepGroup Group;

        // Plasticity: rest scaling without the plastic stretch, and the plastic stretch P per cell.
        private NativeArray<float3x3> _cellRestBase;
        private NativeArray<float3x3> _cellPlastic;

        /// <summary>Total mass in kg at the current density.</summary>
        public float TotalMass
        {
            get
            {
                CompleteStep();
                double sum = 0.0;
                for (int i = 0; i < NodeCount; i++)
                {
                    float w = InverseMass[i];
                    sum += w > 0.0f ? 1.0 / w : 0.0;
                }
                return (float)(sum * Parameters.Density / _massDensity);
            }
        }

        /// <summary>Cell copies of other bodies this body collides with in the current round of steps.</summary>
        public int PartnerCellCount => _partnerCells.IsCreated ? _partnerCells.Length : 0;

        /// <summary>
        /// Rest SDF of every node's grid point (positive outside the surface), used by the contact between
        /// bodies: unlike the grid's SDF, exact (to a few percent) also deeper than 1.5 cells inside.
        /// </summary>
        public NativeArray<float> NodeSdf => _nodeSdf;

        /// <summary>Whether plastic flow has changed the rest shape.</summary>
        public bool HasPlasticDeformation => _cellPlastic.IsCreated;

        private void CreateInteraction()
        {
            _noFloats = new NativeArray<float>(1, Allocator.Persistent);
            _noInts = new NativeArray<int>(1, Allocator.Persistent);
            _noImpulses = new NativeArray<ColliderImpulse>(1, Allocator.Persistent);
            _impulseTotal = new NativeArray<ColliderImpulse>(0, Allocator.Persistent);
            _frameTotal = new NativeArray<ColliderImpulse>(MaxKinematicFrames, Allocator.Persistent);
            _frameOrigins = new NativeArray<float3>(MaxKinematicFrames, Allocator.Persistent);
            _frameNodes = new NativeArray<int>(MaxKinematicFrames, Allocator.Persistent);
            _frameMass = new NativeArray<float4>(MaxKinematicFrames, Allocator.Persistent);
            _partnerCells = new NativeList<PartnerCell>(0, Allocator.Persistent);
            _partnerCorner = new NativeList<float3>(0, Allocator.Persistent);
            _partnerVelocity = new NativeList<float3>(0, Allocator.Persistent);
            _partnerSdf = new NativeList<float>(0, Allocator.Persistent);
            _partnerWeight = new NativeList<float>(0, Allocator.Persistent);
            _partnerNode = new NativeList<int>(0, Allocator.Persistent);
            _partnerBucketStart = new NativeArray<int>(BodyContacts.BucketCount + 1, Allocator.Persistent);
        }

        private void DisposeInteraction()
        {
            DisposeIfCreated(ref _noFloats);
            DisposeIfCreated(ref _noInts);
            DisposeIfCreated(ref _noImpulses);
            DisposeIfCreated(ref _impulsePartial);
            DisposeIfCreated(ref _impulseTotal);
            DisposeIfCreated(ref _framePartial);
            DisposeIfCreated(ref _frameTotal);
            DisposeIfCreated(ref _frameOrigins);
            DisposeIfCreated(ref _frameNodes);
            DisposeIfCreated(ref _frameMass);
            DisposeIfCreated(ref _rigidBodies);
            DisposeIfCreated(ref _shapeStart);
            DisposeIfCreated(ref _shapeConsumed);
            if (_partnerCells.IsCreated) _partnerCells.Dispose();
            if (_partnerCorner.IsCreated) _partnerCorner.Dispose();
            if (_partnerVelocity.IsCreated) _partnerVelocity.Dispose();
            if (_partnerSdf.IsCreated) _partnerSdf.Dispose();
            if (_partnerWeight.IsCreated) _partnerWeight.Dispose();
            if (_partnerNode.IsCreated) _partnerNode.Dispose();
            DisposeIfCreated(ref _partnerBucketStart);
            DisposeIfCreated(ref _partnerEntries);
            DisposeIfCreated(ref _contactSdf);
            DisposeIfCreated(ref _cellRestBase);
            DisposeIfCreated(ref _cellPlastic);
        }

        /// <summary>Per-node arrays of the interaction, rebuilt with the topology.</summary>
        private void RebuildInteractionNodes()
        {
            DisposeInteractionNodes();
            _nodeSdf = new NativeArray<float>(NodeCount, Allocator.Persistent);
            _stuck = new NativeArray<byte>(NodeCount, Allocator.Persistent);
            _partnerShare = new NativeArray<float>(NodeCount, Allocator.Persistent);
            if (!_contactSdf.IsCreated)
            {
                using var sdf = new NativeArray<float>(Grid.Sdf, Allocator.TempJob);
                _contactSdf = new NativeArray<float>(sdf.Length, Allocator.Persistent);
                new ContactSdfJob
                {
                    Sdf = sdf,
                    PointDims = Grid.PointDims,
                    CellSize = Grid.CellSize,
                    Result = _contactSdf,
                }.Run();
            }
            new NodeSdfJob
            {
                RestPositions = RestPositions,
                Sdf = _contactSdf,
                Origin = Grid.Origin,
                InverseCellSize = 1.0f / Grid.CellSize,
                PointDims = Grid.PointDims,
                NodeSdf = _nodeSdf,
            }.Schedule(NodeCount, 512).Complete();
        }

        private void DisposeInteractionNodes()
        {
            DisposeIfCreated(ref _nodeSdf);
            DisposeIfCreated(ref _stuck);
            DisposeIfCreated(ref _partnerShare);
            DisposeIfCreated(ref _partnerCandidate);
            DisposeIfCreated(ref _partnerCandidateGap);
            DisposeIfCreated(ref _partnerCandidateCount);
        }

        /// <summary>
        /// Rest scaling of cell <paramref name="c"/> with the plastic stretch applied: S^-1 P^-1.
        /// Keeps <paramref name="inverse"/> (S^-1) as the base for the plasticity job.
        /// </summary>
        private float3x3 WithPlasticity(int c, float3x3 inverse)
        {
            if (!_cellPlastic.IsCreated)
            {
                return inverse;
            }
            _cellRestBase[c] = inverse;
            return math.mul(inverse, math.inverse(_cellPlastic[c]));
        }

        /// <summary>Removes the plastic deformation: the cells return to their original rest shape.</summary>
        public void ResetPlasticity()
        {
            CompleteStep();
            if (!_cellPlastic.IsCreated)
            {
                return;
            }
            _cellRest.CopyFrom(_cellRestBase);
            DisposeIfCreated(ref _cellRestBase);
            DisposeIfCreated(ref _cellPlastic);
        }

        /// <summary>
        /// Sets up the node job's collider forces, distance fields and body contacts for a step,
        /// and schedules the body-contact candidate search.
        /// </summary>
        private JobHandle ConfigureInteraction(ref NodeStepJob node, JobHandle handle, float dt, int substeps,
            NativeArray<float> fields, NativeArray<RigidBodyState> rigidBodies)
        {
            node.Fields = fields.IsCreated && fields.Length > 0 ? fields : _noFloats;
            node.StepDt = dt;
            node.MaxBodyPush = 0.25f * CellSize;
            node.Stuck = _stuck;
            node.Sticky = false;
            for (int s = 0; s < _shapes.Length; s++)
            {
                node.Sticky |= _shapes[s].Stickiness > 0.0f;
            }

            // Dynamic rigid bodies start from their pose: no interpolation from the previous step.
            int bodies = rigidBodies.IsCreated ? rigidBodies.Length : 0;
            if (!_rigidBodies.IsCreated || _rigidBodies.Length != bodies)
            {
                DisposeIfCreated(ref _rigidBodies);
                _rigidBodies = new NativeArray<RigidBodyState>(bodies, Allocator.Persistent);
            }
            if (bodies > 0)
            {
                _rigidBodies.CopyFrom(rigidBodies);
                for (int s = 0; s < _shapes.Length; s++)
                {
                    ColliderShape shape = _shapes[s];
                    if (shape.DynamicBody > 0)
                    {
                        shape.A0 = shape.A1;
                        shape.B0 = shape.B1;
                        shape.R0 = shape.R1;
                        _shapes[s] = shape;
                    }
                }
                DisposeIfCreated(ref _shapeStart);
                DisposeIfCreated(ref _shapeConsumed);
                _shapeStart = new NativeArray<ColliderShape>(_shapes, Allocator.Persistent);
                _shapeConsumed = new NativeArray<ColliderImpulse>(_shapes.Length, Allocator.Persistent);
            }

            // Collider impulses per block of nodes, summed after the step.
            int shapes = _shapes.Length;
            _impulseDt = dt;
            _impulseSubsteps = substeps;
            if (_impulseTotal.Length != shapes)
            {
                DisposeIfCreated(ref _impulseTotal);
                _impulseTotal = new NativeArray<ColliderImpulse>(shapes, Allocator.Persistent);
            }
            node.AccumulateImpulses = shapes > 0;
            if (shapes > 0)
            {
                int blocks = (NodeCount + NodeBatch - 1) / NodeBatch;
                if (!_impulsePartial.IsCreated || _impulsePartial.Length != blocks * shapes)
                {
                    DisposeIfCreated(ref _impulsePartial);
                    _impulsePartial = new NativeArray<ColliderImpulse>(blocks * shapes, Allocator.Persistent);
                }
                else
                {
                    unsafe
                    {
                        Unity.Collections.LowLevel.Unsafe.UnsafeUtility.MemClear(
                            Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(_impulsePartial),
                            (long)_impulsePartial.Length * Unity.Collections.LowLevel.Unsafe.UnsafeUtility.SizeOf<ColliderImpulse>());
                    }
                }
                node.Impulses = _impulsePartial;
            }
            else
            {
                node.Impulses = _noImpulses;
            }

            // Contact with other bodies.
            node.BodyContacts = _partnerCells.Length > 0;
            node.NodeSdf = _nodeSdf;
            node.PartnerShare = _partnerShare;
            node.PartnersSynced = false;
            if (!node.BodyContacts)
            {
                node.PartnerCandidate = _noInts;
                node.PartnerCandidateCount = _noInts;
                node.PartnerCells = _partnerCells.AsArray();
                node.PartnerCorner = _partnerCorner.AsArray();
                node.PartnerVelocity = _partnerVelocity.AsArray();
                node.PartnerSdf = _partnerSdf.AsArray();
                node.PartnerWeight = _partnerWeight.AsArray();
                return handle;
            }
            if (!_partnerCandidate.IsCreated || _partnerCandidateCount.Length != NodeCount)
            {
                DisposeIfCreated(ref _partnerCandidate);
                DisposeIfCreated(ref _partnerCandidateGap);
                DisposeIfCreated(ref _partnerCandidateCount);
                _partnerCandidate = new NativeArray<int>(NodeCount * BodyContacts.MaxCandidates, Allocator.Persistent);
                _partnerCandidateGap = new NativeArray<float>(NodeCount * BodyContacts.MaxCandidates, Allocator.Persistent);
                _partnerCandidateCount = new NativeArray<int>(NodeCount, Allocator.Persistent);
            }
            if (!_partnerEntries.IsCreated || _partnerEntries.Length < _partnerCells.Length)
            {
                DisposeIfCreated(ref _partnerEntries);
                _partnerEntries = new NativeArray<int>(math.max(_partnerCells.Length, 64) * 2, Allocator.Persistent);
            }
            unsafe
            {
                Unity.Collections.LowLevel.Unsafe.UnsafeUtility.MemClear(
                    Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(_partnerShare),
                    (long)_partnerShare.Length * sizeof(float));
            }
            // Candidates within the cell radius plus the relative motion during the step.
            float margin = (_bounds[2].x + _partnerSpeed) * dt + 0.25f * CellSize;
            float hashCell = _partnerRadius + margin;
            handle = new BuildPartnerHashJob
            {
                Cells = _partnerCells.AsArray(),
                InverseCellSize = 1.0f / hashCell,
                BucketStart = _partnerBucketStart,
                Entries = _partnerEntries,
            }.Schedule(handle);
            handle = new FindPartnerCandidatesJob
            {
                Positions = Positions,
                ContactNode = _contactNode,
                NodeSdf = _nodeSdf,
                MinNodeSdf = -0.25f * CellSize,
                Cells = _partnerCells.AsArray(),
                BucketStart = _partnerBucketStart,
                Entries = _partnerEntries,
                InverseCellSize = 1.0f / hashCell,
                Margin = margin,
                Candidate = _partnerCandidate,
                CandidateGap = _partnerCandidateGap,
                Count = _partnerCandidateCount,
            }.Schedule(NodeCount, 128, handle);
            node.PartnerCandidate = _partnerCandidate;
            node.PartnerCandidateCount = _partnerCandidateCount;
            node.PartnerCells = _partnerCells.AsArray();
            node.PartnerCorner = _partnerCorner.AsArray();
            node.PartnerVelocity = _partnerVelocity.AsArray();
            node.PartnerSdf = _partnerSdf.AsArray();
            node.PartnerWeight = _partnerWeight.AsArray();
            return handle;
        }

        /// <summary>
        /// Sets up the forces on the kinematic frames for a step (HAP-2): the constraint solve sums the
        /// constraint impulses on the kinematic nodes per (slice, frame); their own weight, which no
        /// constraint carries, is added from the frame's mass when the force is read.
        /// </summary>
        private void ConfigureFrames(ref SolveCellsJob solve, float h)
        {
            if (_framesDirty)
            {
                new KinematicFramesJob
                {
                    KinematicId = KinematicId,
                    KinematicLocal = KinematicLocal,
                    InverseMass = InverseMass,
                    Mass = _frameMass,
                    Nodes = _frameNodes,
                }.Run();
                _framesActive = false;
                for (int f = 0; f < MaxKinematicFrames; f++)
                {
                    _framesActive |= _frameNodes[f] > 0;
                }
                _framesDirty = false;
            }
            solve.AccumulateFrames = _framesActive;
            solve.KinematicId = KinematicId;
            solve.FrameOrigins = _frameOrigins;
            solve.InverseDt = 1.0f / h;
            if (!_framesActive)
            {
                solve.FrameImpulses = _noImpulses;
                System.Array.Clear(_frameStepNodes, 0, MaxKinematicFrames);
                return;
            }
            int length = _slices.Length * MaxKinematicFrames;
            if (!_framePartial.IsCreated || _framePartial.Length != length)
            {
                DisposeIfCreated(ref _framePartial);
                _framePartial = new NativeArray<ColliderImpulse>(length, Allocator.Persistent);
            }
            else
            {
                unsafe
                {
                    Unity.Collections.LowLevel.Unsafe.UnsafeUtility.MemClear(
                        Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(_framePartial),
                        (long)_framePartial.Length * Unity.Collections.LowLevel.Unsafe.UnsafeUtility.SizeOf<ColliderImpulse>());
                }
            }
            solve.FrameImpulses = _framePartial;

            // Torques about each frame's origin at the end of the step. The nodes follow the frame
            // rigidly, x = M l + o, so their weight m g has the torque (M sum m l) x g.
            float massScale = math.max(Parameters.Density, 1e-6f) / _massDensity;
            float3 gravity = Parameters.Gravity;
            for (int f = 0; f < MaxKinematicFrames; f++)
            {
                _frameStepNodes[f] = _frameNodes[f];
                float4x4 end = FramesEnd[f];
                _frameOrigins[f] = end.c3.xyz;
                float4 mass = _frameMass[f] * massScale;
                _frameWeight[f] = mass.w * gravity;
                var linear = new float3x3(end.c0.xyz, end.c1.xyz, end.c2.xyz);
                _frameWeightTorque[f] = math.cross(math.mul(linear, mass.xyz), gravity);
            }
        }

        /// <summary>
        /// The nodes of kinematic frame <paramref name="frame"/> (all frames for -1) changed: their mass is
        /// counted again before the next step, and the frame reports no force until then.
        /// </summary>
        private void KinematicFramesChanged(int frame)
        {
            _framesDirty = true;
            if (frame >= 0 && frame < MaxKinematicFrames)
            {
                _frameStepNodes[frame] = 0;
            }
        }

        /// <summary>
        /// After the node job of substep <paramref name="substep"/>: the dynamic rigid bodies take the
        /// opposite of the momentum their colliders gave the nodes and move to the next substep's pose.
        /// </summary>
        private JobHandle ScheduleRigidCoupling(JobHandle handle, int substep, int substeps, float h, float dt)
        {
            if (_rigidBodies.Length == 0)
            {
                return handle;
            }
            return new RigidCouplingJob
            {
                Partial = _impulsePartial,
                ShapeStart = _shapeStart,
                Shapes = _shapes,
                Consumed = _shapeConsumed,
                Bodies = _rigidBodies,
                SubstepDt = h,
                StepDt = dt,
                Substep = substep,
                Advance = substep < substeps,
            }.Schedule(handle);
        }

        /// <summary>
        /// Correction the tissue made to dynamic rigid body <paramref name="index"/> during the last step
        /// (displacement, velocity, rotation and angular velocity on top of its free motion).
        /// </summary>
        public bool TryGetRigidBodyResponse(int index, out RigidBodyState state)
        {
            CompleteStep();
            state = default;
            if (!_rigidBodies.IsCreated || index < 0 || index >= _rigidBodies.Length)
            {
                return false;
            }
            state = _rigidBodies[index];
            return true;
        }

        /// <summary>Jobs after the last substep: impulse sums and plastic flow.</summary>
        private JobHandle ScheduleAfterSubsteps(JobHandle handle, float dt)
        {
            if (_shapes.Length > 0)
            {
                handle = new ReduceImpulseJob { Partial = _impulsePartial, Total = _impulseTotal }.Schedule(handle);
            }
            if (_step.Solve.AccumulateFrames)
            {
                handle = new ReduceImpulseJob { Partial = _framePartial, Total = _frameTotal }.Schedule(handle);
            }
            SoftBodyParameters p = Parameters;
            if (p.PlasticCreep > 0.0f || (_cellPlastic.IsCreated && p.PlasticRecovery > 0.0f))
            {
                if (!_cellPlastic.IsCreated)
                {
                    _cellRestBase = new NativeArray<float3x3>(_cellRest, Allocator.Persistent);
                    _cellPlastic = new NativeArray<float3x3>(CellCount, Allocator.Persistent);
                    for (int c = 0; c < CellCount; c++)
                    {
                        _cellPlastic[c] = float3x3.identity;
                    }
                }
                handle = new PlasticityJob
                {
                    Positions = Positions,
                    CellCopyRange = CellCopyRange,
                    CopyNodes = CopyNodes.AsArray(),
                    TetRotation = _tetRotation,
                    CellRestBase = _cellRestBase,
                    CellPlastic = _cellPlastic,
                    CellRest = _cellRest,
                    InverseCellSize = 1.0f / CellSize,
                    Yield = math.max(p.PlasticYield, 0.0f),
                    Creep = 1.0f - math.exp(-math.max(p.PlasticCreep, 0.0f) * dt),
                    Recovery = 1.0f - math.exp(-math.max(p.PlasticRecovery, 0.0f) * dt),
                    MaxDeformation = p.MaxPlasticDeformation > 0.0f ? p.MaxPlasticDeformation : float.MaxValue,
                }.Schedule(CellCount, 64, handle);
            }
            return handle;
        }

        /// <summary>
        /// Force and torque that collider <paramref name="shape"/> (index into the shapes of the last
        /// step) received from the tissue, averaged over the step. False when it had no contact.
        /// </summary>
        public bool TryGetShapeWrench(int shape, out ApxWrench wrench)
        {
            CompleteStep();
            wrench = default;
            if (shape < 0 || shape >= _impulseTotal.Length || !_shapes.IsCreated || shape >= _shapes.Length)
            {
                return false;
            }
            ColliderImpulse total = _impulseTotal[shape];
            if (total.Contacts <= 0.0f || _impulseDt <= 0.0f)
            {
                return false;
            }
            float3 origin = _shapes[shape].A1;
            wrench = new ApxWrench
            {
                Force = -total.Linear / _impulseDt,
                Torque = -total.Angular / _impulseDt,
                Origin = origin,
                ContactCentroid = total.Weight > 0.0f ? total.Centroid / total.Weight : origin,
                ContactNodes = total.Contacts / math.max(_impulseSubsteps, 1),
                ContactNormals = total.Normals,
            };
            return true;
        }

        /// <summary>
        /// Reference point of collider shape <paramref name="shape"/> averaged over the times its force was
        /// sampled in the last step (the ends of the substeps): the position the step's average force belongs to (HAP-4).
        /// </summary>
        public float3 ShapeMeanPosition(int shape)
        {
            CompleteStep();
            if (!_shapes.IsCreated || shape < 0 || shape >= _shapes.Length)
            {
                return float3.zero;
            }
            ColliderShape s = _shapes[shape];
            // A dynamic body's shape is moved substep by substep; its pose at the start of the step is kept apart.
            float3 start = s.DynamicBody > 0 && _shapeStart.IsCreated && shape < _shapeStart.Length ? _shapeStart[shape].A1 : s.A0;
            int n = math.max(_impulseSubsteps, 1);
            return math.lerp(start, s.A1, (n + 1) / (2.0f * n));
        }

        /// <summary>
        /// Force and torque the tissue exerted on kinematic frame <paramref name="frame"/> (grab: 0, pins: 1..)
        /// in the last step, averaged over the step (HAP-2): the constraint forces on the frame's nodes plus
        /// their weight, with the torque about the frame's origin at the end of the step. The contact centroid
        /// is weighted by the constraint forces; ContactNodes is the number of nodes of the frame. False when
        /// the frame held no nodes in the last step, or they changed since.
        /// </summary>
        public bool TryGetFrameWrench(int frame, out ApxWrench wrench)
        {
            CompleteStep();
            wrench = default;
            if (frame < 0 || frame >= MaxKinematicFrames || _frameStepNodes[frame] == 0 || _impulseDt <= 0.0f)
            {
                return false;
            }
            ColliderImpulse total = _frameTotal[frame];
            float3 origin = _frameOrigins[frame];
            wrench = new ApxWrench
            {
                Force = -total.Linear / _impulseDt + _frameWeight[frame],
                Torque = -total.Angular / _impulseDt + _frameWeightTorque[frame],
                Origin = origin,
                ContactCentroid = total.Weight > 0.0f ? total.Centroid / total.Weight : origin,
                ContactNodes = _frameStepNodes[frame],
            };
            return true;
        }

        /// <summary>Forgets the other bodies of the previous round (see <see cref="AddPartner"/>).</summary>
        public void ClearPartners()
        {
            CompleteStep();
            _partnerCells.Clear();
            _partnerCorner.Clear();
            _partnerVelocity.Clear();
            _partnerSdf.Clear();
            _partnerWeight.Clear();
            _partnerNode.Clear();
            _partnerRanges.Clear();
            _partnerSpeed = 0.0f;
            _partnerRadius = 0.0f;
            Group = null;
        }

        /// <summary>
        /// Copies the cell copies of <paramref name="other"/> within the region for contact during the
        /// next step. Both bodies must have completed their steps; neither is modified.
        /// </summary>
        public void AddPartner(SoftBodySimulation other, float3 regionMin, float3 regionMax, float friction)
        {
            CompleteStep();
            other.CompleteStep();
            int before = _partnerCells.Length;
            new ExtractPartnerCellsJob
            {
                Copies = other.Copies.AsArray(),
                CopyNodes = other.CopyNodes.AsArray(),
                Positions = other.Positions,
                Velocities = other.Velocities,
                NodeSdf = other._nodeSdf,
                InverseMass = other.InverseMass,
                KinematicId = other.KinematicId,
                MassScale = other._massDensity / math.max(other.Parameters.Density, 1e-6f),
                RegionMin = regionMin,
                RegionMax = regionMax,
                Friction = friction,
                Cells = _partnerCells,
                Corner = _partnerCorner,
                Velocity = _partnerVelocity,
                Sdf = _partnerSdf,
                Weight = _partnerWeight,
                Nodes = _partnerNode,
            }.Run();
            if (_partnerCells.Length > before)
            {
                _partnerRanges.Add(new PartnerRange { Body = other, FirstCell = before, CellCount = _partnerCells.Length - before });
                _partnerSpeed = math.max(_partnerSpeed, other._bounds[2].x);
                _partnerRadius = math.max(_partnerRadius, 0.87f * other.CellSize);
                for (int c = before; c < _partnerCells.Length; c++)
                {
                    _partnerRadius = math.max(_partnerRadius, _partnerCells[c].Radius);
                }
            }
        }

        /// <summary>The body rests this round: its group does not wait for it.</summary>
        public void SkipStep() => Group?.Arrive(this);

        internal bool HasBegunStep => _begun;
        internal int BegunSubsteps => _step.Substeps;
        internal float BegunDt => _step.Dt;

        /// <summary>
        /// Before the substeps of a group are scheduled. With <paramref name="synchronised"/> substeps the
        /// copies of the partners in <paramref name="stepping"/> are refreshed between the substeps
        /// (<see cref="ScheduleRefreshPartners"/>), so the contact sees each partner's solved positions of the
        /// substep and both bodies resolve their share of the same overlap. Partners that do not step now
        /// (resting, or not begun when this body had to go ahead) stay where they are: obstacles that
        /// take no share of the overlap.
        /// </summary>
        internal void PrepareSubsteps(List<SoftBodySimulation> stepping, bool synchronised)
        {
            _step.Node.PartnersSynced = synchronised;
            for (int r = 0; r < _partnerRanges.Count; r++)
            {
                PartnerRange range = _partnerRanges[r];
                range.Steps = stepping.Contains(range.Body);
                _partnerRanges[r] = range;
                if (!synchronised || range.Steps)
                {
                    continue;
                }
                int end = (range.FirstCell + range.CellCount) * Lattice.CornerCount;
                for (int k = range.FirstCell * Lattice.CornerCount; k < end; k++)
                {
                    _partnerWeight[k] = 0.0f;
                    _partnerVelocity[k] = float3.zero;
                }
            }
        }

        /// <summary>
        /// After the constraint solves of a substep of every body (<paramref name="dependency"/>): moves the
        /// copied partner cells to the partners' solved positions, which the contacts of the substep then
        /// resolve.
        /// </summary>
        internal JobHandle ScheduleRefreshPartners(JobHandle dependency)
        {
            if (!_step.Node.PartnersSynced)
            {
                return dependency;
            }
            JobHandle handle = dependency;
            foreach (PartnerRange range in _partnerRanges)
            {
                if (!range.Steps)
                {
                    continue;
                }
                handle = new RefreshPartnerCellsJob
                {
                    Nodes = _partnerNode.AsArray(),
                    Positions = range.Body.Positions,
                    Previous = range.Body.Previous,
                    InverseDt = 1.0f / range.Body._step.H,
                    Offset = range.FirstCell * Lattice.CornerCount,
                    Corner = _partnerCorner.AsArray(),
                    Velocity = _partnerVelocity.AsArray(),
                }.Schedule(range.CellCount * Lattice.CornerCount, 256, handle);
            }
            return handle;
        }

        /// <summary>Copy of the complete simulation state, including the cut topology.</summary>
        public SoftBodyState CaptureState()
        {
            CompleteStep();
            return new SoftBodyState
            {
                Severed = Severed.ToArray(),
                EdgeT = EdgeT.ToArray(),
                EdgeNormal = EdgeNormal.ToArray(),
                CellCopyRange = CellCopyRange.ToArray(),
                Copies = Copies.AsArray().ToArray(),
                CopyNodes = CopyNodes.AsArray().ToArray(),
                Positions = Positions.ToArray(),
                Previous = Previous.ToArray(),
                Velocities = Velocities.ToArray(),
                RestPositions = RestPositions.ToArray(),
                InverseMass = InverseMass.ToArray(),
                KinematicId = KinematicId.ToArray(),
                KinematicLocal = KinematicLocal.ToArray(),
                NodeReal = NodeReal.ToArray(),
                PieceId = PieceId.ToArray(),
                StrokeSide = StrokeSide.ToArray(),
                StrokeTravel = StrokeTravel.ToArray(),
                StrokeDistance = StrokeDistance.ToArray(),
                StrokeStartSevered = StrokeStartSevered.ToArray(),
                StrokeKept = StrokeKept.ToArray(),
                ContactNode = _contactNode.ToArray(),
                Stuck = _stuck.ToArray(),
                TetRotation = _tetRotation.ToArray(),
                FramesStart = FramesStart.ToArray(),
                FramesEnd = FramesEnd.ToArray(),
                CellRest = _cellRest.ToArray(),
                CellScale = _cellScale.ToArray(),
                CellRestBase = _cellRestBase.IsCreated ? _cellRestBase.ToArray() : null,
                CellPlastic = _cellPlastic.IsCreated ? _cellPlastic.ToArray() : null,
                CellRestParameters = _cellRestParameters,
                MassDensity = _massDensity,
                SimulatedTime = SimulatedTime,
                Parameters = Parameters,
                Stroke = _stroke,
                StrokeEnded = _strokeEnded,
                StrokeTravelSum = _strokeTravel,
                StrokeNormal = _strokeNormal,
                StrokeMotion = _strokeMotion,
                Bounds = _bounds.ToArray(),
                MaterialPoints = _materialPoints.AsArray().ToArray(),
            };
        }

        /// <summary>
        /// Returns to a state from <see cref="CaptureState"/> of this simulation. Stepping on gives the
        /// same results as stepping on from the capture. Renderers rebuild their topology.
        /// </summary>
        public void RestoreState(SoftBodyState state)
        {
            CompleteStep();
            if (state.Severed.Length != CellCount)
            {
                throw new System.ArgumentException("The state belongs to another lattice.", nameof(state));
            }
            Severed.CopyFrom(state.Severed);
            StrokeStartSevered.CopyFrom(state.StrokeStartSevered);
            StrokeKept.CopyFrom(state.StrokeKept);
            EdgeT.CopyFrom(state.EdgeT);
            EdgeNormal.CopyFrom(state.EdgeNormal);
            CellCopyRange.CopyFrom(state.CellCopyRange);
            Copies.CopyFromNBC(state.Copies);
            CopyNodes.CopyFromNBC(state.CopyNodes);

            DisposeNodes();
            int count = state.Positions.Length;
            Positions = new NativeArray<float3>(state.Positions, Allocator.Persistent);
            Previous = new NativeArray<float3>(state.Previous, Allocator.Persistent);
            Velocities = new NativeArray<float3>(state.Velocities, Allocator.Persistent);
            RestPositions = new NativeArray<float3>(state.RestPositions, Allocator.Persistent);
            InverseMass = new NativeArray<float>(state.InverseMass, Allocator.Persistent);
            KinematicId = new NativeArray<int>(state.KinematicId, Allocator.Persistent);
            KinematicLocal = new NativeArray<float3>(state.KinematicLocal, Allocator.Persistent);
            NodeReal = new NativeArray<byte>(state.NodeReal, Allocator.Persistent);
            PieceId = new NativeArray<int>(state.PieceId, Allocator.Persistent);
            StrokeSide = new NativeArray<sbyte>(state.StrokeSide, Allocator.Persistent);
            StrokeTravel = new NativeArray<float>(state.StrokeTravel, Allocator.Persistent);
            StrokeDistance = new NativeArray<float>(state.StrokeDistance, Allocator.Persistent);
            _contactNode = new NativeArray<byte>(state.ContactNode, Allocator.Persistent);
            _weights = new NativeArray<float>(count, Allocator.Persistent);
            AllocateContacts();
            DisposeIfCreated(ref _tetRotation);
            _tetRotation = new NativeArray<quaternion>(state.TetRotation, Allocator.Persistent);
            FramesStart.CopyFrom(state.FramesStart);
            FramesEnd.CopyFrom(state.FramesEnd);
            _cellRest.CopyFrom(state.CellRest);
            _cellScale.CopyFrom(state.CellScale);
            DisposeIfCreated(ref _cellRestBase);
            DisposeIfCreated(ref _cellPlastic);
            if (state.CellPlastic != null)
            {
                _cellRestBase = new NativeArray<float3x3>(state.CellRestBase, Allocator.Persistent);
                _cellPlastic = new NativeArray<float3x3>(state.CellPlastic, Allocator.Persistent);
            }
            _cellRestParameters = state.CellRestParameters;
            _massDensity = state.MassDensity;
            SimulatedTime = state.SimulatedTime;
            Parameters = state.Parameters;
            _stroke = state.Stroke;
            _strokeEnded = state.StrokeEnded;
            _strokeTravel = state.StrokeTravelSum;
            _strokeNormal = state.StrokeNormal;
            _strokeMotion = state.StrokeMotion;
            _bounds.CopyFrom(state.Bounds);
            _materialPoints.Clear();
            if (state.MaterialPoints != null)
            {
                _materialPoints.CopyFromNBC(state.MaterialPoints);
            }
            DisposeIfCreated(ref _impulsePartial);
            RebuildInteractionNodes();
            _stuck.CopyFrom(state.Stuck);
            KinematicFramesChanged(-1);
            System.Array.Clear(_frameStepNodes, 0, MaxKinematicFrames);
            TopologyVersion++;
        }
    }

    /// <summary>Complete state of a <see cref="SoftBodySimulation"/> (see <see cref="SoftBodySimulation.CaptureState"/>).</summary>
    public sealed class SoftBodyState
    {
        public ushort[] Severed;
        public float[] EdgeT;
        public float3[] EdgeNormal;
        public int2[] CellCopyRange;
        public CellCopy[] Copies;
        public int[] CopyNodes;
        public float3[] Positions;
        public float3[] Previous;
        public float3[] Velocities;
        public float3[] RestPositions;
        public float[] InverseMass;
        public int[] KinematicId;
        public float3[] KinematicLocal;
        public byte[] NodeReal;
        public int[] PieceId;
        public sbyte[] StrokeSide;
        public ushort[] StrokeStartSevered;
        public ushort[] StrokeKept;
        public float[] StrokeTravel;
        public float[] StrokeDistance;
        public byte[] ContactNode;
        public byte[] Stuck;
        public quaternion[] TetRotation;
        public float4x4[] FramesStart;
        public float4x4[] FramesEnd;
        public float3x3[] CellRest;
        public float[] CellScale;
        public float3x3[] CellRestBase;
        public float3x3[] CellPlastic;
        public float4 CellRestParameters;
        public float MassDensity;
        public double SimulatedTime;
        public SoftBodyParameters Parameters;
        public int Stroke;
        public bool StrokeEnded;
        public float StrokeTravelSum;
        public float3 StrokeNormal;
        public float3 StrokeMotion;
        public float3[] Bounds;
        /// <summary>Material points by handle (TODO SUT-1).</summary>
        public MaterialPoint[] MaterialPoints;

        public int NodeCount => Positions.Length;
    }
}
