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

        // Dynamic rigid bodies of the colliders (two-way coupling) and the shapes' start poses.
        private NativeArray<RigidBodyState> _rigidBodies;
        private NativeArray<ColliderShape> _shapeStart;
        private NativeArray<ColliderImpulse> _shapeConsumed;

        // Other soft bodies near this one (see BodyContacts).
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
        private float _partnerSpeed;
        private float _partnerRadius;
        private NativeArray<int> _partnerBucketStart;
        private NativeArray<int> _partnerEntries;
        private NativeArray<int> _partnerCandidate;
        private NativeArray<float> _partnerCandidateGap;
        private NativeArray<int> _partnerCandidateCount;

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

        /// <summary>Rest SDF of every node's grid point (positive outside the surface).</summary>
        public NativeArray<float> NodeSdf => _nodeSdf;

        /// <summary>Whether plastic flow has changed the rest shape.</summary>
        public bool HasPlasticDeformation => _cellPlastic.IsCreated;

        private void CreateInteraction()
        {
            _noFloats = new NativeArray<float>(1, Allocator.Persistent);
            _noInts = new NativeArray<int>(1, Allocator.Persistent);
            _noImpulses = new NativeArray<ColliderImpulse>(1, Allocator.Persistent);
            _impulseTotal = new NativeArray<ColliderImpulse>(0, Allocator.Persistent);
            _partnerCells = new NativeList<PartnerCell>(0, Allocator.Persistent);
            _partnerCorner = new NativeList<float3>(0, Allocator.Persistent);
            _partnerVelocity = new NativeList<float3>(0, Allocator.Persistent);
            _partnerSdf = new NativeList<float>(0, Allocator.Persistent);
            _partnerWeight = new NativeList<float>(0, Allocator.Persistent);
            _partnerBucketStart = new NativeArray<int>(BodyContacts.BucketCount + 1, Allocator.Persistent);
        }

        private void DisposeInteraction()
        {
            DisposeIfCreated(ref _noFloats);
            DisposeIfCreated(ref _noInts);
            DisposeIfCreated(ref _noImpulses);
            DisposeIfCreated(ref _impulsePartial);
            DisposeIfCreated(ref _impulseTotal);
            DisposeIfCreated(ref _rigidBodies);
            DisposeIfCreated(ref _shapeStart);
            DisposeIfCreated(ref _shapeConsumed);
            if (_partnerCells.IsCreated) _partnerCells.Dispose();
            if (_partnerCorner.IsCreated) _partnerCorner.Dispose();
            if (_partnerVelocity.IsCreated) _partnerVelocity.Dispose();
            if (_partnerSdf.IsCreated) _partnerSdf.Dispose();
            if (_partnerWeight.IsCreated) _partnerWeight.Dispose();
            DisposeIfCreated(ref _partnerBucketStart);
            DisposeIfCreated(ref _partnerEntries);
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
            using var sdf = new NativeArray<float>(Grid.Sdf, Allocator.TempJob);
            new NodeSdfJob
            {
                RestPositions = RestPositions,
                Sdf = sdf,
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
            _partnerSpeed = 0.0f;
            _partnerRadius = 0.0f;
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
            }.Run();
            if (_partnerCells.Length > before)
            {
                _partnerSpeed = math.max(_partnerSpeed, other._bounds[2].x);
                _partnerRadius = math.max(_partnerRadius, 0.87f * other.CellSize);
                for (int c = before; c < _partnerCells.Length; c++)
                {
                    _partnerRadius = math.max(_partnerRadius, _partnerCells[c].Radius);
                }
            }
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
                Parameters = Parameters,
                Stroke = _stroke,
                StrokeTravelSum = _strokeTravel,
                StrokeNormal = _strokeNormal,
                Bounds = _bounds.ToArray(),
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
            _contactNode = new NativeArray<byte>(state.ContactNode, Allocator.Persistent);
            _weights = new NativeArray<float>(count, Allocator.Persistent);
            _contactPartner = new NativeArray<int>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactDistance = new NativeArray<float>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactCount = new NativeArray<int>(count, Allocator.Persistent);
            _snapshot = new NativeArray<float3>(count, Allocator.Persistent);
            _bucketEntries = new NativeArray<int>(count, Allocator.Persistent);
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
            Parameters = state.Parameters;
            _stroke = state.Stroke;
            _strokeTravel = state.StrokeTravelSum;
            _strokeNormal = state.StrokeNormal;
            _bounds.CopyFrom(state.Bounds);
            DisposeIfCreated(ref _impulsePartial);
            RebuildInteractionNodes();
            _stuck.CopyFrom(state.Stuck);
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
        public float[] StrokeTravel;
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
        public SoftBodyParameters Parameters;
        public int Stroke;
        public float StrokeTravelSum;
        public float3 StrokeNormal;
        public float3[] Bounds;

        public int NodeCount => Positions.Length;
    }
}
