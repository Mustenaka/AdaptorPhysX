using System;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    [Serializable]
    public struct SoftBodyParameters
    {
        /// <summary>Young's modulus in Pa (soft tissue: ~1e3 - 1e5).</summary>
        public float YoungsModulus;
        public float PoissonRatio;
        /// <summary>Density in kg/m^3.</summary>
        public float Density;
        /// <summary>Velocity damping per second.</summary>
        public float Damping;
        /// <summary>Rest-shape contraction in [0, 0.5); pinned tissue is then under tension and incisions gape.</summary>
        public float PreStrain;
        public float3 Gravity;
        public float CollisionMargin;
        /// <summary>Contact between separated pieces and folded tissue.</summary>
        public bool SelfCollision;

        public static SoftBodyParameters Default => new SoftBodyParameters
        {
            YoungsModulus = 3.0e4f,
            PoissonRatio = 0.45f,
            Density = 1000.0f,
            Damping = 1.0f,
            PreStrain = 0.0f,
            Gravity = new float3(0.0f, -9.81f, 0.0f),
            CollisionMargin = 0.0f,
            SelfCollision = true,
        };
    }

    /// <summary>
    /// Cuttable lattice soft body: sparse hexahedral cells, five Neo-Hookean
    /// tetrahedra per cell copy, and virtual-node duplication for cuts.
    /// Nodes live in world space; the rest space is the voxel grid space.
    /// </summary>
    public sealed class SoftBodySimulation : IDisposable
    {
        public const int MaxKinematicFrames = 32;
        public const int GrabFrame = 0;
        /// <summary>Piece contacts are resolved every this many substeps.</summary>
        public const int ContactInterval = 2;

        private LatticeTables _tables;
        private int[] _colorSliceStart;
        private JobHandle _pending;
        private bool _hasPending;
        private Stopwatch _stepTimer;
        private NativeArray<ColliderShape> _shapes;

        // Static lattice.
        public VoxelGrid Grid { get; private set; }
        public NativeArray<int3> CellCoord;
        public NativeArray<int> GridToCell;
        public NativeArray<float> CellFill;
        private NativeArray<int> _sliceCells;
        private NativeArray<int2> _slices;
        private NativeArray<float3x3> _tetInverseRest;
        private NativeArray<float> _tetVolume;

        // Topology.
        public NativeArray<ushort> Severed;
        public NativeArray<float> EdgeT;
        public NativeArray<float3> EdgeNormal;
        public NativeArray<int2> CellCopyRange;
        public NativeList<CellCopy> Copies;
        public NativeList<int> CopyNodes;

        // Nodes.
        public NativeArray<float3> Positions;
        public NativeArray<float3> Previous;
        public NativeArray<float3> Velocities;
        public NativeArray<float3> RestPositions;
        public NativeArray<float> InverseMass;
        public NativeArray<int> KinematicId;
        public NativeArray<float3> KinematicLocal;
        /// <summary>1 when the node owns material in at least one copy; virtual-only nodes are 0.</summary>
        public NativeArray<byte> NodeReal;
        /// <summary>Material piece of each real node (-1 for virtual-only nodes).</summary>
        public NativeArray<int> PieceId;
        private NativeArray<float> _weights;
        private NativeArray<int> _contactPartner;
        private NativeArray<float> _contactDistance;
        private NativeArray<int> _contactCount;
        private NativeArray<float3> _snapshot;
        private NativeArray<int> _bucketStart;
        private NativeArray<int> _bucketEntries;

        public NativeArray<float4x4> FramesStart;
        public NativeArray<float4x4> FramesEnd;

        public SoftBodyParameters Parameters;
        public float4x4 RestToWorld { get; private set; }

        public int CellCount => CellCoord.Length;
        public int CopyCount => Copies.Length;
        public int NodeCount => Positions.Length;
        public int TetCount => Copies.Length * Lattice.TetsPerCell;
        /// <summary>Incremented on every topology change so renderers know to rebuild.</summary>
        public int TopologyVersion { get; private set; }
        public float LastTopologyMilliseconds { get; private set; }
        public float LastStepMilliseconds { get; private set; }

        public float CellSize => Grid.CellSize;

        /// <summary>World-space AABB of all nodes, updated after every step and topology change.</summary>
        public float3 BoundsMin
        {
            get
            {
                CompleteStep();
                return _bounds[0];
            }
        }

        public float3 BoundsMax
        {
            get
            {
                CompleteStep();
                return _bounds[1];
            }
        }

        private NativeArray<float3> _bounds;

        public SoftBodySimulation(VoxelGrid grid, SoftBodyParameters parameters, float4x4 restToWorld)
        {
            Grid = grid;
            Parameters = parameters;
            RestToWorld = restToWorld;
            _tables = LatticeTables.Create();

            int cellCount = grid.Cells.Length;
            CellCoord = new NativeArray<int3>(grid.Cells, Allocator.Persistent);
            CellFill = new NativeArray<float>(grid.Fill, Allocator.Persistent);
            GridToCell = new NativeArray<int>(grid.Dims.x * grid.Dims.y * grid.Dims.z, Allocator.Persistent);
            for (int i = 0; i < GridToCell.Length; i++)
            {
                GridToCell[i] = -1;
            }
            for (int c = 0; c < cellCount; c++)
            {
                int3 p = grid.Cells[c];
                GridToCell[p.x + grid.Dims.x * (p.y + grid.Dims.y * p.z)] = c;
            }

            // Slices of cells across the longest axis, coloured by parity: slices of one colour are
            // two cells apart, so they share no grid point and run in parallel; the cells of a
            // slice are solved in order (Gauss-Seidel within the slice). Two colours keep the
            // number of dependent jobs per substep low.
            int axis = grid.Dims.x >= grid.Dims.y && grid.Dims.x >= grid.Dims.z ? 0 : grid.Dims.y >= grid.Dims.z ? 1 : 2;
            var slices = new SortedDictionary<long, List<int>>();
            for (int c = 0; c < cellCount; c++)
            {
                int slice = grid.Cells[c][axis];
                long key = ((long)(slice & 1) << 32) | (uint)slice;
                if (!slices.TryGetValue(key, out List<int> list))
                {
                    slices[key] = list = new List<int>();
                }
                list.Add(c);
            }
            _colorSliceStart = new int[3];
            var sliceCells = new List<int>(cellCount);
            var sliceRanges = new List<int2>(slices.Count);
            int lastColor = -1;
            foreach (KeyValuePair<long, List<int>> slice in slices)
            {
                int color = (int)(slice.Key >> 32);
                while (lastColor < color)
                {
                    _colorSliceStart[++lastColor] = sliceRanges.Count;
                }
                sliceRanges.Add(new int2(sliceCells.Count, slice.Value.Count));
                sliceCells.AddRange(slice.Value);
            }
            while (lastColor < 2)
            {
                _colorSliceStart[++lastColor] = sliceRanges.Count;
            }
            _sliceCells = new NativeArray<int>(sliceCells.ToArray(), Allocator.Persistent);
            _slices = new NativeArray<int2>(sliceRanges.ToArray(), Allocator.Persistent);

            float restEdge = grid.CellSize * (1.0f - math.clamp(parameters.PreStrain, 0.0f, 0.49f));
            Lattice.BuildRestTets(restEdge, out float3x3[] inverseRest, out float[] volumes);
            _tetInverseRest = new NativeArray<float3x3>(inverseRest, Allocator.Persistent);
            _tetVolume = new NativeArray<float>(volumes, Allocator.Persistent);

            Severed = new NativeArray<ushort>(cellCount, Allocator.Persistent);
            EdgeT = new NativeArray<float>(cellCount * Lattice.EdgeCount, Allocator.Persistent);
            EdgeNormal = new NativeArray<float3>(cellCount * Lattice.EdgeCount, Allocator.Persistent);
            CellCopyRange = new NativeArray<int2>(cellCount, Allocator.Persistent);
            Copies = new NativeList<CellCopy>(cellCount, Allocator.Persistent);
            CopyNodes = new NativeList<int>(cellCount * 8, Allocator.Persistent);

            _bounds = new NativeArray<float3>(2, Allocator.Persistent);
            _bucketStart = new NativeArray<int>(Contacts.BucketCount + 1, Allocator.Persistent);
            FramesStart = new NativeArray<float4x4>(MaxKinematicFrames, Allocator.Persistent);
            FramesEnd = new NativeArray<float4x4>(MaxKinematicFrames, Allocator.Persistent);
            for (int i = 0; i < MaxKinematicFrames; i++)
            {
                FramesStart[i] = float4x4.identity;
                FramesEnd[i] = float4x4.identity;
            }

            RebuildTopology(false);
        }

        public void Dispose()
        {
            CompleteStep();
            _tables.Dispose();
            if (_shapes.IsCreated) _shapes.Dispose();
            DisposeIfCreated(ref CellCoord);
            DisposeIfCreated(ref GridToCell);
            DisposeIfCreated(ref CellFill);
            DisposeIfCreated(ref _sliceCells);
            DisposeIfCreated(ref _slices);
            DisposeIfCreated(ref _tetInverseRest);
            DisposeIfCreated(ref _tetVolume);
            DisposeIfCreated(ref Severed);
            DisposeIfCreated(ref EdgeT);
            DisposeIfCreated(ref EdgeNormal);
            DisposeIfCreated(ref CellCopyRange);
            if (Copies.IsCreated) Copies.Dispose();
            if (CopyNodes.IsCreated) CopyNodes.Dispose();
            DisposeNodes();
            DisposeIfCreated(ref _bounds);
            DisposeIfCreated(ref _bucketStart);
            DisposeIfCreated(ref FramesStart);
            DisposeIfCreated(ref FramesEnd);
        }

        /// <summary>Advances the body by <paramref name="dt"/> and waits for the result.</summary>
        public void Step(float dt, int substeps, NativeArray<ColliderShape> shapes)
        {
            ScheduleStep(dt, substeps, shapes);
            CompleteStep();
        }

        /// <summary>
        /// Schedules one step of <paramref name="dt"/> with small XPBD substeps. The shapes are
        /// copied, so the caller may reuse its array. Every other method completes the step first.
        /// </summary>
        public void ScheduleStep(float dt, int substeps, NativeArray<ColliderShape> shapes)
        {
            CompleteStep();
            _stepTimer = Stopwatch.StartNew();
            if (!_shapes.IsCreated || _shapes.Length != shapes.Length)
            {
                if (_shapes.IsCreated) _shapes.Dispose();
                _shapes = new NativeArray<ColliderShape>(shapes.Length, Allocator.Persistent);
            }
            _shapes.CopyFrom(shapes);
            substeps = math.max(substeps, 1);
            float h = dt / substeps;
            SoftBodyParameters p = Parameters;
            float nu = math.clamp(p.PoissonRatio, 0.0f, 0.49f);
            float mu = p.YoungsModulus / (2.0f * (1.0f + nu));
            float lambdaLame = p.YoungsModulus * nu / ((1.0f + nu) * (1.0f - 2.0f * nu));
            // Without the log term of Smith et al. the energy linearises to
            // shear mu and Lame lambda - mu, hence lambda = lambda_L + mu.
            float lambda = lambdaLame + mu;

            var solve = new SolveCellsJob
            {
                Positions = Positions,
                Weights = _weights,
                SliceCells = _sliceCells,
                Slices = _slices,
                CellCoord = CellCoord,
                CellCopyRange = CellCopyRange,
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                TetCorners = _tables.TetCorners,
                TetInverseRest = _tetInverseRest,
                TetVolume = _tetVolume,
                HydrostaticCompliance = 1.0f / (lambda * h * h),
                DeviatoricCompliance = 1.0f / (mu * h * h),
                Gamma = 1.0f + mu / lambda,
            };
            var node = new NodeStepJob
            {
                Positions = Positions,
                Previous = Previous,
                Velocities = Velocities,
                KinematicId = KinematicId,
                KinematicLocal = KinematicLocal,
                FramesStart = FramesStart,
                FramesEnd = FramesEnd,
                Shapes = _shapes,
                Weights = _weights,
                Snapshot = _snapshot,
                ContactPartner = _contactPartner,
                ContactDistance = _contactDistance,
                ContactCount = _contactCount,
                Contacts = p.SelfCollision,
                Dt = h,
                Gravity = p.Gravity,
                DampingFactor = math.max(0.0f, 1.0f - p.Damping * h),
                Margin = p.CollisionMargin,
            };

            JobHandle handle = new EffectiveMassJob
            {
                InverseMass = InverseMass,
                KinematicId = KinematicId,
                Weights = _weights,
            }.Schedule(NodeCount, 256);

            bool contacts = p.SelfCollision;
            var copy = new CopyPositionsJob { Source = Positions, Destination = _snapshot };
            if (contacts)
            {
                float searchRadius = 1.8f * CellSize;
                handle = new BuildContactHashJob
                {
                    Positions = Positions,
                    NodeReal = NodeReal,
                    InverseCellSize = 1.0f / searchRadius,
                    BucketStart = _bucketStart,
                    Entries = _bucketEntries,
                }.Schedule(handle);
                handle = new FindContactsJob
                {
                    Positions = Positions,
                    RestPositions = RestPositions,
                    NodeReal = NodeReal,
                    PieceId = PieceId,
                    BucketStart = _bucketStart,
                    Entries = _bucketEntries,
                    InverseCellSize = 1.0f / searchRadius,
                    SearchRadius = searchRadius,
                    ContactDistance = 0.8f * CellSize,
                    NeighbourRestDistance = 1.8f * CellSize,
                    Partner = _contactPartner,
                    Distance = _contactDistance,
                    Count = _contactCount,
                }.Schedule(NodeCount, 128, handle);
            }

            for (int s = 0; s <= substeps; s++)
            {
                // Finish substep s-1 (contacts, colliders, velocity) and predict substep s.
                node.Finish = s > 0;
                node.Predict = s < substeps;
                node.FinishTimePrev = (s - 1) / (float)substeps;
                node.FinishTime = s / (float)substeps;
                node.PredictTime = (s + 1) / (float)substeps;
                // Piece contacts every few substeps: each resolution costs two dependent jobs.
                node.Contacts = contacts && s > 0 && (s % ContactInterval == 0 || s == substeps);
                if (node.Contacts)
                {
                    handle = copy.Schedule(NodeCount, 512, handle);
                }
                handle = node.Schedule(NodeCount, 256, handle);
                if (s == substeps)
                {
                    break;
                }
                for (int color = 0; color < 2; color++)
                {
                    int count = _colorSliceStart[color + 1] - _colorSliceStart[color];
                    if (count == 0)
                    {
                        continue;
                    }
                    solve.SliceOffset = _colorSliceStart[color];
                    handle = solve.Schedule(count, 1, handle);
                }
            }
            handle = new BoundsJob { Positions = Positions, Bounds = _bounds }.Schedule(handle);
            _pending = handle;
            _hasPending = true;
            JobHandle.ScheduleBatchedJobs();
        }

        /// <summary>Waits for a scheduled step. Cheap when nothing is pending.</summary>
        public void CompleteStep()
        {
            if (!_hasPending)
            {
                return;
            }
            _pending.Complete();
            _hasPending = false;
            LastStepMilliseconds = (float)_stepTimer.Elapsed.TotalMilliseconds;
        }

        /// <summary>
        /// Cuts along the quad swept by a blade edge moving from (a0,b0) to (a1,b1).
        /// Returns the number of lattice edges severed; topology is rebuilt when non-zero.
        /// </summary>
        public int Cut(float3 a0, float3 b0, float3 a1, float3 b1)
        {
            CompleteStep();
            // Overlap consecutive sweeps along the motion so edges of tissue that moves
            // relative to the blade between two steps are not skipped.
            float3 motion = 0.5f * ((a1 + b1) - (a0 + b0));
            float travel = math.length(motion);
            if (travel > 1e-7f)
            {
                float3 dir = motion / travel;
                float back = 0.5f * CellSize, front = 0.1f * CellSize;
                a0 -= dir * back;
                b0 -= dir * back;
                a1 += dir * front;
                b1 += dir * front;
            }
            using var count = new NativeArray<int>(1, Allocator.TempJob);
            new CutJob
            {
                Positions = Positions,
                CellCoord = CellCoord,
                GridToCell = GridToCell,
                CellCopyRange = CellCopyRange,
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                EdgeCorners = _tables.EdgeCorners,
                Dims = Grid.Dims,
                A0 = a0, B0 = b0, A1 = a1, B1 = b1,
                Severed = Severed,
                EdgeT = EdgeT,
                EdgeNormal = EdgeNormal,
                SeveredCount = count,
            }.Run();
            int severed = count[0];
            if (severed > 0)
            {
                RebuildTopology(true);
            }
            return severed;
        }

        /// <summary>Attaches nodes to kinematic frame <paramref name="frame"/> at their current positions.</summary>
        public void Attach(IReadOnlyList<int> nodes, int frame, float4x4 frameToWorld)
        {
            CompleteStep();
            float4x4 worldToFrame = math.inverse(frameToWorld);
            FramesStart[frame] = frameToWorld;
            FramesEnd[frame] = frameToWorld;
            foreach (int n in nodes)
            {
                KinematicId[n] = frame;
                KinematicLocal[n] = math.transform(worldToFrame, Positions[n]);
            }
        }

        public void Detach(int frame)
        {
            CompleteStep();
            for (int i = 0; i < NodeCount; i++)
            {
                if (KinematicId[i] == frame)
                {
                    KinematicId[i] = -1;
                }
            }
        }

        /// <summary>Node closest to the ray within <paramref name="maxDistance"/>, preferring the nearest along the ray.</summary>
        public int PickNode(float3 origin, float3 direction, float maxDistance)
        {
            CompleteStep();
            direction = math.normalize(direction);
            int best = -1;
            float bestAlong = float.MaxValue;
            for (int i = 0; i < NodeCount; i++)
            {
                if (NodeReal[i] == 0)
                {
                    continue;
                }
                float3 d = Positions[i] - origin;
                float along = math.dot(d, direction);
                if (along <= 0.0f)
                {
                    continue;
                }
                float off = math.length(d - along * direction);
                if (off <= maxDistance && along < bestAlong)
                {
                    bestAlong = along;
                    best = i;
                }
            }
            return best;
        }

        /// <summary>
        /// Nodes within rest distance <paramref name="radius"/> of <paramref name="seed"/> that are
        /// connected to it through material (so a grab never crosses an incision).
        /// </summary>
        public List<int> ConnectedNodes(int seed, float radius)
        {
            CompleteStep();
            var nodeCopies = new List<int>[NodeCount];
            for (int q = 0; q < Copies.Length; q++)
            {
                int mask = Copies[q].Mask;
                for (int k = 0; k < 8; k++)
                {
                    if (((mask >> k) & 1) == 0)
                    {
                        continue;
                    }
                    int n = CopyNodes[q * 8 + k];
                    (nodeCopies[n] ??= new List<int>(4)).Add(q);
                }
            }
            var result = new List<int>();
            var visited = new HashSet<int> { seed };
            var queue = new Queue<int>();
            queue.Enqueue(seed);
            float3 center = RestPositions[seed];
            while (queue.Count > 0)
            {
                int n = queue.Dequeue();
                result.Add(n);
                if (nodeCopies[n] == null)
                {
                    continue;
                }
                foreach (int q in nodeCopies[n])
                {
                    int mask = Copies[q].Mask;
                    for (int k = 0; k < 8; k++)
                    {
                        int m = CopyNodes[q * 8 + k];
                        if (((mask >> k) & 1) != 0 && !visited.Contains(m) &&
                            math.distance(RestPositions[m], center) <= radius)
                        {
                            visited.Add(m);
                            queue.Enqueue(m);
                        }
                    }
                }
            }
            return result;
        }

        /// <summary>Number of material pieces (connected components of real copy corners).</summary>
        public int CountPieces() => PieceSizes().Count;

        /// <summary>Real node count of every material piece, largest first.</summary>
        public List<int> PieceSizes()
        {
            CompleteStep();
            var sizes = new Dictionary<int, int>();
            for (int i = 0; i < NodeCount; i++)
            {
                int piece = PieceId[i];
                if (piece >= 0)
                {
                    sizes[piece] = sizes.TryGetValue(piece, out int c) ? c + 1 : 1;
                }
            }
            var result = new List<int>(sizes.Values);
            result.Sort((x, y) => y.CompareTo(x));
            return result;
        }

        private void RebuildTopology(bool hasOld)
        {
            var timer = Stopwatch.StartNew();
            var newRange = new NativeArray<int2>(CellCount, Allocator.Persistent);
            var newCopies = new NativeList<CellCopy>(Copies.Length + 16, Allocator.Persistent);
            var newCopyNodes = new NativeList<int>(CopyNodes.Length + 128, Allocator.Persistent);
            var nodeParent = new NativeList<int>(math.max(NodeCount, 16), Allocator.TempJob);
            var nodeRest = new NativeList<float3>(math.max(NodeCount, 16), Allocator.TempJob);
            var nodeMass = new NativeList<float>(math.max(NodeCount, 16), Allocator.TempJob);
            float h = Grid.CellSize;

            new TopologyJob
            {
                CellCoord = CellCoord,
                GridToCell = GridToCell,
                CellFill = CellFill,
                Severed = Severed,
                ComponentCounts = _tables.ComponentCounts,
                ComponentMasks = _tables.ComponentMasks,
                Directions = _tables.Directions,
                PairStart = _tables.PairStart,
                Pairs = _tables.Pairs,
                Dims = Grid.Dims,
                Origin = Grid.Origin,
                CellSize = h,
                CellMass = Parameters.Density * h * h * h,
                HasOld = hasOld,
                OldRange = CellCopyRange,
                OldCopies = Copies.AsArray(),
                OldCopyNodes = CopyNodes.AsArray(),
                NewRange = newRange,
                NewCopies = newCopies,
                NewCopyNodes = newCopyNodes,
                NodeParent = nodeParent,
                NodeRest = nodeRest,
                NodeMass = nodeMass,
            }.Run();

            int count = nodeParent.Length;
            var positions = new NativeArray<float3>(count, Allocator.Persistent);
            var previous = new NativeArray<float3>(count, Allocator.Persistent);
            var velocities = new NativeArray<float3>(count, Allocator.Persistent);
            var inverseMass = new NativeArray<float>(count, Allocator.Persistent);
            var kinematicId = new NativeArray<int>(count, Allocator.Persistent);
            var kinematicLocal = new NativeArray<float3>(count, Allocator.Persistent);
            using (var empty3 = new NativeArray<float3>(0, Allocator.TempJob))
            using (var emptyI = new NativeArray<int>(0, Allocator.TempJob))
            {
                new RemapNodesJob
                {
                    NodeParent = nodeParent.AsArray(),
                    NodeRest = nodeRest.AsArray(),
                    NodeMass = nodeMass.AsArray(),
                    OldPositions = hasOld ? Positions : empty3,
                    OldPrevious = hasOld ? Previous : empty3,
                    OldVelocities = hasOld ? Velocities : empty3,
                    OldKinematicId = hasOld ? KinematicId : emptyI,
                    OldKinematicLocal = hasOld ? KinematicLocal : empty3,
                    RestToWorld = RestToWorld,
                    Positions = positions,
                    Previous = previous,
                    Velocities = velocities,
                    InverseMass = inverseMass,
                    KinematicId = kinematicId,
                    KinematicLocal = kinematicLocal,
                }.Schedule(count, 256).Complete();
            }

            DisposeNodes();
            CellCopyRange.Dispose();
            Copies.Dispose();
            CopyNodes.Dispose();
            CellCopyRange = newRange;
            Copies = newCopies;
            CopyNodes = newCopyNodes;
            Positions = positions;
            Previous = previous;
            Velocities = velocities;
            InverseMass = inverseMass;
            KinematicId = kinematicId;
            KinematicLocal = kinematicLocal;
            RestPositions = new NativeArray<float3>(nodeRest.AsArray(), Allocator.Persistent);
            _weights = new NativeArray<float>(count, Allocator.Persistent);
            NodeReal = new NativeArray<byte>(count, Allocator.Persistent);
            PieceId = new NativeArray<int>(count, Allocator.Persistent);
            new PieceJob
            {
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                NodeReal = NodeReal,
                PieceId = PieceId,
            }.Run();
            _contactPartner = new NativeArray<int>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactDistance = new NativeArray<float>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactCount = new NativeArray<int>(count, Allocator.Persistent);
            _snapshot = new NativeArray<float3>(count, Allocator.Persistent);
            _bucketEntries = new NativeArray<int>(count, Allocator.Persistent);
            nodeParent.Dispose();
            nodeRest.Dispose();
            nodeMass.Dispose();

            new BoundsJob { Positions = Positions, Bounds = _bounds }.Run();
            TopologyVersion++;
            LastTopologyMilliseconds = (float)timer.Elapsed.TotalMilliseconds;
        }

        private void DisposeNodes()
        {
            DisposeIfCreated(ref Positions);
            DisposeIfCreated(ref Previous);
            DisposeIfCreated(ref Velocities);
            DisposeIfCreated(ref RestPositions);
            DisposeIfCreated(ref InverseMass);
            DisposeIfCreated(ref KinematicId);
            DisposeIfCreated(ref KinematicLocal);
            DisposeIfCreated(ref _weights);
            DisposeIfCreated(ref NodeReal);
            DisposeIfCreated(ref PieceId);
            DisposeIfCreated(ref _contactPartner);
            DisposeIfCreated(ref _contactDistance);
            DisposeIfCreated(ref _contactCount);
            DisposeIfCreated(ref _snapshot);
            DisposeIfCreated(ref _bucketEntries);
        }

        private static void DisposeIfCreated<T>(ref NativeArray<T> array) where T : struct
        {
            if (array.IsCreated)
            {
                array.Dispose();
            }
        }
    }
}
