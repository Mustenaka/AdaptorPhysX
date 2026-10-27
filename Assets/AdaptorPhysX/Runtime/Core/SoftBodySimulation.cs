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
        /// <summary>
        /// Skin layer: cells within this depth below the surface use SkinStiffness times the
        /// Young's modulus and SkinPreStrain. Pre-strained skin over a softer bulk makes incisions
        /// gape like real skin. Zero disables the layer. Applied when the simulation is created.
        /// </summary>
        public float SkinThickness;
        public float SkinStiffness;
        public float SkinPreStrain;
        /// <summary>
        /// Plasticity: elastic strain (Frobenius norm of U - I) above which the rest shape flows,
        /// the rate of that flow (1/s, 0 disables plasticity), the rate at which the plastic
        /// deformation recovers (1/s) and its largest size (Frobenius norm of P - I, 0 = unlimited).
        /// </summary>
        public float PlasticYield;
        public float PlasticCreep;
        public float PlasticRecovery;
        public float MaxPlasticDeformation;

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
            SkinThickness = 0.0f,
            SkinStiffness = 10.0f,
            SkinPreStrain = 0.15f,
            PlasticYield = 0.1f,
            PlasticCreep = 0.0f,
            PlasticRecovery = 0.0f,
            MaxPlasticDeformation = 0.5f,
        };
    }

    /// <summary>
    /// Cuttable lattice soft body: sparse hexahedral cells, five Neo-Hookean
    /// tetrahedra per cell copy, and virtual-node duplication for cuts.
    /// Nodes live in world space; the rest space is the voxel grid space.
    /// </summary>
    public sealed partial class SoftBodySimulation : IDisposable
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
        private StepJobs _step;
        /// <summary>A step was begun and waits for the other bodies of <see cref="Group"/> to schedule its substeps.</summary>
        private bool _begun;

        /// <summary>Jobs of the step being scheduled, from <see cref="BeginStep"/> to <see cref="EndStep"/>.</summary>
        private struct StepJobs
        {
            public SolveCellsJob Solve;
            public NodeStepJob Node;
            public CopyPositionsJob Copy;
            public bool Contacts;
            public int Substeps;
            public float Dt;
            public float H;
            public JobHandle Handle;
        }

        // Static lattice.
        public VoxelGrid Grid { get; private set; }
        public NativeArray<int3> CellCoord;
        public NativeArray<int> GridToCell;
        public NativeArray<float> CellFill;
        /// <summary>Rest signed distance to the surface at grid points, (Dims+1)^3, x fastest. Negative inside.</summary>
        public NativeArray<float> PointSdf;
        private NativeArray<int> _sliceCells;
        private NativeArray<int4> _sliceGroups;
        /// <summary>Per slice: lane groups (start, count) and single cells (start, count).</summary>
        private NativeArray<int4> _slices;
        private NativeArray<float3x3> _tetInverseRest;
        private NativeArray<float> _tetVolume;
        /// <summary>Per cell: inverse rest-shape scaling (pre-strain) applied to the tet rest matrices.</summary>
        private NativeArray<float3x3> _cellRest;
        /// <summary>Per cell: rest-volume scale times stiffness multiplier.</summary>
        private NativeArray<float> _cellScale;
        /// <summary>PreStrain, SkinThickness, SkinStiffness and SkinPreStrain that _cellRest and _cellScale were built with.</summary>
        private float4 _cellRestParameters;
        /// <summary>Density the node masses were built with (at the last topology rebuild).</summary>
        private float _massDensity;

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
        /// <summary>
        /// Side of the blade (+1, -1) each node was on when the current cut stroke passed it, 0 when
        /// not passed; with the stroke's blade travel at that moment (see <see cref="Cut"/>).
        /// </summary>
        public NativeArray<sbyte> StrokeSide;
        public NativeArray<float> StrokeTravel;
        private int _stroke;
        private float _strokeTravel;
        private float3 _strokeNormal;
        private NativeArray<float> _weights;
        private NativeArray<quaternion> _tetRotation;
        /// <summary>Real nodes not enclosed by full uncut cells; only these take part in contacts.</summary>
        private NativeArray<byte> _contactNode;
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
        /// <summary>Number of steps scheduled so far; renderers skip frames in which the body did not step.</summary>
        public int StepCount { get; private set; }
        /// <summary>Sum of the lengths of the steps scheduled so far (s).</summary>
        public double SimulatedTime { get; private set; }

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

        /// <summary>Largest speed of a node with material after the last step (m/s).</summary>
        public float MaxSpeed
        {
            get
            {
                CompleteStep();
                return _bounds[2].x;
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
            PointSdf = new NativeArray<float>(grid.Sdf, Allocator.Persistent);
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
            // number of dependent jobs per substep low. Most cells of a slice are solved four at a
            // time in SIMD lanes (see BuildLaneGroups).
            int axis = grid.LongestAxis;
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
            var sliceGroups = new List<int4>(cellCount / 4);
            var sliceRanges = new List<int4>(slices.Count);
            int lastColor = -1;
            foreach (KeyValuePair<long, List<int>> slice in slices)
            {
                int color = (int)(slice.Key >> 32);
                while (lastColor < color)
                {
                    _colorSliceStart[++lastColor] = sliceRanges.Count;
                }
                int groupStart = sliceGroups.Count, cellStart = sliceCells.Count;
                BuildLaneGroups(grid, slice.Value, sliceGroups, sliceCells);
                sliceRanges.Add(new int4(groupStart, sliceGroups.Count - groupStart, cellStart, sliceCells.Count - cellStart));
            }
            while (lastColor < 2)
            {
                _colorSliceStart[++lastColor] = sliceRanges.Count;
            }
            _sliceCells = new NativeArray<int>(sliceCells.ToArray(), Allocator.Persistent);
            _sliceGroups = new NativeArray<int4>(sliceGroups.ToArray(), Allocator.Persistent);
            _slices = new NativeArray<int4>(sliceRanges.ToArray(), Allocator.Persistent);

            Lattice.BuildRestTets(grid.CellSize, out float3x3[] inverseRest, out float[] volumes);
            _cellRest = new NativeArray<float3x3>(cellCount, Allocator.Persistent);
            _cellScale = new NativeArray<float>(cellCount, Allocator.Persistent);
            UpdateCellRest();
            _tetInverseRest = new NativeArray<float3x3>(inverseRest, Allocator.Persistent);
            _tetVolume = new NativeArray<float>(volumes, Allocator.Persistent);

            Severed = new NativeArray<ushort>(cellCount, Allocator.Persistent);
            EdgeT = new NativeArray<float>(cellCount * Lattice.EdgeCount, Allocator.Persistent);
            EdgeNormal = new NativeArray<float3>(cellCount * Lattice.EdgeCount, Allocator.Persistent);
            CellCopyRange = new NativeArray<int2>(cellCount, Allocator.Persistent);
            Copies = new NativeList<CellCopy>(cellCount, Allocator.Persistent);
            CopyNodes = new NativeList<int>(cellCount * 8, Allocator.Persistent);

            _bounds = new NativeArray<float3>(3, Allocator.Persistent);
            _bucketStart = new NativeArray<int>(Contacts.BucketCount + 1, Allocator.Persistent);
            FramesStart = new NativeArray<float4x4>(MaxKinematicFrames, Allocator.Persistent);
            FramesEnd = new NativeArray<float4x4>(MaxKinematicFrames, Allocator.Persistent);
            for (int i = 0; i < MaxKinematicFrames; i++)
            {
                FramesStart[i] = float4x4.identity;
                FramesEnd[i] = float4x4.identity;
            }

            CreateInteraction();
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
            DisposeIfCreated(ref PointSdf);
            DisposeIfCreated(ref _sliceCells);
            DisposeIfCreated(ref _sliceGroups);
            DisposeIfCreated(ref _slices);
            DisposeIfCreated(ref _tetInverseRest);
            DisposeIfCreated(ref _tetVolume);
            DisposeIfCreated(ref _cellRest);
            DisposeIfCreated(ref _tetRotation);
            DisposeIfCreated(ref _cellScale);
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
            DisposeInteraction();
        }

        /// <summary>
        /// Splits the cells of a slice into groups of four solved together in SIMD lanes, cell i of
        /// each quarter of the slice, and single cells. The four cells of a group must share no
        /// grid point; groups that would are solved as single cells.
        /// </summary>
        /// <summary>
        /// Per-cell rest-shape scaling and stiffness from the pre-strain and skin parameters.
        /// <see cref="ScheduleStep"/> calls it again when one of them changes, so they take effect
        /// on the next step.
        /// </summary>
        private void UpdateCellRest()
        {
            SoftBodyParameters p = Parameters;
            _cellRestParameters = new float4(p.PreStrain, p.SkinThickness, p.SkinStiffness, p.SkinPreStrain);
            float bulk = 1.0f - math.clamp(p.PreStrain, 0.0f, 0.49f);
            for (int c = 0; c < _cellRest.Length; c++)
            {
                // Rest shape X -> S X. The bulk contracts isotropically; the skin contracts only in
                // the surface plane (S = b n n^T + s (I - n n^T)), as skin tension acts along the
                // surface. Linear ramp over one cell at the skin boundary.
                float skin = p.SkinThickness > 0.0f
                    ? math.saturate((p.SkinThickness - Grid.Depth[c]) / Grid.CellSize + 0.5f)
                    : 0.0f;
                float3 n = Grid.Normal[c];
                if (math.lengthsq(n) < 0.5f)
                {
                    skin = 0.0f;
                }
                float tangential = 1.0f - math.clamp(math.lerp(p.PreStrain, p.SkinPreStrain, skin), 0.0f, 0.49f);
                float3x3 nn = new float3x3(n * n.x, n * n.y, n * n.z);
                float3x3 inverse = nn * (1.0f / bulk) + (float3x3.identity - nn) * (1.0f / tangential);
                float stiffness = math.lerp(1.0f, math.max(p.SkinStiffness, 1e-3f), skin);
                _cellRest[c] = WithPlasticity(c, inverse);
                _cellScale[c] = bulk * tangential * tangential * stiffness;
            }
        }

        private static void BuildLaneGroups(VoxelGrid grid, List<int> cells, List<int4> groups, List<int> singles)
        {
            int quarter = cells.Count / 4;
            for (int i = 0; i < quarter; i++)
            {
                var g = new int4(cells[i], cells[i + quarter], cells[i + 2 * quarter], cells[i + 3 * quarter]);
                if (Apart(grid, g.x, g.y) && Apart(grid, g.x, g.z) && Apart(grid, g.x, g.w) &&
                    Apart(grid, g.y, g.z) && Apart(grid, g.y, g.w) && Apart(grid, g.z, g.w))
                {
                    groups.Add(g);
                    continue;
                }
                singles.Add(g.x);
                singles.Add(g.y);
                singles.Add(g.z);
                singles.Add(g.w);
            }
            for (int i = 4 * quarter; i < cells.Count; i++)
            {
                singles.Add(cells[i]);
            }
        }

        private static bool Apart(VoxelGrid grid, int a, int b) => math.cmax(math.abs(grid.Cells[a] - grid.Cells[b])) >= 2;

        /// <summary>Advances the body by <paramref name="dt"/> and waits for the result.</summary>
        public void Step(float dt, int substeps, NativeArray<ColliderShape> shapes, NativeArray<float> fields = default,
            NativeArray<RigidBodyState> rigidBodies = default)
        {
            ScheduleStep(dt, substeps, shapes, fields, rigidBodies);
            CompleteStep();
        }

        /// <summary>
        /// Schedules one step of <paramref name="dt"/> with small XPBD substeps. The shapes are
        /// copied, so the caller may reuse its array. <paramref name="fields"/> holds the values of the
        /// distance-field shapes (see <see cref="ColliderShape.FieldOffset"/>); it is read by the jobs and
        /// must stay valid until the step completes. <paramref name="rigidBodies"/> (copied) are the
        /// dynamic bodies of shapes with <see cref="ColliderShape.DynamicBody"/>; their response is read
        /// with <see cref="TryGetRigidBodyResponse"/>. Every other method completes the step first.
        /// </summary>
        public void ScheduleStep(float dt, int substeps, NativeArray<ColliderShape> shapes, NativeArray<float> fields = default,
            NativeArray<RigidBodyState> rigidBodies = default)
        {
            BeginStep(dt, substeps, shapes, fields, rigidBodies);
            if (Group != null)
            {
                // The substeps are scheduled together with the other bodies in contact.
                _begun = true;
                Group.Arrive(this);
                return;
            }
            for (int s = 0; s <= _step.Substeps; s++)
            {
                ScheduleNodePass(s, default);
                ScheduleSolvePass(s);
            }
            EndStep();
        }

        /// <summary>Everything before the substeps: copies the inputs and schedules the per-step jobs.</summary>
        private void BeginStep(float dt, int substeps, NativeArray<ColliderShape> shapes, NativeArray<float> fields,
            NativeArray<RigidBodyState> rigidBodies)
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
            // Co-rotational energy mu |F - R|^2 + lambda / 2 (det F - 1)^2 linearises to linear
            // elasticity with the Lame parameters.
            float lambda = math.max(p.YoungsModulus * nu / ((1.0f + nu) * (1.0f - 2.0f * nu)), 1e-3f * mu);

            var solve = new SolveCellsJob
            {
                Positions = Positions,
                Weights = _weights,
                SliceCells = _sliceCells,
                SliceGroups = _sliceGroups,
                Slices = _slices,
                CellCoord = CellCoord,
                CellCopyRange = CellCopyRange,
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                TetInverseRest = _tetInverseRest,
                TetVolume = _tetVolume,
                CellRest = _cellRest,
                CellScale = _cellScale,
                TetRotation = _tetRotation,
                HydrostaticCompliance = 1.0f / (lambda * h * h),
                DeviatoricCompliance = 1.0f / (2.0f * mu * h * h),
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

            // Pre-strain and skin parameters set the rest shape, density the masses: changes to
            // them apply from this step on (the masses are rebuilt with the new density at the
            // next topology change).
            if (!_cellRestParameters.Equals(new float4(p.PreStrain, p.SkinThickness, p.SkinStiffness, p.SkinPreStrain)))
            {
                UpdateCellRest();
            }
            ConfigureFrames(ref solve, h);
            JobHandle handle = new EffectiveMassJob
            {
                InverseMass = InverseMass,
                KinematicId = KinematicId,
                Scale = _massDensity / math.max(p.Density, 1e-6f),
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
                    ContactNode = _contactNode,
                    InverseCellSize = 1.0f / searchRadius,
                    BucketStart = _bucketStart,
                    Entries = _bucketEntries,
                }.Schedule(handle);
                handle = new FindContactsJob
                {
                    Positions = Positions,
                    RestPositions = RestPositions,
                    ContactNode = _contactNode,
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

            handle = ConfigureInteraction(ref node, handle, dt, substeps, fields, rigidBodies);
            _step = new StepJobs
            {
                Solve = solve,
                Node = node,
                Copy = copy,
                Contacts = contacts,
                Substeps = substeps,
                Dt = dt,
                H = h,
                Handle = handle,
            };
        }

        /// <summary>
        /// Schedules the node job of pass <paramref name="s"/> (0 to substeps) of the begun step after
        /// <paramref name="dependency"/>: finishes substep s-1 (contacts, colliders, velocity) and
        /// predicts substep s.
        /// </summary>
        internal void ScheduleNodePass(int s, JobHandle dependency)
        {
            int substeps = _step.Substeps;
            JobHandle handle = JobHandle.CombineDependencies(_step.Handle, dependency);
            NodeStepJob node = _step.Node;
            node.Finish = s > 0;
            node.Predict = s < substeps;
            node.FinishTimePrev = (s - 1) / (float)substeps;
            node.FinishTime = s / (float)substeps;
            node.PredictTime = (s + 1) / (float)substeps;
            // Piece contacts every few substeps: each resolution costs two dependent jobs.
            node.Contacts = _step.Contacts && s > 0 && (s % ContactInterval == 0 || s == substeps);
            if (node.Contacts)
            {
                handle = _step.Copy.Schedule(NodeCount, 512, handle);
            }
            handle = node.Schedule(NodeCount, NodeBatch, handle);
            _step.Handle = ScheduleRigidCoupling(handle, s, substeps, _step.H, _step.Dt);
        }

        /// <summary>Schedules the constraint solve of substep <paramref name="s"/> after its node job (none after the last).</summary>
        internal void ScheduleSolvePass(int s)
        {
            if (s >= _step.Substeps)
            {
                return;
            }
            JobHandle handle = _step.Handle;
            SolveCellsJob solve = _step.Solve;
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
            _step.Handle = handle;
        }

        /// <summary>Jobs of the begun step scheduled so far.</summary>
        internal JobHandle StepHandle => _step.Handle;

        /// <summary>Jobs after the last substep; the step then completes like any other.</summary>
        internal void EndStep()
        {
            JobHandle handle = ScheduleAfterSubsteps(_step.Handle, _step.Dt);
            handle = new BoundsJob { Positions = Positions, Velocities = Velocities, NodeReal = NodeReal, Bounds = _bounds }.Schedule(handle);
            StepCount++;
            SimulatedTime += _step.Dt;
            _begun = false;
            _pending = handle;
            _hasPending = true;
            JobHandle.ScheduleBatchedJobs();
        }

        /// <summary>Waits for a scheduled step. Cheap when nothing is pending.</summary>
        public void CompleteStep()
        {
            if (_begun)
            {
                // Still waiting for bodies of its group: go ahead with those that have begun.
                Group.ScheduleBegun();
            }
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
        /// Consecutive sweeps of one blade stroke pass the same non-zero <paramref name="stroke"/>:
        /// nodes then keep the side of the blade they were on when the blade passed them, so
        /// tissue that moves across the blade plane during the stroke is still separated along
        /// one surface. Stroke 0 treats every call as a stroke of its own.
        /// </summary>
        public int Cut(float3 a0, float3 b0, float3 a1, float3 b1, int stroke = 0)
        {
            CompleteStep();
            if (stroke == 0 || stroke != _stroke)
            {
                for (int i = 0; i < StrokeSide.Length; i++)
                {
                    StrokeSide[i] = 0;
                }
                _strokeTravel = 0.0f;
                _strokeNormal = float3.zero;
            }
            _stroke = stroke;
            float3 sweptA0 = a0, sweptB0 = b0;
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
            _strokeTravel += travel;
            // The blade normal flips when the stroke reverses; the labels keep the stroke's orientation.
            float3 normal = math.normalizesafe(math.cross(b0 - a0, b1 - a0) + math.cross(b1 - a0, a1 - a0));
            float orientation = math.dot(normal, _strokeNormal) < 0.0f ? -1.0f : 1.0f;
            if (math.lengthsq(normal) > 0.5f)
            {
                _strokeNormal = normal * orientation;
            }
            using var count = new NativeArray<int>(1, Allocator.TempJob);
            new CutJob
            {
                Positions = Positions,
                CellCoord = CellCoord,
                GridToCell = GridToCell,
                PointSdf = PointSdf,
                CellCopyRange = CellCopyRange,
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                EdgeCorners = _tables.EdgeCorners,
                Dims = Grid.Dims,
                A0 = a0, B0 = b0, A1 = a1, B1 = b1,
                SweptA0 = sweptA0, SweptB0 = sweptB0,
                StrokeSide = StrokeSide,
                StrokeTravel = StrokeTravel,
                Travel = _strokeTravel,
                // Tissue dragged along by the blade is passed over several cells of blade travel.
                LabelTravel = 8.0f * CellSize,
                // Wide enough that a node next to a labelled one is labelled too while the tissue moves.
                Band = 3.0f * CellSize,
                Orientation = orientation,
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
            KinematicFramesChanged(frame);
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
            KinematicFramesChanged(frame);
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
            var copyParent = new NativeList<int>(Copies.Length + 16, Allocator.TempJob);
            float h = Grid.CellSize;
            _massDensity = Parameters.Density;

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
                CellMass = _massDensity * h * h * h,
                HasOld = hasOld,
                OldRange = CellCopyRange,
                OldCopies = Copies.AsArray(),
                OldCopyNodes = CopyNodes.AsArray(),
                NewRange = newRange,
                NewCopies = newCopies,
                NewCopyNodes = newCopyNodes,
                NewCopyParent = copyParent,
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
            var strokeSide = new NativeArray<sbyte>(count, Allocator.Persistent);
            var strokeTravel = new NativeArray<float>(count, Allocator.Persistent);
            using (var empty3 = new NativeArray<float3>(0, Allocator.TempJob))
            using (var emptyI = new NativeArray<int>(0, Allocator.TempJob))
            using (var emptyS = new NativeArray<sbyte>(0, Allocator.TempJob))
            using (var emptyF = new NativeArray<float>(0, Allocator.TempJob))
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
                    OldStrokeSide = hasOld ? StrokeSide : emptyS,
                    OldStrokeTravel = hasOld ? StrokeTravel : emptyF,
                    RestToWorld = RestToWorld,
                    Positions = positions,
                    Previous = previous,
                    Velocities = velocities,
                    InverseMass = inverseMass,
                    KinematicId = kinematicId,
                    KinematicLocal = kinematicLocal,
                    StrokeSide = strokeSide,
                    StrokeTravel = strokeTravel,
                }.Schedule(count, 256).Complete();
            }

            // Tet rotations follow their copies so the polar decomposition stays warm across cuts.
            // New tetrahedra start at the rest-to-world rotation: the warm-started update turns at
            // most half a radian per substep and cannot leave identity for a half turn at all.
            var initialRotation = new quaternion(new float3x3(RestToWorld.c0.xyz, RestToWorld.c1.xyz, RestToWorld.c2.xyz));
            var rotations = new NativeArray<quaternion>(newCopies.Length * Lattice.TetsPerCell, Allocator.Persistent);
            for (int q = 0; q < copyParent.Length; q++)
            {
                int parent = copyParent[q];
                for (int t = 0; t < Lattice.TetsPerCell; t++)
                {
                    rotations[q * Lattice.TetsPerCell + t] = parent >= 0 && _tetRotation.IsCreated
                        ? _tetRotation[parent * Lattice.TetsPerCell + t]
                        : initialRotation;
                }
            }
            copyParent.Dispose();
            DisposeIfCreated(ref _tetRotation);
            _tetRotation = rotations;

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
            StrokeSide = strokeSide;
            StrokeTravel = strokeTravel;
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
            _contactNode = new NativeArray<byte>(count, Allocator.Persistent);
            new ContactNodesJob
            {
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                CellCopyRange = CellCopyRange,
                CellFill = CellFill,
                Severed = Severed,
                NodeReal = NodeReal,
                ContactNode = _contactNode,
            }.Run();
            _contactPartner = new NativeArray<int>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactDistance = new NativeArray<float>(count * Contacts.MaxPerNode, Allocator.Persistent);
            _contactCount = new NativeArray<int>(count, Allocator.Persistent);
            _snapshot = new NativeArray<float3>(count, Allocator.Persistent);
            _bucketEntries = new NativeArray<int>(count, Allocator.Persistent);
            nodeParent.Dispose();
            nodeRest.Dispose();
            nodeMass.Dispose();
            RebuildInteractionNodes();
            // Copies of kinematic nodes stay kinematic; the frames' masses are shared among them.
            KinematicFramesChanged(-1);

            new BoundsJob { Positions = Positions, Velocities = Velocities, NodeReal = NodeReal, Bounds = _bounds }.Run();
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
            DisposeIfCreated(ref StrokeSide);
            DisposeIfCreated(ref StrokeTravel);
            DisposeIfCreated(ref _weights);
            DisposeIfCreated(ref NodeReal);
            DisposeIfCreated(ref PieceId);
            DisposeIfCreated(ref _contactNode);
            DisposeIfCreated(ref _contactPartner);
            DisposeIfCreated(ref _contactDistance);
            DisposeIfCreated(ref _contactCount);
            DisposeIfCreated(ref _snapshot);
            DisposeIfCreated(ref _bucketEntries);
            DisposeInteractionNodes();
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
