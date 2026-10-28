using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// A cell copy of another soft body near this one, copied when the round of steps begins:
    /// the partner's nodes are then free to move in its own jobs. Corner data lives in the
    /// per-corner arrays at 8 * index.
    /// </summary>
    internal struct PartnerCell
    {
        public float3 Center;
        /// <summary>Largest distance from the centre to a corner.</summary>
        public float Radius;
        /// <summary>Corners that carry material in the copy (tetrahedra with a virtual corner are skipped).</summary>
        public int Mask;
        /// <summary>Friction coefficient of the pair.</summary>
        public float Friction;
    }

    /// <summary>
    /// Contact between soft bodies. The bodies step in parallel, each against a copy of its
    /// partners taken at the start of the round and moved on with their velocities during the
    /// step. A node is pushed out of a partner cell along the gradient of the partner's rest SDF,
    /// which is linear on each deformed tetrahedron, until the partner's surface and its own
    /// surface (its rest SDF away from the node) touch. Each body takes its mass-weighted share
    /// of the correction; the partner does the same with the roles swapped.
    /// </summary>
    internal static class BodyContacts
    {
        public const int MaxCandidates = 12;
        public const int BucketCount = 1 << 12;

        /// <summary>
        /// Deepest penetration of point <paramref name="x"/> into the candidate partner cells at
        /// time <paramref name="tau"/> (seconds into the step), less the node's own distance
        /// outside its surface (<paramref name="nodeSdf"/>). Returns 0 without contact.
        /// </summary>
        /// <param name="normal">Outward normal of the partner surface.</param>
        /// <param name="partnerWeight">Inverse mass of the partner point (sum of b_k^2 w_k).</param>
        /// <param name="partnerVelocity">Velocity of the partner point.</param>
        public static float Penetration(float3 x, float nodeSdf, int first, int count, float tau,
            NativeArray<int> candidate, NativeArray<PartnerCell> cells, NativeArray<float3> corner,
            NativeArray<float3> velocity, NativeArray<float> sdf, NativeArray<float> weight,
            out float3 normal, out float partnerWeight, out float3 partnerVelocity, out float friction)
        {
            normal = float3.zero;
            partnerWeight = 0.0f;
            partnerVelocity = float3.zero;
            friction = 0.0f;
            float best = 0.0f;
            for (int k = 0; k < count; k++)
            {
                int cell = candidate[first + k];
                int b = cell * Lattice.CornerCount;
                float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
                for (int c = 0; c < 8; c++)
                {
                    float3 p = corner[b + c] + velocity[b + c] * tau;
                    lo = math.min(lo, p);
                    hi = math.max(hi, p);
                }
                if (math.any(x < lo) || math.any(x > hi))
                {
                    continue;
                }
                int mask = cells[cell].Mask;
                for (int t = 0; t < Lattice.TetsPerCell; t++)
                {
                    int4 tc = Tet(t);
                    if (((mask >> tc.x) & (mask >> tc.y) & (mask >> tc.z) & (mask >> tc.w) & 1) == 0)
                    {
                        continue;
                    }
                    float3 p0 = corner[b + tc.x] + velocity[b + tc.x] * tau;
                    float3x3 m = new float3x3(
                        corner[b + tc.y] + velocity[b + tc.y] * tau - p0,
                        corner[b + tc.z] + velocity[b + tc.z] * tau - p0,
                        corner[b + tc.w] + velocity[b + tc.w] * tau - p0);
                    float det = math.determinant(m);
                    if (!(math.abs(det) > 1e-18f))
                    {
                        continue;
                    }
                    float3x3 inverse = math.inverse(m);
                    float3 bc = math.mul(inverse, x - p0);
                    float b0 = 1.0f - math.csum(bc);
                    const float eps = -1e-5f;
                    if (b0 < eps || math.any(bc < eps))
                    {
                        continue;
                    }
                    // The rest SDF interpolated linearly over the deformed tetrahedron; its world
                    // gradient is inverse^T times the corner differences.
                    float s0 = sdf[b + tc.x];
                    float3 ds = new float3(sdf[b + tc.y], sdf[b + tc.z], sdf[b + tc.w]) - s0;
                    float3 gradient = math.mul(math.transpose(inverse), ds);
                    float g = math.length(gradient);
                    if (g < 0.5f)
                    {
                        // On the medial axis of the SDF (e.g. inside an edge of the partner, where the corners
                        // lie as far from two faces) the gradient is short or zero, and the distance
                        // -phi / g would be far too large.
                        break;
                    }
                    float depth = -(s0 + math.dot(bc, ds)) / g - nodeSdf;
                    if (depth > best)
                    {
                        best = depth;
                        normal = gradient / g;
                        float4 w = new float4(b0, bc);
                        partnerWeight = w.x * w.x * weight[b + tc.x] + w.y * w.y * weight[b + tc.y] +
                                        w.z * w.z * weight[b + tc.z] + w.w * w.w * weight[b + tc.w];
                        partnerVelocity = w.x * velocity[b + tc.x] + w.y * velocity[b + tc.y] +
                                          w.z * velocity[b + tc.z] + w.w * velocity[b + tc.w];
                        friction = cells[cell].Friction;
                    }
                    // A point lies in one tetrahedron of a cell.
                    break;
                }
            }
            return best;
        }

        /// <summary>Corners of tetrahedron <paramref name="t"/> of the Kuhn split (Lattice.TetCorners).</summary>
        internal static int4 Tet(int t)
        {
            switch (t)
            {
                case 0: return new int4(0, 1, 3, 7);
                case 1: return new int4(0, 1, 7, 5);
                case 2: return new int4(0, 2, 7, 3);
                case 3: return new int4(0, 2, 6, 7);
                case 4: return new int4(0, 4, 5, 7);
                default: return new int4(0, 4, 7, 6);
            }
        }
    }

    /// <summary>
    /// Signed distance used by the contact between bodies at every grid point. The voxelizer's SDF is
    /// exact within 1.5 cells of the surface and set to <see cref="Voxelizer.FarCells"/> cells deeper in,
    /// where its gradient vanishes and a node that got that deep into a partner would no longer be pushed
    /// out. Inside points beyond the exact band take the shortest path to it through the 26-neighbourhood
    /// (forward and backward sweeps, twice); the band and the outside keep the voxelizer's values.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct ContactSdfJob : IJob
    {
        [ReadOnly] public NativeArray<float> Sdf;
        public int3 PointDims;
        public float CellSize;
        public NativeArray<float> Result;

        public void Execute()
        {
            float far = Voxelizer.FarCells * CellSize;
            var distance = new NativeArray<float>(Sdf.Length, Allocator.Temp);
            var unknown = new NativeArray<bool>(Sdf.Length, Allocator.Temp);
            for (int i = 0; i < Sdf.Length; i++)
            {
                float s = Sdf[i];
                unknown[i] = s <= -0.9999f * far;
                // Distance inside the surface; -1 marks outside points, which take no part.
                distance[i] = unknown[i] ? float.MaxValue : s < 0.0f ? -s : -1.0f;
            }
            for (int round = 0; round < 2; round++)
            {
                for (int i = 0; i < Sdf.Length; i++)
                {
                    Relax(i, distance, unknown, true);
                }
                for (int i = Sdf.Length - 1; i >= 0; i--)
                {
                    Relax(i, distance, unknown, false);
                }
            }
            for (int i = 0; i < Sdf.Length; i++)
            {
                Result[i] = unknown[i] && distance[i] < float.MaxValue ? -distance[i] : Sdf[i];
            }
        }

        /// <summary>Shortest distance of unknown point <paramref name="i"/> through its neighbours before (or after) it.</summary>
        private void Relax(int i, NativeArray<float> distance, NativeArray<bool> unknown, bool forward)
        {
            if (!unknown[i])
            {
                return;
            }
            int3 p = new int3(i % PointDims.x, (i / PointDims.x) % PointDims.y, i / (PointDims.x * PointDims.y));
            float best = distance[i];
            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                int offset = dx + PointDims.x * (dy + PointDims.y * dz);
                if (forward ? offset >= 0 : offset <= 0)
                {
                    continue;
                }
                int3 q = p + new int3(dx, dy, dz);
                if (math.any(q < 0) || math.any(q >= PointDims))
                {
                    continue;
                }
                float d = distance[i + offset];
                if (d >= 0.0f && d < float.MaxValue)
                {
                    best = math.min(best, d + CellSize * math.sqrt(dx * dx + dy * dy + dz * dz));
                }
            }
            distance[i] = best;
        }
    }

    /// <summary>Rest SDF at each node's grid point.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct NodeSdfJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> RestPositions;
        [ReadOnly] public NativeArray<float> Sdf;
        public float3 Origin;
        public float InverseCellSize;
        public int3 PointDims;
        public NativeArray<float> NodeSdf;

        public void Execute(int i)
        {
            int3 p = math.clamp((int3)math.round((RestPositions[i] - Origin) * InverseCellSize), 0, PointDims - 1);
            NodeSdf[i] = Sdf[p.x + PointDims.x * (p.y + PointDims.y * p.z)];
        }
    }

    /// <summary>Copies the cell copies of a partner body that intersect a region into the receiving body's arrays.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct ExtractPartnerCellsJob : IJob
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        [ReadOnly] public NativeArray<float> NodeSdf;
        [ReadOnly] public NativeArray<float> InverseMass;
        [ReadOnly] public NativeArray<int> KinematicId;
        /// <summary>Density the partner's masses were built with over its current density.</summary>
        public float MassScale;
        public float3 RegionMin, RegionMax;
        public float Friction;

        public NativeList<PartnerCell> Cells;
        public NativeList<float3> Corner;
        public NativeList<float3> Velocity;
        public NativeList<float> Sdf;
        public NativeList<float> Weight;
        /// <summary>Partner node of each corner.</summary>
        public NativeList<int> Nodes;

        public void Execute()
        {
            for (int q = 0; q < Copies.Length; q++)
            {
                int b = q * Lattice.CornerCount;
                float3 lo = Positions[CopyNodes[b]], hi = lo;
                for (int k = 1; k < 8; k++)
                {
                    float3 x = Positions[CopyNodes[b + k]];
                    lo = math.min(lo, x);
                    hi = math.max(hi, x);
                }
                if (math.any(hi < RegionMin) || math.any(lo > RegionMax))
                {
                    continue;
                }
                float3 center = 0.5f * (lo + hi);
                float radius = 0.0f;
                for (int k = 0; k < 8; k++)
                {
                    int n = CopyNodes[b + k];
                    float3 x = Positions[n];
                    radius = math.max(radius, math.distance(x, center));
                    Corner.Add(x);
                    Velocity.Add(Velocities[n]);
                    Sdf.Add(NodeSdf[n]);
                    Weight.Add(KinematicId[n] >= 0 ? 0.0f : InverseMass[n] * MassScale);
                    Nodes.Add(n);
                }
                Cells.Add(new PartnerCell { Center = center, Radius = radius, Mask = Copies[q].Mask, Friction = Friction });
            }
        }
    }

    /// <summary>
    /// Moves the copied corners of one partner to its solved positions of the current substep, with the
    /// velocity over the substep so far (<see cref="SubstepGroup"/>).
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RefreshPartnerCellsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> Nodes;
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Previous;
        /// <summary>One over the partner's substep length.</summary>
        public float InverseDt;
        /// <summary>First corner of the partner in the copy arrays.</summary>
        public int Offset;
        [NativeDisableParallelForRestriction] public NativeArray<float3> Corner;
        [NativeDisableParallelForRestriction] public NativeArray<float3> Velocity;

        public void Execute(int c)
        {
            int n = Nodes[Offset + c];
            Corner[Offset + c] = Positions[n];
            Velocity[Offset + c] = (Positions[n] - Previous[n]) * InverseDt;
        }
    }

    /// <summary>
    /// Soft bodies in contact that step together in one round (TODO BUG-8). Each body steps against
    /// copies of its partners' cells. Taken once per step, the copies do not show how far a partner
    /// gives way or is pushed back by its elasticity during the step; each body resolves only its share
    /// of the overlap it sees, and under a sustained load the rest grows until the nodes are lost. The
    /// bodies of a group therefore schedule their substeps in turn: after the constraint solves of a
    /// substep each body refreshes its copies to the partners' solved positions, and the contacts of
    /// the substep then resolve the same overlap from both sides, each body its mass-weighted share.
    /// A member that is read before the others have begun its step goes ahead with those that have;
    /// partners that do not step then stay in place as obstacles. Members with different step or
    /// substep lengths keep the copies of the start of the step.
    /// </summary>
    internal sealed class SubstepGroup
    {
        private readonly List<SoftBodySimulation> _members = new List<SoftBodySimulation>();
        private readonly HashSet<SoftBodySimulation> _arrived = new HashSet<SoftBodySimulation>();
        private bool _scheduled;

        public void Add(SoftBodySimulation body)
        {
            _members.Add(body);
            body.Group = this;
        }

        /// <summary>A member began its step or rests this round; the last one schedules the group.</summary>
        public void Arrive(SoftBodySimulation body)
        {
            if (_arrived.Add(body) && (_scheduled || _arrived.Count == _members.Count))
            {
                ScheduleBegun();
            }
        }

        /// <summary>Schedules the substeps of the members that have begun their steps.</summary>
        public void ScheduleBegun()
        {
            _scheduled = true;
            var begun = new List<SoftBodySimulation>();
            foreach (SoftBodySimulation body in _members)
            {
                if (body.HasBegunStep)
                {
                    begun.Add(body);
                }
            }
            if (begun.Count == 0)
            {
                return;
            }
            int substeps = begun[0].BegunSubsteps;
            bool aligned = true;
            foreach (SoftBodySimulation body in begun)
            {
                aligned &= body.BegunSubsteps == substeps && body.BegunDt == begun[0].BegunDt;
            }
            foreach (SoftBodySimulation body in begun)
            {
                body.PrepareSubsteps(begun, aligned);
            }
            if (!aligned)
            {
                foreach (SoftBodySimulation body in begun)
                {
                    for (int s = 0; s <= body.BegunSubsteps; s++)
                    {
                        body.ScheduleNodePass(s, default);
                        body.ScheduleSolvePass(s);
                    }
                    body.EndStep();
                }
                return;
            }
            var solved = new NativeArray<JobHandle>(begun.Count, Allocator.Temp);
            var refreshed = new NativeArray<JobHandle>(begun.Count, Allocator.Temp);
            JobHandle copies = default;
            for (int s = 0; s <= substeps; s++)
            {
                for (int b = 0; b < begun.Count; b++)
                {
                    begun[b].ScheduleNodePass(s, copies);
                    begun[b].ScheduleSolvePass(s);
                    solved[b] = begun[b].StepHandle;
                }
                if (s == substeps)
                {
                    break;
                }
                // Each refresh reads the partners' solved positions and writes the body's own copies,
                // which its next node job reads; the partners' next node jobs wait for it.
                JobHandle allSolved = JobHandle.CombineDependencies(solved);
                for (int b = 0; b < begun.Count; b++)
                {
                    refreshed[b] = begun[b].ScheduleRefreshPartners(allSolved);
                }
                copies = JobHandle.CombineDependencies(refreshed);
            }
            solved.Dispose();
            refreshed.Dispose();
            foreach (SoftBodySimulation body in begun)
            {
                body.EndStep();
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct BuildPartnerHashJob : IJob
    {
        [ReadOnly] public NativeArray<PartnerCell> Cells;
        public float InverseCellSize;
        /// <summary>BucketCount + 1 prefix offsets.</summary>
        public NativeArray<int> BucketStart;
        public NativeArray<int> Entries;

        public void Execute()
        {
            for (int b = 0; b < BucketStart.Length; b++)
            {
                BucketStart[b] = 0;
            }
            for (int c = 0; c < Cells.Length; c++)
            {
                BucketStart[Bucket(Cells[c].Center) + 1]++;
            }
            for (int b = 1; b < BucketStart.Length; b++)
            {
                BucketStart[b] += BucketStart[b - 1];
            }
            var fill = new NativeArray<int>(BodyContacts.BucketCount, Allocator.Temp);
            for (int c = 0; c < Cells.Length; c++)
            {
                int bucket = Bucket(Cells[c].Center);
                Entries[BucketStart[bucket] + fill[bucket]++] = c;
            }
        }

        private int Bucket(float3 x) => PartnerBucket((int3)math.floor(x * InverseCellSize));

        public static int PartnerBucket(int3 c) =>
            (int)((uint)(c.x * 73856093 ^ c.y * 19349663 ^ c.z * 83492791) & (BodyContacts.BucketCount - 1));
    }

    /// <summary>
    /// For every contact node near the partners, the partner cells it may enter during the step,
    /// nearest first (at most <see cref="BodyContacts.MaxCandidates"/>).
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FindPartnerCandidatesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<byte> ContactNode;
        [ReadOnly] public NativeArray<float> NodeSdf;
        /// <summary>
        /// Nodes deeper inside their own surface take no part: their gap condition is implied by the
        /// outer nodes', and the two disagree by a fraction of a millimetre on a coarse lattice, which
        /// makes the contact and the elasticity push the nodes back and forth every substep.
        /// </summary>
        public float MinNodeSdf;
        [ReadOnly] public NativeArray<PartnerCell> Cells;
        [ReadOnly] public NativeArray<int> BucketStart;
        [ReadOnly] public NativeArray<int> Entries;
        public float InverseCellSize;
        /// <summary>Relative motion allowed during the step on top of the cell radius.</summary>
        public float Margin;

        [NativeDisableParallelForRestriction] public NativeArray<int> Candidate;
        [NativeDisableParallelForRestriction] public NativeArray<float> CandidateGap;
        public NativeArray<int> Count;

        public void Execute(int i)
        {
            int count = 0;
            if (ContactNode[i] != 0 && NodeSdf[i] >= MinNodeSdf)
            {
                float3 x = Positions[i];
                int3 c = (int3)math.floor(x * InverseCellSize);
                int b = i * BodyContacts.MaxCandidates;
                for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    int bucket = BuildPartnerHashJob.PartnerBucket(c + new int3(dx, dy, dz));
                    for (int e = BucketStart[bucket]; e < BucketStart[bucket + 1]; e++)
                    {
                        int cell = Entries[e];
                        PartnerCell p = Cells[cell];
                        float gap = math.distance(x, p.Center) - p.Radius;
                        if (gap > Margin)
                        {
                            continue;
                        }
                        // Insert, keeping the nearest; cells listed twice (hash collisions) are skipped.
                        bool listed = false;
                        for (int k = 0; k < count && !listed; k++)
                        {
                            listed = Candidate[b + k] == cell;
                        }
                        if (listed || (count == BodyContacts.MaxCandidates && gap >= CandidateGap[b + count - 1]))
                        {
                            continue;
                        }
                        int slot = count < BodyContacts.MaxCandidates ? count++ : count - 1;
                        while (slot > 0 && CandidateGap[b + slot - 1] > gap)
                        {
                            Candidate[b + slot] = Candidate[b + slot - 1];
                            CandidateGap[b + slot] = CandidateGap[b + slot - 1];
                            slot--;
                        }
                        Candidate[b + slot] = cell;
                        CandidateGap[b + slot] = gap;
                    }
                }
            }
            Count[i] = count;
        }
    }
}
