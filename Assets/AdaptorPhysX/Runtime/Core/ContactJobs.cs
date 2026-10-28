using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Node-level contact between separated pieces, and self-contact of folded tissue.
    /// Contacts are gathered once per frame from spatial hashes; piece contacts are resolved every
    /// few substeps, folds every substep (see <see cref="FoldContactJob"/>).
    /// Nodes of different pieces may not come closer than min(rest distance, contact distance), so
    /// the two sides of a fresh cut start in contact instead of being pushed apart. Within a piece a
    /// node is pushed out of the cells of the piece that lie far from it at rest (a fold), along their
    /// rest SDF like the contact between bodies (see <see cref="BodyContacts"/>): a distance between
    /// nodes does not tell on which side of the other surface a node is, and pushed apart along the
    /// line between them, nodes that had passed each other under a load were held on the wrong side.
    /// </summary>
    internal static class Contacts
    {
        public const int MaxPerNode = 16;
        public const int BucketCount = 1 << 14;
        /// <summary>Cells of its own piece a node is tested against for a fold, nearest first.</summary>
        public const int MaxFoldCells = 8;
        /// <summary>
        /// Cells whose centre lies closer to a node's rest grid point than this many cells along every
        /// axis (the 4 x 4 x 4 cells around it) are its rest neighbourhood and take no part in its fold contact.
        /// </summary>
        public const float FoldRestCells = 2.0f;

        public static int Bucket(int3 c) =>
            (int)((uint)(c.x * 73856093 ^ c.y * 19349663 ^ c.z * 83492791) & (BucketCount - 1));
    }

    /// <summary>
    /// Marks the real nodes that can touch another piece or a fold: all but those surrounded by
    /// eight full, uncut cells, which the contact search skips.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct ContactNodesJob : IJob
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<float> CellFill;
        [ReadOnly] public NativeArray<ushort> Severed;
        [ReadOnly] public NativeArray<byte> NodeReal;
        public NativeArray<byte> ContactNode;

        public void Execute()
        {
            var solid = new NativeArray<byte>(ContactNode.Length, Allocator.Temp);
            for (int q = 0; q < Copies.Length; q++)
            {
                int cell = Copies[q].Cell;
                if (CellCopyRange[cell].y != 1 || Severed[cell] != 0 || CellFill[cell] < 1.0f)
                {
                    continue;
                }
                for (int k = 0; k < 8; k++)
                {
                    solid[CopyNodes[q * 8 + k]]++;
                }
            }
            for (int i = 0; i < ContactNode.Length; i++)
            {
                ContactNode[i] = (byte)(NodeReal[i] != 0 && solid[i] < 8 ? 1 : 0);
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct BuildContactHashJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<byte> ContactNode;
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
            for (int i = 0; i < Positions.Length; i++)
            {
                if (ContactNode[i] != 0)
                {
                    BucketStart[Contacts.Bucket((int3)math.floor(Positions[i] * InverseCellSize)) + 1]++;
                }
            }
            for (int b = 1; b < BucketStart.Length; b++)
            {
                BucketStart[b] += BucketStart[b - 1];
            }
            var fill = new NativeArray<int>(Contacts.BucketCount, Allocator.Temp);
            for (int i = 0; i < Positions.Length; i++)
            {
                if (ContactNode[i] == 0)
                {
                    continue;
                }
                int bucket = Contacts.Bucket((int3)math.floor(Positions[i] * InverseCellSize));
                Entries[BucketStart[bucket] + fill[bucket]++] = i;
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FindContactsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> RestPositions;
        [ReadOnly] public NativeArray<byte> ContactNode;
        [ReadOnly] public NativeArray<int> PieceId;
        [ReadOnly] public NativeArray<int> BucketStart;
        [ReadOnly] public NativeArray<int> Entries;
        public float InverseCellSize;
        public float SearchRadius;
        public float ContactDistance;

        [NativeDisableParallelForRestriction] public NativeArray<int> Partner;
        [NativeDisableParallelForRestriction] public NativeArray<float> Distance;
        public NativeArray<int> Count;

        public void Execute(int i)
        {
            int count = 0;
            if (ContactNode[i] != 0)
            {
                float3 x = Positions[i];
                int3 c = (int3)math.floor(x * InverseCellSize);
                float searchSq = SearchRadius * SearchRadius;
                for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    int bucket = Contacts.Bucket(c + new int3(dx, dy, dz));
                    for (int e = BucketStart[bucket]; e < BucketStart[bucket + 1] && count < Contacts.MaxPerNode; e++)
                    {
                        int j = Entries[e];
                        // Nodes of the same piece meet in a fold (FoldContactJob).
                        if (j == i || PieceId[i] == PieceId[j] || math.distancesq(Positions[j], x) > searchSq)
                        {
                            continue;
                        }
                        float d0 = math.min(math.distance(RestPositions[i], RestPositions[j]), ContactDistance);
                        if (d0 <= 0.0f)
                        {
                            continue;
                        }
                        Partner[i * Contacts.MaxPerNode + count] = j;
                        Distance[i * Contacts.MaxPerNode + count] = d0;
                        count++;
                    }
                }
            }
            Count[i] = count;
        }
    }

    /// <summary>
    /// Bounding sphere (centre, radius) and mean velocity of every cell copy that a node of a fold can enter:
    /// one with a corner on the contact surface (see <see cref="ContactNodesJob"/>), i.e. the cells of the
    /// outer two layers. Their piece, or -1 for the copies that take no part.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FoldCellsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        [ReadOnly] public NativeArray<byte> ContactNode;
        [ReadOnly] public NativeArray<int> PieceId;
        public NativeArray<float4> Cells;
        public NativeArray<float3> CellVelocity;
        public NativeArray<int> CellPiece;

        public void Execute(int q)
        {
            int b = q * Lattice.CornerCount;
            int mask = Copies[q].Mask;
            int piece = -1;
            bool surface = false;
            float3 lo = Positions[CopyNodes[b]], hi = lo;
            float3 velocity = float3.zero;
            for (int k = 0; k < 8; k++)
            {
                int n = CopyNodes[b + k];
                if (((mask >> k) & 1) != 0)
                {
                    piece = PieceId[n];
                }
                surface |= ContactNode[n] != 0;
                lo = math.min(lo, Positions[n]);
                hi = math.max(hi, Positions[n]);
                velocity += Velocities[n];
            }
            CellPiece[q] = surface ? piece : -1;
            float3 center = 0.5f * (lo + hi);
            float radius = 0.0f;
            for (int k = 0; k < 8; k++)
            {
                radius = math.max(radius, math.distance(Positions[CopyNodes[b + k]], center));
            }
            Cells[q] = new float4(center, radius);
            CellVelocity[q] = velocity * 0.125f;
        }
    }

    /// <summary>
    /// Spatial hash of the fold cells by centre. The hash cell is the largest radius plus the margin plus
    /// the reach: how far a node can move relative to a cell during the step, from the spread of the cell
    /// velocities (a body that falls as a whole has none), at most <see cref="MaxReach"/>.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct BuildFoldHashJob : IJob
    {
        [ReadOnly] public NativeArray<float4> Cells;
        [ReadOnly] public NativeArray<float3> CellVelocity;
        [ReadOnly] public NativeArray<int> CellPiece;
        public float Margin;
        public float StepDt;
        public float MaxReach;
        /// <summary>One over the hash cell size and the reach, written for <see cref="FindFoldCandidatesJob"/>.</summary>
        public NativeArray<float> Scale;
        /// <summary>BucketCount + 1 prefix offsets.</summary>
        public NativeArray<int> BucketStart;
        public NativeArray<int> Entries;

        public void Execute()
        {
            float radius = 0.0f;
            float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
            for (int q = 0; q < Cells.Length; q++)
            {
                if (CellPiece[q] >= 0)
                {
                    radius = math.max(radius, Cells[q].w);
                    lo = math.min(lo, CellVelocity[q]);
                    hi = math.max(hi, CellVelocity[q]);
                }
            }
            float reach = math.all(hi >= lo) ? math.min(math.length(hi - lo) * StepDt, MaxReach) : 0.0f;
            float scale = 1.0f / (radius + Margin + reach);
            Scale[0] = scale;
            Scale[1] = reach;
            for (int b = 0; b < BucketStart.Length; b++)
            {
                BucketStart[b] = 0;
            }
            for (int q = 0; q < Cells.Length; q++)
            {
                if (CellPiece[q] >= 0)
                {
                    BucketStart[Contacts.Bucket((int3)math.floor(Cells[q].xyz * scale)) + 1]++;
                }
            }
            for (int b = 1; b < BucketStart.Length; b++)
            {
                BucketStart[b] += BucketStart[b - 1];
            }
            var fill = new NativeArray<int>(Contacts.BucketCount, Allocator.Temp);
            for (int q = 0; q < Cells.Length; q++)
            {
                if (CellPiece[q] < 0)
                {
                    continue;
                }
                int bucket = Contacts.Bucket((int3)math.floor(Cells[q].xyz * scale));
                Entries[BucketStart[bucket] + fill[bucket]++] = q;
            }
        }
    }

    /// <summary>
    /// For every outer contact node, the cells of its own piece that lie outside its rest neighbourhood
    /// (<see cref="Contacts.FoldRestCells"/>) and near its path during the step relative to the cell, nearest
    /// to the node first (at most <see cref="Contacts.MaxFoldCells"/>). A soft part falling onto the rest of
    /// the body moves a few cells in a step; the cells next to where it starts are not those it reaches.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FindFoldCandidatesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        [ReadOnly] public NativeArray<float3> RestPositions;
        [ReadOnly] public NativeArray<byte> ContactNode;
        [ReadOnly] public NativeArray<int> PieceId;
        [ReadOnly] public NativeArray<float> NodeSdf;
        /// <summary>
        /// Nodes deeper inside their own surface take no part. Unlike between bodies two layers of nodes do: the
        /// outer layer alone let a soft part that fell onto the body push through, deeper ones made it shake.
        /// </summary>
        public float MinNodeSdf;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<float4> Cells;
        [ReadOnly] public NativeArray<float3> CellVelocity;
        [ReadOnly] public NativeArray<int> CellPiece;
        [ReadOnly] public NativeArray<int> BucketStart;
        [ReadOnly] public NativeArray<int> Entries;
        /// <summary>One over the hash cell size, and the reach (see <see cref="BuildFoldHashJob"/>).</summary>
        [ReadOnly] public NativeArray<float> Scale;
        public float3 Origin;
        public float CellSize;
        public float StepDt;
        /// <summary>Distance allowed between the cell's sphere and the node's path, for the change of velocity during the step.</summary>
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
                float3 v = Velocities[i];
                float scale = Scale[0], reach = Scale[1];
                int3 c = (int3)math.floor(x * scale);
                float3 rest = math.round((RestPositions[i] - Origin) / CellSize);
                int piece = PieceId[i];
                int b = i * Contacts.MaxFoldCells;
                for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    int bucket = Contacts.Bucket(c + new int3(dx, dy, dz));
                    for (int e = BucketStart[bucket]; e < BucketStart[bucket + 1]; e++)
                    {
                        int q = Entries[e];
                        if (CellPiece[q] != piece ||
                            math.cmax(math.abs((float3)CellCoord[Copies[q].Cell] + 0.5f - rest)) < Contacts.FoldRestCells)
                        {
                            continue;
                        }
                        // Distance from the cell's sphere to the node's path during the step, relative to the cell.
                        float4 cell = Cells[q];
                        float3 path = (v - CellVelocity[q]) * StepDt;
                        float length = math.length(path);
                        if (length > reach)
                        {
                            path *= reach / length;
                        }
                        float3 offset = cell.xyz - x;
                        float along = math.lengthsq(path) > 0.0f ? math.saturate(math.dot(offset, path) / math.lengthsq(path)) : 0.0f;
                        if (math.distance(offset, along * path) - cell.w > Margin)
                        {
                            continue;
                        }
                        // Nearest to where the node is first: a node that shakes has a long path in a random
                        // direction, and the cells along it would push out the cell it is in.
                        float gap = math.length(offset) - cell.w;
                        // Insert, keeping the nearest; cells listed twice (hash collisions) are skipped.
                        bool listed = false;
                        for (int k = 0; k < count && !listed; k++)
                        {
                            listed = Candidate[b + k] == q;
                        }
                        if (listed || (count == Contacts.MaxFoldCells && gap >= CandidateGap[b + count - 1]))
                        {
                            continue;
                        }
                        int slot = count < Contacts.MaxFoldCells ? count++ : count - 1;
                        while (slot > 0 && CandidateGap[b + slot - 1] > gap)
                        {
                            Candidate[b + slot] = Candidate[b + slot - 1];
                            CandidateGap[b + slot] = CandidateGap[b + slot - 1];
                            slot--;
                        }
                        Candidate[b + slot] = q;
                        CandidateGap[b + slot] = gap;
                    }
                }
            }
            Count[i] = count;
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CopyPositionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Source;
        public NativeArray<float3> Destination;

        public void Execute(int i) => Destination[i] = Source[i];
    }
}
