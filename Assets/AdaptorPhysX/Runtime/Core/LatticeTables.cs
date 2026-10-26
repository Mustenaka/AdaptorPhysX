using System;
using Unity.Collections;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Constant topology tables of one hexahedral lattice cell.
    /// Corner c sits at offset (c &amp; 1, (c &gt;&gt; 1) &amp; 1, (c &gt;&gt; 2) &amp; 1).
    /// </summary>
    public static class Lattice
    {
        public const int CornerCount = 8;
        public const int EdgeCount = 12;
        public const int TetsPerCell = 6;
        public const int NeighborDirections = 13;

        /// <summary>Edge e connects corners EdgeCorners[2e] (lower) and EdgeCorners[2e+1]; axis = e / 4.</summary>
        public static readonly int[] EdgeCorners =
        {
            0, 1, 2, 3, 4, 5, 6, 7, // x
            0, 2, 1, 3, 4, 6, 5, 7, // y
            0, 4, 1, 5, 2, 6, 3, 7, // z
        };

        /// <summary>
        /// Six positively oriented tetrahedra per cell (Kuhn subdivision around the 0-7
        /// diagonal). Every cell uses the same split, so the response has no cell-parity
        /// alternation and face diagonals of neighbouring cells match. The table holds the
        /// pattern twice (indexed by cell parity) to keep the parity hook. SolveCellsJob
        /// hard-codes the split to keep a cell's corners in registers; change both together.
        /// </summary>
        public static readonly int[] TetCorners =
        {
            0, 1, 3, 7, 0, 1, 7, 5, 0, 2, 7, 3, 0, 2, 6, 7, 0, 4, 5, 7, 0, 4, 7, 6,
            0, 1, 3, 7, 0, 1, 7, 5, 0, 2, 7, 3, 0, 2, 6, 7, 0, 4, 5, 7, 0, 4, 7, 6,
        };

        public static int3 CornerOffset(int c) => new int3(c & 1, (c >> 1) & 1, (c >> 2) & 1);

        public static int CornerIndex(int3 o) => o.x | (o.y << 1) | (o.z << 2);

        /// <summary>Local edge index of the edge along <paramref name="axis"/> starting at corner <paramref name="start"/>.</summary>
        public static int EdgeFromStart(int axis, int start)
        {
            for (int e = axis * 4; e < axis * 4 + 4; e++)
            {
                if (EdgeCorners[2 * e] == start)
                {
                    return e;
                }
            }
            return -1;
        }

        /// <summary>
        /// Connected corner components for every 12-bit severed-edge mask.
        /// Components are listed in order of their lowest corner.
        /// </summary>
        public static void BuildComponentTable(out byte[] counts, out byte[] masks)
        {
            counts = new byte[1 << EdgeCount];
            masks = new byte[(1 << EdgeCount) * CornerCount];
            var parent = new int[CornerCount];
            for (int severed = 0; severed < 1 << EdgeCount; severed++)
            {
                for (int c = 0; c < CornerCount; c++)
                {
                    parent[c] = c;
                }
                for (int e = 0; e < EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0)
                    {
                        Union(parent, EdgeCorners[2 * e], EdgeCorners[2 * e + 1]);
                    }
                }
                int count = 0;
                for (int c = 0; c < CornerCount; c++)
                {
                    if (Find(parent, c) != c)
                    {
                        continue;
                    }
                    byte mask = 0;
                    for (int k = 0; k < CornerCount; k++)
                    {
                        if (Find(parent, k) == c)
                        {
                            mask |= (byte)(1 << k);
                        }
                    }
                    masks[severed * CornerCount + count++] = mask;
                }
                // Roots are the lowest corner of each component because Union keeps the smaller index.
                counts[severed] = (byte)count;
            }
        }

        /// <summary>
        /// The 13 "positive" neighbour directions and, for each, the corner pairs
        /// (corner in this cell, same grid point's corner in the neighbour).
        /// </summary>
        public static void BuildNeighborTable(out int3[] directions, out int[] pairStart, out int[] pairs)
        {
            directions = new int3[NeighborDirections];
            pairStart = new int[NeighborDirections + 1];
            var pairList = new System.Collections.Generic.List<int>();
            int d = 0;
            for (int z = -1; z <= 1; z++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        bool positive = z > 0 || (z == 0 && (y > 0 || (y == 0 && x > 0)));
                        if (!positive)
                        {
                            continue;
                        }
                        var dir = new int3(x, y, z);
                        directions[d] = dir;
                        pairStart[d] = pairList.Count / 2;
                        for (int k = 0; k < CornerCount; k++)
                        {
                            int3 other = CornerOffset(k) - dir;
                            if (math.all(other >= 0) && math.all(other <= 1))
                            {
                                pairList.Add(k);
                                pairList.Add(CornerIndex(other));
                            }
                        }
                        d++;
                    }
                }
            }
            pairStart[NeighborDirections] = pairList.Count / 2;
            pairs = pairList.ToArray();
        }

        /// <summary>Rest inverse shape matrices and volumes of the 10 tetrahedra of a cube with edge <paramref name="h"/>.</summary>
        public static void BuildRestTets(float h, out float3x3[] inverseRest, out float[] volumes)
        {
            inverseRest = new float3x3[2 * TetsPerCell];
            volumes = new float[2 * TetsPerCell];
            for (int t = 0; t < 2 * TetsPerCell; t++)
            {
                float3 x0 = (float3)CornerOffset(TetCorners[4 * t]) * h;
                float3 x1 = (float3)CornerOffset(TetCorners[4 * t + 1]) * h;
                float3 x2 = (float3)CornerOffset(TetCorners[4 * t + 2]) * h;
                float3 x3 = (float3)CornerOffset(TetCorners[4 * t + 3]) * h;
                var dm = new float3x3(x1 - x0, x2 - x0, x3 - x0);
                float det = math.determinant(dm);
                if (det <= 0.0f)
                {
                    throw new InvalidOperationException("Lattice tet table has a non-positive tetrahedron.");
                }
                inverseRest[t] = math.inverse(dm);
                volumes[t] = det / 6.0f;
            }
        }

        private static int Find(int[] parent, int x)
        {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        }

        private static void Union(int[] parent, int a, int b)
        {
            a = Find(parent, a);
            b = Find(parent, b);
            if (a == b)
            {
                return;
            }
            if (a < b)
            {
                parent[b] = a;
            }
            else
            {
                parent[a] = b;
            }
        }
    }

    /// <summary>Lattice tables copied into native memory for Burst jobs.</summary>
    public struct LatticeTables : IDisposable
    {
        public NativeArray<int> EdgeCorners;
        public NativeArray<int> TetCorners;
        public NativeArray<byte> ComponentCounts;
        public NativeArray<byte> ComponentMasks;
        public NativeArray<int3> Directions;
        public NativeArray<int> PairStart;
        public NativeArray<int> Pairs;

        public static LatticeTables Create()
        {
            Lattice.BuildComponentTable(out byte[] counts, out byte[] masks);
            Lattice.BuildNeighborTable(out int3[] directions, out int[] pairStart, out int[] pairs);
            return new LatticeTables
            {
                EdgeCorners = new NativeArray<int>(Lattice.EdgeCorners, Allocator.Persistent),
                TetCorners = new NativeArray<int>(Lattice.TetCorners, Allocator.Persistent),
                ComponentCounts = new NativeArray<byte>(counts, Allocator.Persistent),
                ComponentMasks = new NativeArray<byte>(masks, Allocator.Persistent),
                Directions = new NativeArray<int3>(directions, Allocator.Persistent),
                PairStart = new NativeArray<int>(pairStart, Allocator.Persistent),
                Pairs = new NativeArray<int>(pairs, Allocator.Persistent),
            };
        }

        public void Dispose()
        {
            if (EdgeCorners.IsCreated) EdgeCorners.Dispose();
            if (TetCorners.IsCreated) TetCorners.Dispose();
            if (ComponentCounts.IsCreated) ComponentCounts.Dispose();
            if (ComponentMasks.IsCreated) ComponentMasks.Dispose();
            if (Directions.IsCreated) Directions.Dispose();
            if (PairStart.IsCreated) PairStart.Dispose();
            if (Pairs.IsCreated) Pairs.Dispose();
        }
    }
}
