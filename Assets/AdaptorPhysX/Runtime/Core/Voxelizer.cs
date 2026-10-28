using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>Result of voxelizing a closed triangle mesh into lattice cells.</summary>
    public sealed class VoxelGrid
    {
        public float3 Origin;
        public float CellSize;
        public int3 Dims;
        /// <summary>Signed distance at grid points, (Dims+1)^3, x fastest. Negative inside.</summary>
        public float[] Sdf;
        /// <summary>Occupied cells, listed with the longest axis outermost (see <see cref="LongestAxis"/>).</summary>
        public int3[] Cells;
        /// <summary>Approximate material fraction of each cell in [1/8, 1].</summary>
        public float[] Fill;
        /// <summary>Depth of each cell centre below the surface (mean of corner SDFs, negated).</summary>
        public float[] Depth;
        /// <summary>Outward surface normal at each cell centre (normalised SDF gradient, zero deep inside).</summary>
        public float3[] Normal;

        public int3 PointDims => Dims + 1;

        /// <summary>
        /// Axis the solver slices the body across. Cells (and hence copies and nodes) are ordered
        /// slice by slice, so parallel slices write to separate memory instead of sharing cache lines.
        /// </summary>
        public int LongestAxis => Dims.x >= Dims.y && Dims.x >= Dims.z ? 0 : Dims.y >= Dims.z ? 1 : 2;

        public int PointIndex(int3 p) => p.x + PointDims.x * (p.y + PointDims.y * p.z);

        public float3 PointPosition(int3 p) => Origin + (float3)p * CellSize;

        /// <summary>Trilinear SDF at a rest-space point (clamped to the grid).</summary>
        public float SampleSdf(float3 p)
        {
            float3 g = math.clamp((p - Origin) / CellSize, 0.0f, (float3)Dims - 1e-4f);
            int3 c = (int3)math.floor(g);
            float3 u = g - c;
            float value = 0.0f;
            for (int k = 0; k < 8; k++)
            {
                value += Geometry.TrilinearWeight(k, u) * Sdf[PointIndex(c + Lattice.CornerOffset(k))];
            }
            return value;
        }
    }

    public static class Voxelizer
    {
        /// <summary>SDF value, in cells, of the points further than 1.5 cells from the surface.</summary>
        public const float FarCells = 2.0f;

        /// <summary>Triangles per leaf of the winding number tree.</summary>
        private const int WindingLeafTriangles = 8;
        /// <summary>A node is replaced by its dipole when the point is further than this many node radii.</summary>
        private const float WindingBeta = 2.0f;

        /// <summary>
        /// Voxelizes a mesh given in rest space. A cell is kept when one of its corners is inside the
        /// mesh or when it contains a mesh vertex, so every vertex can be embedded. Inside and outside
        /// are decided by ray parity for a closed mesh, and by the generalized winding number for an
        /// open one (<paramref name="closed"/> false), which closes holes in the lattice.
        /// </summary>
        public static VoxelGrid Voxelize(float3[] vertices, int[] triangles, float cellSize, bool closed = true)
        {
            float3 min = vertices[0];
            float3 max = vertices[0];
            foreach (float3 v in vertices)
            {
                min = math.min(min, v);
                max = math.max(max, v);
            }

            var grid = new VoxelGrid { CellSize = cellSize };
            grid.Origin = min - 0.5f * cellSize;
            grid.Dims = (int3)math.ceil((max - grid.Origin) / cellSize) + 1;
            int3 pd = grid.PointDims;
            int pointCount = pd.x * pd.y * pd.z;

            using var verts = new NativeArray<float3>(vertices, Allocator.TempJob);
            using var tris = new NativeArray<int>(triangles, Allocator.TempJob);
            using var inside = new NativeArray<byte>(pointCount, Allocator.TempJob);
            using var distance = new NativeArray<float>(pointCount, Allocator.TempJob);
            using var order = new NativeArray<int>(closed ? 0 : triangles.Length / 3, Allocator.TempJob);
            using var nodes = new NativeList<WindingNode>(Allocator.TempJob);
            using var rowStart = new NativeArray<int>(closed ? pd.y * pd.z + 1 : 1, Allocator.TempJob);
            using var rowTriangles = new NativeList<int>(Allocator.TempJob);
            using var slabStart = new NativeArray<int>(pd.z + 1, Allocator.TempJob);
            using var slabTriangles = new NativeList<int>(Allocator.TempJob);

            JobHandle parity;
            if (closed)
            {
                JobHandle rows = new BinRowTrianglesJob
                {
                    Vertices = verts,
                    Triangles = tris,
                    Origin = grid.Origin,
                    CellSize = cellSize,
                    PointDims = pd,
                    RowStart = rowStart,
                    RowTriangles = rowTriangles,
                }.Schedule();
                parity = new InsideParityJob
                {
                    Vertices = verts,
                    Triangles = tris,
                    RowStart = rowStart,
                    RowTriangles = rowTriangles.AsDeferredJobArray(),
                    Origin = grid.Origin,
                    CellSize = cellSize,
                    PointDims = pd,
                    Inside = inside,
                }.Schedule(pd.y * pd.z, 4, rows);
            }
            else
            {
                JobHandle tree = new BuildWindingTreeJob { Vertices = verts, Triangles = tris, Order = order, Nodes = nodes }.Schedule();
                parity = new InsideWindingJob
                {
                    Vertices = verts,
                    Triangles = tris,
                    Order = order,
                    Nodes = nodes.AsDeferredJobArray(),
                    Origin = grid.Origin,
                    CellSize = cellSize,
                    PointDims = pd,
                    Inside = inside,
                }.Schedule(pointCount, 64, tree);
            }
            JobHandle slabs = new BinSlabTrianglesJob
            {
                Vertices = verts,
                Triangles = tris,
                Origin = grid.Origin,
                CellSize = cellSize,
                PointDims = pd,
                SlabStart = slabStart,
                SlabTriangles = slabTriangles,
            }.Schedule();
            var near = new NearDistanceJob
            {
                Vertices = verts,
                Triangles = tris,
                SlabStart = slabStart,
                SlabTriangles = slabTriangles.AsDeferredJobArray(),
                Origin = grid.Origin,
                CellSize = cellSize,
                PointDims = pd,
                Distance = distance,
            }.Schedule(pd.z, 1, slabs);
            JobHandle.CombineDependencies(parity, near).Complete();

            grid.Sdf = new float[pointCount];
            for (int i = 0; i < pointCount; i++)
            {
                grid.Sdf[i] = inside[i] != 0 ? -distance[i] : distance[i];
            }

            var vertexCells = new HashSet<int>();
            foreach (float3 v in vertices)
            {
                int3 c = math.clamp((int3)math.floor((v - grid.Origin) / cellSize), 0, grid.Dims - 1);
                vertexCells.Add(c.x + grid.Dims.x * (c.y + grid.Dims.y * c.z));
            }

            var cells = new List<int3>();
            var fill = new List<float>();
            var depth = new List<float>();
            var normal = new List<float3>();
            var corner = new float[8];
            int axis = grid.LongestAxis, axis1 = (axis + 1) % 3, axis2 = (axis + 2) % 3;
            for (int slice = 0; slice < grid.Dims[axis]; slice++)
            {
                for (int row = 0; row < grid.Dims[axis2]; row++)
                {
                    for (int column = 0; column < grid.Dims[axis1]; column++)
                    {
                        int3 c = default;
                        c[axis] = slice;
                        c[axis1] = column;
                        c[axis2] = row;
                        int x = c.x, y = c.y, z = c.z;
                        bool anyInside = false;
                        for (int k = 0; k < 8; k++)
                        {
                            corner[k] = grid.Sdf[grid.PointIndex(c + Lattice.CornerOffset(k))];
                            anyInside |= corner[k] < 0.0f;
                        }
                        bool hasVertex = vertexCells.Contains(x + grid.Dims.x * (y + grid.Dims.y * z));
                        if (!anyInside && !hasVertex)
                        {
                            continue;
                        }
                        cells.Add(c);
                        fill.Add(EstimateFill(corner));
                        float mean = 0.0f;
                        foreach (float value in corner)
                        {
                            mean += value;
                        }
                        depth.Add(-mean / 8.0f);
                        float3 gradient = float3.zero;
                        for (int k = 0; k < 8; k++)
                        {
                            gradient += Geometry.TrilinearGradient(k, new float3(0.5f)) * corner[k];
                        }
                        normal.Add(math.normalizesafe(gradient));
                    }
                }
            }
            grid.Cells = cells.ToArray();
            grid.Fill = fill.ToArray();
            grid.Depth = depth.ToArray();
            grid.Normal = normal.ToArray();
            return grid;
        }

        /// <summary>Samples the trilinear SDF at the 8 octant centres of the cell.</summary>
        private static float EstimateFill(float[] corner)
        {
            int insideSamples = 0;
            for (int s = 0; s < 8; s++)
            {
                float3 u = 0.25f + 0.5f * (float3)Lattice.CornerOffset(s);
                float value = 0.0f;
                for (int k = 0; k < 8; k++)
                {
                    float3 o = Lattice.CornerOffset(k);
                    float3 w = math.select(1.0f - u, u, o > 0.5f);
                    value += w.x * w.y * w.z * corner[k];
                }
                insideSamples += value < 0.0f ? 1 : 0;
            }
            return math.max(insideSamples, 1) / 8.0f;
        }

        /// <summary>Offsets of the parity rays from the grid rows, in cells: they keep rays off shared triangle edges.</summary>
        private const float RayOffsetY = 1.37e-4f, RayOffsetZ = 2.91e-4f;

        /// <summary>
        /// Lists, for every grid point row (y, z), the triangles whose yz bounds contain its parity ray, with a
        /// margin of a thousandth of a cell: a superset of the triangles the ray can hit.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct BinRowTrianglesJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            /// <summary>Row r's triangles are RowTriangles[RowStart[r] .. RowStart[r + 1]).</summary>
            public NativeArray<int> RowStart;
            public NativeList<int> RowTriangles;

            public void Execute()
            {
                for (int pass = 0; pass < 2; pass++)
                {
                    var cursor = new NativeArray<int>(pass == 0 ? 0 : RowStart.Length - 1, Allocator.Temp);
                    for (int t = 0; t < Triangles.Length; t += 3)
                    {
                        float3 a = Vertices[Triangles[t]], b = Vertices[Triangles[t + 1]], c = Vertices[Triangles[t + 2]];
                        float2 lo = (math.min(a, math.min(b, c)).yz - Origin.yz) / CellSize;
                        float2 hi = (math.max(a, math.max(b, c)).yz - Origin.yz) / CellSize;
                        var offset = new float2(RayOffsetY, RayOffsetZ);
                        int2 r0 = math.max((int2)math.ceil(lo - offset - 1e-3f), 0);
                        int2 r1 = math.min((int2)math.floor(hi - offset + 1e-3f), PointDims.yz - 1);
                        for (int k = r0.y; k <= r1.y; k++)
                        {
                            for (int j = r0.x; j <= r1.x; j++)
                            {
                                int row = j + PointDims.y * k;
                                if (pass == 0)
                                {
                                    RowStart[row + 1]++;
                                }
                                else
                                {
                                    RowTriangles[RowStart[row] + cursor[row]++] = t / 3;
                                }
                            }
                        }
                    }
                    if (pass == 0)
                    {
                        for (int r = 1; r < RowStart.Length; r++)
                        {
                            RowStart[r] += RowStart[r - 1];
                        }
                        RowTriangles.ResizeUninitialized(RowStart[RowStart.Length - 1]);
                    }
                    cursor.Dispose();
                }
            }
        }

        /// <summary>Even-odd ray parity along +x for every grid point row.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct InsideParityJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            [ReadOnly] public NativeArray<int> RowStart;
            [ReadOnly] public NativeArray<int> RowTriangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            [NativeDisableParallelForRestriction] public NativeArray<byte> Inside;

            public void Execute(int row)
            {
                int j = row % PointDims.y;
                int k = row / PointDims.y;
                float ry = Origin.y + j * CellSize + CellSize * RayOffsetY;
                float rz = Origin.z + k * CellSize + CellSize * RayOffsetZ;
                var hits = new NativeList<float>(16, Allocator.Temp);
                for (int e = RowStart[row]; e < RowStart[row + 1]; e++)
                {
                    int t = 3 * RowTriangles[e];
                    float3 a = Vertices[Triangles[t]];
                    float3 b = Vertices[Triangles[t + 1]];
                    float3 c = Vertices[Triangles[t + 2]];
                    float2 p = new float2(ry, rz);
                    float2 a2 = a.yz, b2 = b.yz, c2 = c.yz;
                    float d = (b2.x - a2.x) * (c2.y - a2.y) - (c2.x - a2.x) * (b2.y - a2.y);
                    if (math.abs(d) < 1e-20f)
                    {
                        continue;
                    }
                    float w1 = ((p.x - a2.x) * (c2.y - a2.y) - (c2.x - a2.x) * (p.y - a2.y)) / d;
                    float w2 = ((b2.x - a2.x) * (p.y - a2.y) - (p.x - a2.x) * (b2.y - a2.y)) / d;
                    float w0 = 1.0f - w1 - w2;
                    if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f)
                    {
                        continue;
                    }
                    hits.Add(w0 * a.x + w1 * b.x + w2 * c.x);
                }
                hits.Sort();
                int h = 0;
                for (int i = 0; i < PointDims.x; i++)
                {
                    float x = Origin.x + i * CellSize;
                    while (h < hits.Length && hits[h] < x)
                    {
                        h++;
                    }
                    Inside[i + PointDims.x * (j + PointDims.y * k)] = (byte)(h & 1);
                }
                hits.Dispose();
            }
        }

        /// <summary>
        /// Node of the bounding tree for fast winding numbers (Barill et al. 2018, first order): far from a
        /// node, its triangles subtend the solid angle of a dipole, the sum of their area-weighted normals at
        /// their area-weighted centroid.
        /// </summary>
        private struct WindingNode
        {
            /// <summary>Range of the node's triangles in the Morton order.</summary>
            public int Start, Count;
            /// <summary>Children, -1 for a leaf.</summary>
            public int Left, Right;
            public float3 Center;
            /// <summary>Sum of the triangles' area vectors (half the cross product of two edges).</summary>
            public float3 Dipole;
            public float Area;
            /// <summary>Distance from <see cref="Center"/> that encloses the node's triangles.</summary>
            public float Radius;
        }

        /// <summary>
        /// Sorts the triangles by the Morton code of their centroids and halves the sorted range down to
        /// leaves of <see cref="WindingLeafTriangles"/>; children follow their parents in the node list.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct BuildWindingTreeJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public NativeArray<int> Order;
            public NativeList<WindingNode> Nodes;

            public void Execute()
            {
                int count = Triangles.Length / 3;
                float3 lo = float.MaxValue, hi = float.MinValue;
                for (int t = 0; t < count; t++)
                {
                    float3 c = Centroid(t);
                    lo = math.min(lo, c);
                    hi = math.max(hi, c);
                }
                float3 scale = 1023.0f / math.max(hi - lo, 1e-12f);
                var keys = new NativeArray<ulong>(count, Allocator.Temp);
                for (int t = 0; t < count; t++)
                {
                    uint3 q = (uint3)math.clamp((Centroid(t) - lo) * scale, 0.0f, 1023.0f);
                    ulong morton = Spread(q.x) | (Spread(q.y) << 1) | (Spread(q.z) << 2);
                    keys[t] = (morton << 32) | (uint)t;
                }
                keys.Sort();
                for (int t = 0; t < count; t++)
                {
                    Order[t] = (int)(keys[t] & 0xffffffffUL);
                }
                keys.Dispose();

                Nodes.Add(new WindingNode { Start = 0, Count = count, Left = -1, Right = -1 });
                for (int n = 0; n < Nodes.Length; n++)
                {
                    WindingNode node = Nodes[n];
                    if (node.Count <= WindingLeafTriangles)
                    {
                        continue;
                    }
                    int half = node.Count / 2;
                    node.Left = Nodes.Length;
                    Nodes.Add(new WindingNode { Start = node.Start, Count = half, Left = -1, Right = -1 });
                    node.Right = Nodes.Length;
                    Nodes.Add(new WindingNode { Start = node.Start + half, Count = node.Count - half, Left = -1, Right = -1 });
                    Nodes[n] = node;
                }
                for (int n = Nodes.Length - 1; n >= 0; n--)
                {
                    WindingNode node = Nodes[n];
                    if (node.Left < 0)
                    {
                        float3 weighted = float3.zero, mean = float3.zero;
                        for (int i = node.Start; i < node.Start + node.Count; i++)
                        {
                            int t = Order[i];
                            float3 a = Vertices[Triangles[3 * t]], b = Vertices[Triangles[3 * t + 1]], c = Vertices[Triangles[3 * t + 2]];
                            float3 areaVector = 0.5f * math.cross(b - a, c - a);
                            float area = math.length(areaVector);
                            node.Dipole += areaVector;
                            node.Area += area;
                            weighted += area * (a + b + c) / 3.0f;
                            mean += (a + b + c) / 3.0f;
                        }
                        node.Center = node.Area > 0.0f ? weighted / node.Area : mean / node.Count;
                        for (int i = node.Start; i < node.Start + node.Count; i++)
                        {
                            int t = Order[i];
                            for (int k = 0; k < 3; k++)
                            {
                                node.Radius = math.max(node.Radius, math.distance(node.Center, Vertices[Triangles[3 * t + k]]));
                            }
                        }
                    }
                    else
                    {
                        WindingNode left = Nodes[node.Left], right = Nodes[node.Right];
                        node.Dipole = left.Dipole + right.Dipole;
                        node.Area = left.Area + right.Area;
                        node.Center = node.Area > 0.0f
                            ? (left.Area * left.Center + right.Area * right.Center) / node.Area
                            : 0.5f * (left.Center + right.Center);
                        node.Radius = math.max(math.distance(node.Center, left.Center) + left.Radius,
                            math.distance(node.Center, right.Center) + right.Radius);
                    }
                    Nodes[n] = node;
                }
            }

            private float3 Centroid(int t) =>
                (Vertices[Triangles[3 * t]] + Vertices[Triangles[3 * t + 1]] + Vertices[Triangles[3 * t + 2]]) / 3.0f;

            /// <summary>Inserts two zero bits between the 10 low bits of <paramref name="v"/>.</summary>
            private static ulong Spread(uint v)
            {
                ulong x = v & 0x3ffUL;
                x = (x | (x << 16)) & 0x30000ffUL;
                x = (x | (x << 8)) & 0x300f00fUL;
                x = (x | (x << 4)) & 0x30c30c3UL;
                x = (x | (x << 2)) & 0x9249249UL;
                return x;
            }
        }

        /// <summary>
        /// Generalized winding number at every grid point: the solid angle the surface subtends, over 4π.
        /// A point is inside when its magnitude exceeds 1/2, so holes are closed and the orientation of a
        /// consistently oriented mesh does not matter.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct InsideWindingJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            [ReadOnly] public NativeArray<int> Order;
            [ReadOnly] public NativeArray<WindingNode> Nodes;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            [WriteOnly] public NativeArray<byte> Inside;

            public void Execute(int index)
            {
                int x = index % PointDims.x;
                int y = index / PointDims.x % PointDims.y;
                int z = index / (PointDims.x * PointDims.y);
                float3 q = Origin + new float3(x, y, z) * CellSize;
                float solidAngle = 0.0f;
                var stack = new FixedList512Bytes<int>();
                stack.Add(0);
                while (stack.Length > 0)
                {
                    int n = stack[stack.Length - 1];
                    stack.RemoveAt(stack.Length - 1);
                    WindingNode node = Nodes[n];
                    float3 d = node.Center - q;
                    float distance = math.length(d);
                    if (distance > WindingBeta * node.Radius)
                    {
                        solidAngle += math.dot(node.Dipole, d) / (distance * distance * distance);
                    }
                    else if (node.Left < 0)
                    {
                        for (int i = node.Start; i < node.Start + node.Count; i++)
                        {
                            int t = Order[i];
                            solidAngle += SolidAngle(Vertices[Triangles[3 * t]] - q, Vertices[Triangles[3 * t + 1]] - q,
                                Vertices[Triangles[3 * t + 2]] - q);
                        }
                    }
                    else
                    {
                        stack.Add(node.Right);
                        stack.Add(node.Left);
                    }
                }
                Inside[index] = (byte)(math.abs(solidAngle) > 2.0f * math.PI ? 1 : 0);
            }

            /// <summary>Signed solid angle of a triangle seen from the origin (Van Oosterom and Strackee 1983).</summary>
            private static float SolidAngle(float3 a, float3 b, float3 c)
            {
                float la = math.length(a), lb = math.length(b), lc = math.length(c);
                float det = math.dot(a, math.cross(b, c));
                float denominator = la * lb * lc + math.dot(a, b) * lc + math.dot(b, c) * la + math.dot(c, a) * lb;
                return 2.0f * math.atan2(det, denominator);
            }
        }

        /// <summary>Grid points within 1.5 cells of triangle t (the range of the exact distance), clamped to the grid.</summary>
        private static void NearRange(NativeArray<float3> vertices, NativeArray<int> triangles, int t, float3 origin,
            float cellSize, int3 pointDims, out int3 p0, out int3 p1)
        {
            float3 a = vertices[triangles[t]];
            float3 b = vertices[triangles[t + 1]];
            float3 c = vertices[triangles[t + 2]];
            float3 lo = math.min(a, math.min(b, c)) - 1.5f * cellSize;
            float3 hi = math.max(a, math.max(b, c)) + 1.5f * cellSize;
            p0 = math.max((int3)math.ceil((lo - origin) / cellSize), 0);
            p1 = math.min((int3)math.floor((hi - origin) / cellSize), pointDims - 1);
        }

        /// <summary>Lists, for every z layer of grid points, the triangles whose distance range (<see cref="NearRange"/>) includes it.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct BinSlabTrianglesJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            /// <summary>Layer z's triangles are SlabTriangles[SlabStart[z] .. SlabStart[z + 1]).</summary>
            public NativeArray<int> SlabStart;
            public NativeList<int> SlabTriangles;

            public void Execute()
            {
                for (int pass = 0; pass < 2; pass++)
                {
                    var cursor = new NativeArray<int>(pass == 0 ? 0 : SlabStart.Length - 1, Allocator.Temp);
                    for (int t = 0; t < Triangles.Length; t += 3)
                    {
                        NearRange(Vertices, Triangles, t, Origin, CellSize, PointDims, out int3 p0, out int3 p1);
                        if (p0.x > p1.x || p0.y > p1.y)
                        {
                            continue;
                        }
                        for (int z = p0.z; z <= p1.z; z++)
                        {
                            if (pass == 0)
                            {
                                SlabStart[z + 1]++;
                            }
                            else
                            {
                                SlabTriangles[SlabStart[z] + cursor[z]++] = t / 3;
                            }
                        }
                    }
                    if (pass == 0)
                    {
                        for (int z = 1; z < SlabStart.Length; z++)
                        {
                            SlabStart[z] += SlabStart[z - 1];
                        }
                        SlabTriangles.ResizeUninitialized(SlabStart[SlabStart.Length - 1]);
                    }
                    cursor.Dispose();
                }
            }
        }

        /// <summary>Exact distance to the surface for grid points within 1.5 cells of a triangle, one z layer per index.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct NearDistanceJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            [ReadOnly] public NativeArray<int> SlabStart;
            [ReadOnly] public NativeArray<int> SlabTriangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            [NativeDisableParallelForRestriction] public NativeArray<float> Distance;

            public void Execute(int z)
            {
                float far = FarCells * CellSize;
                int layer = PointDims.x * PointDims.y;
                for (int i = 0; i < layer; i++)
                {
                    Distance[z * layer + i] = far;
                }
                for (int e = SlabStart[z]; e < SlabStart[z + 1]; e++)
                {
                    int t = 3 * SlabTriangles[e];
                    float3 a = Vertices[Triangles[t]];
                    float3 b = Vertices[Triangles[t + 1]];
                    float3 c = Vertices[Triangles[t + 2]];
                    NearRange(Vertices, Triangles, t, Origin, CellSize, PointDims, out int3 p0, out int3 p1);
                    for (int y = p0.y; y <= p1.y; y++)
                    {
                        for (int x = p0.x; x <= p1.x; x++)
                        {
                            float3 p = Origin + new float3(x, y, z) * CellSize;
                            float dist = math.distance(p, Geometry.ClosestPointOnTriangle(p, a, b, c));
                            int index = x + PointDims.x * (y + PointDims.y * z);
                            Distance[index] = math.min(Distance[index], dist);
                        }
                    }
                }
            }
        }
    }
}
