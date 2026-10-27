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

        /// <summary>
        /// Voxelizes a closed mesh given in rest space. A cell is kept when one of its
        /// corners is inside the mesh or when it contains a mesh vertex, so every
        /// vertex can be embedded.
        /// </summary>
        public static VoxelGrid Voxelize(float3[] vertices, int[] triangles, float cellSize)
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

            var parity = new InsideParityJob
            {
                Vertices = verts,
                Triangles = tris,
                Origin = grid.Origin,
                CellSize = cellSize,
                PointDims = pd,
                Inside = inside,
            }.Schedule(pd.y * pd.z, 4);
            var near = new NearDistanceJob
            {
                Vertices = verts,
                Triangles = tris,
                Origin = grid.Origin,
                CellSize = cellSize,
                PointDims = pd,
                Distance = distance,
            }.Schedule();
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

        /// <summary>Even-odd ray parity along +x for every grid point row.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct InsideParityJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            [NativeDisableParallelForRestriction] public NativeArray<byte> Inside;

            public void Execute(int row)
            {
                int j = row % PointDims.y;
                int k = row / PointDims.y;
                // Tiny irrational offsets keep rays off shared triangle edges.
                float ry = Origin.y + j * CellSize + CellSize * 1.37e-4f;
                float rz = Origin.z + k * CellSize + CellSize * 2.91e-4f;
                var hits = new NativeList<float>(16, Allocator.Temp);
                for (int t = 0; t < Triangles.Length; t += 3)
                {
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

        /// <summary>Exact distance to the surface for grid points within 1.5 cells of a triangle.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct NearDistanceJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 PointDims;
            public NativeArray<float> Distance;

            public void Execute()
            {
                float far = FarCells * CellSize;
                for (int i = 0; i < Distance.Length; i++)
                {
                    Distance[i] = far;
                }
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    float3 a = Vertices[Triangles[t]];
                    float3 b = Vertices[Triangles[t + 1]];
                    float3 c = Vertices[Triangles[t + 2]];
                    float3 lo = math.min(a, math.min(b, c)) - 1.5f * CellSize;
                    float3 hi = math.max(a, math.max(b, c)) + 1.5f * CellSize;
                    int3 p0 = math.max((int3)math.ceil((lo - Origin) / CellSize), 0);
                    int3 p1 = math.min((int3)math.floor((hi - Origin) / CellSize), PointDims - 1);
                    for (int z = p0.z; z <= p1.z; z++)
                    {
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
}
