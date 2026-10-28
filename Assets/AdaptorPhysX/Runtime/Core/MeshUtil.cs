using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace AdaptorPhysX
{
    public static class MeshUtil
    {
        /// <summary>Vertices closer than this fraction of the bounding box diagonal are welded by <see cref="CountOpenEdges"/>.</summary>
        public const float WeldTolerance = 1e-6f;

        /// <summary>
        /// Edges used by one triangle (holes) and by more than two (non-manifold), after welding vertices closer
        /// than <see cref="WeldTolerance"/> of the bounding box diagonal: generated seams (the ellipsoid at
        /// theta = 0 and 2 pi) differ in the last bits. The voxelizer's ray parity needs no boundary edges;
        /// non-manifold edges where closed sheets touch (the Stanford dragon has 108) do not disturb it.
        /// </summary>
        public static void CountOpenEdges(float3[] positions, int[] triangles, out int boundaryEdges, out int nonManifoldEdges)
        {
            using var nativePositions = new NativeArray<float3>(positions, Allocator.TempJob);
            using var nativeTriangles = new NativeArray<int>(triangles, Allocator.TempJob);
            using var counts = new NativeArray<int>(2, Allocator.TempJob);
            new OpenEdgesJob { Positions = nativePositions, Triangles = nativeTriangles, Counts = counts }.Run();
            boundaryEdges = counts[0];
            nonManifoldEdges = counts[1];
        }

        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct OpenEdgesJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<int> Triangles;
            public NativeArray<int> Counts;

            public void Execute()
            {
                float3 lo = float.MaxValue, hi = float.MinValue;
                for (int i = 0; i < Positions.Length; i++)
                {
                    lo = math.min(lo, Positions[i]);
                    hi = math.max(hi, Positions[i]);
                }
                float tolerance = math.max(WeldTolerance * math.length(hi - lo), 1e-12f);
                // Each welded vertex is stored in the grid cell (of the tolerance's size) of its first position;
                // a position looks for one within the tolerance in the 27 cells around it.
                var grid = new NativeParallelMultiHashMap<int3, int>(Positions.Length, Allocator.Temp);
                var id = new NativeArray<int>(Positions.Length, Allocator.Temp);
                int welded = 0;
                for (int i = 0; i < Positions.Length; i++)
                {
                    float3 p = Positions[i];
                    int3 cell = (int3)math.floor(p / tolerance);
                    int found = -1;
                    for (int n = 0; n < 27 && found < 0; n++)
                    {
                        int3 key = cell + new int3(n % 3 - 1, n / 3 % 3 - 1, n / 9 - 1);
                        if (!grid.TryGetFirstValue(key, out int other, out NativeParallelMultiHashMapIterator<int3> it))
                        {
                            continue;
                        }
                        do
                        {
                            if (math.distancesq(Positions[other], p) <= tolerance * tolerance)
                            {
                                found = id[other];
                                break;
                            }
                        } while (grid.TryGetNextValue(out other, ref it));
                    }
                    if (found < 0)
                    {
                        found = welded++;
                        grid.Add(cell, i);
                    }
                    id[i] = found;
                }
                var keys = new NativeList<long>(Triangles.Length, Allocator.Temp);
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    for (int e = 0; e < 3; e++)
                    {
                        int a = id[Triangles[t + e]], b = id[Triangles[t + (e + 1) % 3]];
                        if (a != b)
                        {
                            keys.Add(a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a);
                        }
                    }
                }
                keys.Sort();
                int boundary = 0, nonManifold = 0;
                for (int i = 0; i < keys.Length;)
                {
                    int j = i + 1;
                    while (j < keys.Length && keys[j] == keys[i])
                    {
                        j++;
                    }
                    boundary += j - i == 1 ? 1 : 0;
                    nonManifold += j - i > 2 ? 1 : 0;
                    i = j;
                }
                Counts[0] = boundary;
                Counts[1] = nonManifold;
                keys.Dispose();
                id.Dispose();
                grid.Dispose();
            }
        }

        /// <summary>
        /// Area-weighted vertex normals of the triangles (<paramref name="geometric"/>), and how well
        /// <paramref name="normals"/> agree with them: the mean over the triangles of the cosine between the
        /// triangle's normal and the sum of its vertices' normals (1 when they agree, about 0 for normals that
        /// do not follow the surface, -1 when they point inward).
        /// </summary>
        public static float NormalAgreement(float3[] positions, float3[] normals, int[] triangles, out float3[] geometric)
        {
            using var nativePositions = new NativeArray<float3>(positions, Allocator.TempJob);
            using var nativeNormals = new NativeArray<float3>(normals, Allocator.TempJob);
            using var nativeTriangles = new NativeArray<int>(triangles, Allocator.TempJob);
            using var result = new NativeArray<float3>(positions.Length, Allocator.TempJob);
            using var agreement = new NativeArray<float>(1, Allocator.TempJob);
            new NormalAgreementJob
            {
                Positions = nativePositions,
                Normals = nativeNormals,
                Triangles = nativeTriangles,
                Geometric = result,
                Agreement = agreement,
            }.Run();
            geometric = result.ToArray();
            return agreement[0];
        }

        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct NormalAgreementJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<float3> Normals;
            [ReadOnly] public NativeArray<int> Triangles;
            public NativeArray<float3> Geometric;
            public NativeArray<float> Agreement;

            public void Execute()
            {
                double sum = 0.0;
                int count = 0;
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    int a = Triangles[t], b = Triangles[t + 1], c = Triangles[t + 2];
                    float3 area = math.cross(Positions[b] - Positions[a], Positions[c] - Positions[a]);
                    Geometric[a] += area;
                    Geometric[b] += area;
                    Geometric[c] += area;
                    float3 given = Normals[a] + Normals[b] + Normals[c];
                    float lengths = math.length(area) * math.length(given);
                    if (lengths > 0.0f)
                    {
                        sum += math.dot(area, given) / lengths;
                        count++;
                    }
                }
                for (int i = 0; i < Geometric.Length; i++)
                {
                    Geometric[i] = math.normalizesafe(Geometric[i], new float3(0.0f, 1.0f, 0.0f));
                }
                Agreement[0] = count > 0 ? (float)(sum / count) : 0.0f;
            }
        }

        /// <summary>
        /// Splits every edge longer than <paramref name="maxEdge"/> (shared midpoints, so no
        /// T-junctions) until none remain. Incisions on the skin follow triangle edges, so the
        /// render mesh must be finer than the lattice.
        /// </summary>
        public static void Subdivide(List<float3> positions, List<float3> normals, List<Vector2> uvs, List<int> triangles,
            float maxEdge, int maxPasses = 8)
        {
            var uv = new float2[uvs.Count];
            for (int i = 0; i < uv.Length; i++)
            {
                uv[i] = uvs[i];
            }
            using var nativePositions = new NativeList<float3>(positions.Count, Allocator.TempJob);
            using var nativeNormals = new NativeList<float3>(normals.Count, Allocator.TempJob);
            using var nativeUvs = new NativeList<float2>(uv.Length, Allocator.TempJob);
            using var nativeTriangles = new NativeList<int>(triangles.Count, Allocator.TempJob);
            using (var array = new NativeArray<float3>(positions.ToArray(), Allocator.Temp)) nativePositions.AddRange(array);
            using (var array = new NativeArray<float3>(normals.ToArray(), Allocator.Temp)) nativeNormals.AddRange(array);
            using (var array = new NativeArray<float2>(uv, Allocator.Temp)) nativeUvs.AddRange(array);
            using (var array = new NativeArray<int>(triangles.ToArray(), Allocator.Temp)) nativeTriangles.AddRange(array);
            new SubdivideJob
            {
                Positions = nativePositions,
                Normals = nativeNormals,
                Uvs = nativeUvs,
                Triangles = nativeTriangles,
                MaxEdgeSq = maxEdge * maxEdge,
                MaxPasses = maxPasses,
            }.Run();
            positions.Clear();
            positions.AddRange(nativePositions.AsArray().ToArray());
            normals.Clear();
            normals.AddRange(nativeNormals.AsArray().ToArray());
            uvs.Clear();
            foreach (float2 v in nativeUvs.AsArray().ToArray())
            {
                uvs.Add(v);
            }
            triangles.Clear();
            triangles.AddRange(nativeTriangles.AsArray().ToArray());
        }

        /// <summary>
        /// Each pass splits the edges longer than the limit at their midpoints, created in the order the
        /// triangles meet them, and replaces every triangle by its 1–4 parts.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct SubdivideJob : IJob
        {
            public NativeList<float3> Positions;
            public NativeList<float3> Normals;
            public NativeList<float2> Uvs;
            public NativeList<int> Triangles;
            public float MaxEdgeSq;
            public int MaxPasses;

            public void Execute()
            {
                var midpoints = new NativeParallelHashMap<long, int>(math.max(Triangles.Length, 64), Allocator.Temp);
                var output = new NativeList<int>(Triangles.Length * 2, Allocator.Temp);
                for (int pass = 0; pass < MaxPasses; pass++)
                {
                    midpoints.Clear();
                    output.Clear();
                    bool changed = false;
                    for (int t = 0; t < Triangles.Length; t += 3)
                    {
                        int a = Triangles[t], b = Triangles[t + 1], c = Triangles[t + 2];
                        int ab = Mid(a, b, ref midpoints), bc = Mid(b, c, ref midpoints), ca = Mid(c, a, ref midpoints);
                        int splits = (ab >= 0 ? 1 : 0) + (bc >= 0 ? 1 : 0) + (ca >= 0 ? 1 : 0);
                        changed |= splits > 0;
                        if (splits == 0)
                        {
                            Add(ref output, a, b, c);
                        }
                        else if (splits == 3)
                        {
                            Add(ref output, a, ab, ca);
                            Add(ref output, ab, b, bc);
                            Add(ref output, ca, bc, c);
                            Add(ref output, ab, bc, ca);
                        }
                        else
                        {
                            // Rotate so that the split edges come first: (a,b) always split.
                            while (ab < 0 || (splits == 2 && bc < 0))
                            {
                                (a, b, c) = (b, c, a);
                                (ab, bc, ca) = (bc, ca, ab);
                            }
                            if (splits == 1)
                            {
                                Add(ref output, a, ab, c);
                                Add(ref output, ab, b, c);
                            }
                            else
                            {
                                Add(ref output, ab, b, bc);
                                Add(ref output, a, ab, bc);
                                Add(ref output, a, bc, c);
                            }
                        }
                    }
                    Triangles.Clear();
                    Triangles.AddRange(output.AsArray());
                    if (!changed)
                    {
                        break;
                    }
                }
                output.Dispose();
                midpoints.Dispose();
            }

            private int Mid(int a, int b, ref NativeParallelHashMap<long, int> midpoints)
            {
                if (math.distancesq(Positions[a], Positions[b]) <= MaxEdgeSq)
                {
                    return -1;
                }
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (!midpoints.TryGetValue(key, out int m))
                {
                    m = Positions.Length;
                    Positions.Add(0.5f * (Positions[a] + Positions[b]));
                    Normals.Add(math.normalizesafe(Normals[a] + Normals[b]));
                    Uvs.Add(0.5f * (Uvs[a] + Uvs[b]));
                    midpoints.Add(key, m);
                }
                return m;
            }

            private static void Add(ref NativeList<int> output, int a, int b, int c)
            {
                output.Add(a);
                output.Add(b);
                output.Add(c);
            }
        }

        /// <summary>Box centred at the origin with independent (sharp) faces.</summary>
        public static Mesh CreateBox(Vector3 size, int segments = 8)
        {
            var positions = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var triangles = new List<int>();
            Vector3[] axes = { Vector3.right, Vector3.up, Vector3.forward };
            for (int axis = 0; axis < 3; axis++)
            {
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    Vector3 n = axes[axis] * sign;
                    Vector3 u = axes[(axis + 1) % 3];
                    Vector3 v = axes[(axis + 2) % 3];
                    if (sign < 0)
                    {
                        (u, v) = (v, u);
                    }
                    int start = positions.Count;
                    for (int j = 0; j <= segments; j++)
                    {
                        for (int i = 0; i <= segments; i++)
                        {
                            float s = i / (float)segments - 0.5f, t = j / (float)segments - 0.5f;
                            Vector3 p = n * 0.5f + u * s + v * t;
                            positions.Add(Vector3.Scale(p, size));
                            normals.Add(n);
                            uvs.Add(new Vector2(s + 0.5f, t + 0.5f));
                        }
                    }
                    for (int j = 0; j < segments; j++)
                    {
                        for (int i = 0; i < segments; i++)
                        {
                            int i0 = start + j * (segments + 1) + i;
                            int i1 = i0 + 1, i2 = i0 + segments + 1, i3 = i2 + 1;
                            triangles.AddRange(new[] { i0, i1, i2, i1, i3, i2 });
                        }
                    }
                }
            }
            return Build("ApxBox", positions, normals, uvs, triangles);
        }

        /// <summary>Ellipsoid centred at the origin.</summary>
        public static Mesh CreateEllipsoid(Vector3 radii, int rings = 24, int sectors = 48)
        {
            var positions = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var triangles = new List<int>();
            for (int r = 0; r <= rings; r++)
            {
                float phi = Mathf.PI * r / rings;
                for (int s = 0; s <= sectors; s++)
                {
                    float theta = 2.0f * Mathf.PI * s / sectors;
                    var d = new Vector3(Mathf.Sin(phi) * Mathf.Cos(theta), Mathf.Cos(phi), Mathf.Sin(phi) * Mathf.Sin(theta));
                    Vector3 p = Vector3.Scale(d, radii);
                    positions.Add(p);
                    normals.Add(new Vector3(p.x / (radii.x * radii.x), p.y / (radii.y * radii.y),
                        p.z / (radii.z * radii.z)).normalized);
                    uvs.Add(new Vector2(s / (float)sectors, r / (float)rings));
                }
            }
            for (int r = 0; r < rings; r++)
            {
                for (int s = 0; s < sectors; s++)
                {
                    int i0 = r * (sectors + 1) + s, i1 = i0 + 1, i2 = i0 + sectors + 1, i3 = i2 + 1;
                    if (r > 0)
                    {
                        triangles.AddRange(new[] { i0, i1, i2 });
                    }
                    if (r < rings - 1)
                    {
                        triangles.AddRange(new[] { i1, i3, i2 });
                    }
                }
            }
            return Build("ApxEllipsoid", positions, normals, uvs, triangles);
        }

        private static Mesh Build(string name, List<Vector3> positions, List<Vector3> normals, List<Vector2> uvs,
            List<int> triangles)
        {
            var mesh = new Mesh { name = name, indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(positions);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
