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
        /// <summary>
        /// Edges used by one triangle (holes) and by more than two (non-manifold), after welding equal
        /// positions. The voxelizer's ray parity needs no boundary edges; non-manifold edges where closed
        /// sheets touch (the Stanford dragon has 108) do not disturb it.
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
                var weld = new NativeParallelHashMap<float3, int>(Positions.Length, Allocator.Temp);
                var id = new NativeArray<int>(Positions.Length, Allocator.Temp);
                int welded = 0;
                for (int i = 0; i < Positions.Length; i++)
                {
                    if (!weld.TryGetValue(Positions[i], out int w))
                    {
                        w = welded++;
                        weld.Add(Positions[i], w);
                    }
                    id[i] = w;
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
                weld.Dispose();
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
            float maxSq = maxEdge * maxEdge;
            for (int pass = 0; pass < maxPasses; pass++)
            {
                var midpoints = new Dictionary<long, int>();
                var output = new List<int>(triangles.Count * 2);
                bool changed = false;

                int Mid(int a, int b)
                {
                    if (math.distancesq(positions[a], positions[b]) <= maxSq)
                    {
                        return -1;
                    }
                    long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                    if (!midpoints.TryGetValue(key, out int m))
                    {
                        m = positions.Count;
                        positions.Add(0.5f * (positions[a] + positions[b]));
                        normals.Add(math.normalizesafe(normals[a] + normals[b]));
                        uvs.Add(0.5f * (uvs[a] + uvs[b]));
                        midpoints.Add(key, m);
                    }
                    return m;
                }

                for (int t = 0; t < triangles.Count; t += 3)
                {
                    int a = triangles[t], b = triangles[t + 1], c = triangles[t + 2];
                    int ab = Mid(a, b), bc = Mid(b, c), ca = Mid(c, a);
                    int splits = (ab >= 0 ? 1 : 0) + (bc >= 0 ? 1 : 0) + (ca >= 0 ? 1 : 0);
                    changed |= splits > 0;
                    if (splits == 0)
                    {
                        output.AddRange(new[] { a, b, c });
                    }
                    else if (splits == 3)
                    {
                        output.AddRange(new[] { a, ab, ca, ab, b, bc, ca, bc, c, ab, bc, ca });
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
                            output.AddRange(new[] { a, ab, c, ab, b, c });
                        }
                        else
                        {
                            output.AddRange(new[] { ab, b, bc, a, ab, bc, a, bc, c });
                        }
                    }
                }
                triangles.Clear();
                triangles.AddRange(output);
                if (!changed)
                {
                    return;
                }
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
