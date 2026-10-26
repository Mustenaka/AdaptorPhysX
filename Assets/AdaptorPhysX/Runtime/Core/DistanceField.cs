using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Signed distance field of a triangle mesh sampled at the points of a regular grid, in the
    /// mesh's local space (negative inside). Collider shape for MeshColliders and for instruments
    /// of any shape (jaws, curved scissors).
    /// </summary>
    public sealed class DistanceField
    {
        public float3 Origin;
        public float CellSize;
        /// <summary>Grid points per axis, x fastest.</summary>
        public int3 Dims;
        public float[] Values;

        public int PointCount => Dims.x * Dims.y * Dims.z;

        /// <summary>
        /// Samples the mesh on a grid with <paramref name="resolution"/> cells along its longest
        /// axis, padded by <paramref name="padding"/> (at least two cells) on every side. Inside
        /// and outside are decided by the majority of even-odd ray tests along x, y and z, so a
        /// small hole flips at most one of the three votes; distances are exact within two cells
        /// of the surface and propagated as closest points beyond.
        /// </summary>
        public static DistanceField Bake(Vector3[] vertices, int[] triangles, int resolution, float padding)
        {
            float3 min = vertices[0], max = vertices[0];
            var verts = new float3[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                verts[i] = vertices[i];
                min = math.min(min, verts[i]);
                max = math.max(max, verts[i]);
            }
            float cell = math.max(math.cmax(max - min), 1e-6f) / math.max(resolution, 2);
            float pad = math.max(padding, 2.0f * cell);
            var field = new DistanceField { CellSize = cell, Origin = min - pad };
            field.Dims = (int3)math.ceil((max + pad - field.Origin) / cell) + 1;
            int count = field.PointCount;

            using var nativeVerts = new NativeArray<float3>(verts, Allocator.TempJob);
            using var nativeTris = new NativeArray<int>(triangles, Allocator.TempJob);
            using var votes = new NativeArray<byte>(count, Allocator.TempJob);
            using var closest = new NativeArray<float3>(count, Allocator.TempJob);
            using var values = new NativeArray<float>(count, Allocator.TempJob);
            JobHandle handle = default;
            for (int axis = 0; axis < 3; axis++)
            {
                int3 d = field.Dims;
                int rows = d[(axis + 1) % 3] * d[(axis + 2) % 3];
                handle = new FieldParityJob
                {
                    Vertices = nativeVerts,
                    Triangles = nativeTris,
                    Origin = field.Origin,
                    CellSize = cell,
                    Dims = d,
                    Axis = axis,
                    Votes = votes,
                }.Schedule(rows, 4, handle);
            }
            handle = JobHandle.CombineDependencies(handle, new FieldClosestJob
            {
                Vertices = nativeVerts,
                Triangles = nativeTris,
                Origin = field.Origin,
                CellSize = cell,
                Dims = field.Dims,
                Closest = closest,
            }.Schedule());
            new FieldSignJob
            {
                Origin = field.Origin,
                CellSize = cell,
                Dims = field.Dims,
                Votes = votes,
                Closest = closest,
                Values = values,
            }.Schedule(handle).Complete();
            field.Values = values.ToArray();
            return field;
        }

        /// <summary>
        /// Distance and gradient at local point <paramref name="p"/> of the field stored at
        /// <paramref name="offset"/> in <paramref name="data"/>: trilinear inside the grid; outside
        /// it, the distance to the grid box is added to the value at the nearest grid point.
        /// </summary>
        public static float Sample(NativeArray<float> data, int offset, int3 dims, float3 origin, float cellSize,
            float3 p, out float3 gradient)
        {
            float3 g = (p - origin) / cellSize;
            float3 clamped = math.clamp(g, 0.0f, (float3)(dims - 1) - 1e-4f);
            int3 c = (int3)math.floor(clamped);
            float3 u = clamped - c;
            float value = 0.0f;
            gradient = float3.zero;
            for (int k = 0; k < 8; k++)
            {
                int3 q = c + Lattice.CornerOffset(k);
                float v = data[offset + q.x + dims.x * (q.y + dims.y * q.z)];
                value += Geometry.TrilinearWeight(k, u) * v;
                gradient += Geometry.TrilinearGradient(k, u) * v;
            }
            gradient /= cellSize;
            float outside = math.length(g - clamped) * cellSize;
            if (outside > 0.0f)
            {
                gradient = math.normalizesafe((g - clamped) * cellSize, gradient);
                value += outside;
            }
            return value;
        }

        /// <summary>Votes for "inside" from even-odd ray parity along one axis, one grid row per index.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct FieldParityJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 Dims;
            public int Axis;
            [NativeDisableParallelForRestriction] public NativeArray<byte> Votes;

            public void Execute(int row)
            {
                int a1 = (Axis + 1) % 3, a2 = (Axis + 2) % 3;
                int j = row % Dims[a1];
                int k = row / Dims[a1];
                // Tiny irrational offsets keep rays off shared triangle edges.
                float2 ray = new float2(Origin[a1] + j * CellSize + CellSize * 1.37e-4f,
                    Origin[a2] + k * CellSize + CellSize * 2.91e-4f);
                var hits = new NativeList<float>(16, Allocator.Temp);
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    float3 a = Vertices[Triangles[t]], b = Vertices[Triangles[t + 1]], c = Vertices[Triangles[t + 2]];
                    float2 a2d = new float2(a[a1], a[a2]), b2d = new float2(b[a1], b[a2]), c2d = new float2(c[a1], c[a2]);
                    float d = (b2d.x - a2d.x) * (c2d.y - a2d.y) - (c2d.x - a2d.x) * (b2d.y - a2d.y);
                    if (math.abs(d) < 1e-20f)
                    {
                        continue;
                    }
                    float w1 = ((ray.x - a2d.x) * (c2d.y - a2d.y) - (c2d.x - a2d.x) * (ray.y - a2d.y)) / d;
                    float w2 = ((b2d.x - a2d.x) * (ray.y - a2d.y) - (ray.x - a2d.x) * (b2d.y - a2d.y)) / d;
                    float w0 = 1.0f - w1 - w2;
                    if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f)
                    {
                        continue;
                    }
                    hits.Add(w0 * a[Axis] + w1 * b[Axis] + w2 * c[Axis]);
                }
                hits.Sort();
                int h = 0;
                for (int i = 0; i < Dims[Axis]; i++)
                {
                    float x = Origin[Axis] + i * CellSize;
                    while (h < hits.Length && hits[h] < x)
                    {
                        h++;
                    }
                    int3 p = default;
                    p[Axis] = i;
                    p[a1] = j;
                    p[a2] = k;
                    int index = p.x + Dims.x * (p.y + Dims.y * p.z);
                    Votes[index] = (byte)(Votes[index] + (h & 1));
                }
                hits.Dispose();
            }
        }

        /// <summary>Closest surface point of every grid point within two cells of a triangle (NaN elsewhere).</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct FieldClosestJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int> Triangles;
            public float3 Origin;
            public float CellSize;
            public int3 Dims;
            public NativeArray<float3> Closest;

            public void Execute()
            {
                for (int i = 0; i < Closest.Length; i++)
                {
                    Closest[i] = new float3(float.NaN);
                }
                var distanceSq = new NativeArray<float>(Closest.Length, Allocator.Temp);
                for (int i = 0; i < distanceSq.Length; i++)
                {
                    distanceSq[i] = float.MaxValue;
                }
                float band = 2.0f * CellSize;
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    float3 a = Vertices[Triangles[t]], b = Vertices[Triangles[t + 1]], c = Vertices[Triangles[t + 2]];
                    int3 p0 = math.max((int3)math.ceil((math.min(a, math.min(b, c)) - band - Origin) / CellSize), 0);
                    int3 p1 = math.min((int3)math.floor((math.max(a, math.max(b, c)) + band - Origin) / CellSize), Dims - 1);
                    for (int z = p0.z; z <= p1.z; z++)
                    for (int y = p0.y; y <= p1.y; y++)
                    for (int x = p0.x; x <= p1.x; x++)
                    {
                        float3 p = Origin + new float3(x, y, z) * CellSize;
                        float3 q = Geometry.ClosestPointOnTriangle(p, a, b, c);
                        float dsq = math.distancesq(p, q);
                        int index = x + Dims.x * (y + Dims.y * z);
                        if (dsq < distanceSq[index])
                        {
                            distanceSq[index] = dsq;
                            Closest[index] = q;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Propagates closest points from the band to the whole grid (two forward and backward
        /// sweeps over the 26-neighbourhood) and signs the distances by the parity votes.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct FieldSignJob : IJob
        {
            public float3 Origin;
            public float CellSize;
            public int3 Dims;
            [ReadOnly] public NativeArray<byte> Votes;
            public NativeArray<float3> Closest;
            public NativeArray<float> Values;

            public void Execute()
            {
                for (int pass = 0; pass < 4; pass++)
                {
                    int step = (pass & 1) == 0 ? 1 : -1;
                    int3 start = step > 0 ? int3.zero : Dims - 1;
                    for (int z = start.z; z >= 0 && z < Dims.z; z += step)
                    for (int y = start.y; y >= 0 && y < Dims.y; y += step)
                    for (int x = start.x; x >= 0 && x < Dims.x; x += step)
                    {
                        int index = x + Dims.x * (y + Dims.y * z);
                        float3 p = Origin + new float3(x, y, z) * CellSize;
                        float3 best = Closest[index];
                        float bestSq = math.any(math.isnan(best)) ? float.MaxValue : math.distancesq(p, best);
                        // Neighbours already visited in this sweep direction.
                        for (int dz = -1; dz <= 0; dz++)
                        for (int dy = -1; dy <= 1; dy++)
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (dz == 0 && (dy > 0 || (dy == 0 && dx >= 0)))
                            {
                                continue;
                            }
                            int3 n = new int3(x, y, z) + new int3(dx, dy, dz) * step;
                            if (math.any(n < 0) || math.any(n >= Dims))
                            {
                                continue;
                            }
                            float3 candidate = Closest[n.x + Dims.x * (n.y + Dims.y * n.z)];
                            if (math.any(math.isnan(candidate)))
                            {
                                continue;
                            }
                            float dsq = math.distancesq(p, candidate);
                            if (dsq < bestSq)
                            {
                                bestSq = dsq;
                                best = candidate;
                            }
                        }
                        Closest[index] = best;
                    }
                }
                for (int i = 0; i < Values.Length; i++)
                {
                    int3 q = new int3(i % Dims.x, (i / Dims.x) % Dims.y, i / (Dims.x * Dims.y));
                    float3 p = Origin + (float3)q * CellSize;
                    float3 c = Closest[i];
                    float distance = math.any(math.isnan(c)) ? 4.0f * CellSize : math.distance(p, c);
                    Values[i] = Votes[i] >= 2 ? -distance : distance;
                }
            }
        }
    }

    /// <summary>
    /// Distance fields baked at runtime from MeshCollider meshes, shared by all soft bodies and
    /// kept until the domain reloads.
    /// </summary>
    public static class DistanceFieldCache
    {
        private static readonly Dictionary<(int, int), DistanceField> Fields = new Dictionary<(int, int), DistanceField>();

        public static DistanceField Get(Mesh mesh, int resolution)
        {
            var key = (mesh.GetInstanceID(), resolution);
            if (!Fields.TryGetValue(key, out DistanceField field))
            {
                Fields[key] = field = DistanceField.Bake(mesh.vertices, mesh.triangles, resolution, 0.0f);
            }
            return field;
        }

        public static void Clear() => Fields.Clear();
    }
}
