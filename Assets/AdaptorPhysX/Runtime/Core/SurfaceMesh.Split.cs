using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    // Burst version of the skin triangle split (SplitSkinTriangle and the methods it calls, which remain for
    // triangles whose pieces exceed the fixed lists here). Same decisions, step for step.
    public sealed partial class SurfaceMesh
    {
        /// <summary>A vertex of a split skin triangle: <see cref="SplitTriangle"/> in native form.</summary>
        private struct SplitVertex
        {
            public int Cell;
            public int Ordinal;
            public int Source;
            public float3 U;
            public float3 Normal;
            public float3 Rest;
            public float2 Uv;
        }

        /// <summary><see cref="SkinCorner"/> in native form.</summary>
        private struct SplitCorner
        {
            public int Vertex;
            public int EdgeLo, EdgeHi;
            public float EdgeT;
            public float3 Rest;
            public float3 Normal;
            public float2 Uv;
        }

        /// <summary>A piece: a range of corners in a corner pool and its side of every plane so far (bit i, plane i).</summary>
        private struct SplitPiece
        {
            public int Start;
            public int Count;
            public int Signs;
        }

        /// <summary>
        /// Splits each skin triangle touching a cut cell by the cut planes of its vertices' cells and resolves the
        /// copy of every piece vertex. Writes per triangle (foreach index = position in DynamicTriangles):
        /// int2 (vertex count, index count), the vertices, the local indices; vertex count -1 when a fixed list
        /// overflowed (the managed split handles that triangle).
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct SplitTrianglesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> DynamicTriangles;
            [ReadOnly] public NativeArray<int> Triangles;
            [ReadOnly] public NativeArray<float3> RestPositions;
            [ReadOnly] public NativeArray<float3> RestNormals;
            [ReadOnly] public NativeArray<float2> Uvs;
            [ReadOnly] public NativeArray<int> EmbedCell;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            [ReadOnly] public NativeArray<CellCopy> Copies;
            [ReadOnly] public NativeArray<ushort> Severed;
            [ReadOnly] public NativeArray<float> EdgeT;
            [ReadOnly] public NativeArray<float3> EdgeNormal;
            [ReadOnly] public NativeArray<int3> CellCoord;
            [ReadOnly] public NativeArray<int> GridToCell;
            [ReadOnly] public NativeArray<int> EdgeCorners;
            /// <summary>(start, count) of each cell's cut planes in <see cref="Planes"/>.</summary>
            [ReadOnly] public NativeArray<int2> CellPlanes;
            [ReadOnly] public NativeArray<float4> Planes;
            /// <summary>(start, count) of each cell's blocker triangle vertices in <see cref="Blockers"/>.</summary>
            [ReadOnly] public NativeArray<int2> CellBlockers;
            [ReadOnly] public NativeArray<float3> Blockers;
            public float3 Origin;
            public int3 Dims;
            public float CellSize;
            public NativeStream.Writer Output;

            public void Execute(int index)
            {
                Output.BeginForEachIndex(index);
                int t = DynamicTriangles[index];
                var vertices = new FixedList4096Bytes<SplitVertex>();
                var keys = new FixedList4096Bytes<int4>();
                var indices = new FixedList512Bytes<int>();
                if (Split(t, ref vertices, ref keys, ref indices))
                {
                    Output.Write(new int2(vertices.Length, indices.Length));
                    for (int i = 0; i < vertices.Length; i++)
                    {
                        Output.Write(vertices[i]);
                    }
                    for (int i = 0; i < indices.Length; i++)
                    {
                        Output.Write(indices[i]);
                    }
                }
                else
                {
                    Output.Write(new int2(-1, 0));
                }
                Output.EndForEachIndex();
            }

            private bool Split(int t, ref FixedList4096Bytes<SplitVertex> vertices, ref FixedList4096Bytes<int4> keys,
                ref FixedList512Bytes<int> indices)
            {
                var planes = new FixedList512Bytes<float4>();
                if (!CollectPlanes(t, ref planes))
                {
                    return false;
                }
                var pool = new FixedList4096Bytes<SplitCorner>();
                var pieces = new FixedList512Bytes<SplitPiece>();
                for (int i = 0; i < 3; i++)
                {
                    int v = Triangles[t + i];
                    pool.Add(new SplitCorner
                    {
                        Vertex = v,
                        EdgeLo = -1,
                        EdgeHi = -1,
                        Rest = RestPositions[v],
                        Normal = RestNormals[v],
                        Uv = Uvs[v],
                    });
                }
                pieces.Add(new SplitPiece { Start = 0, Count = 3 });
                var nextPool = new FixedList4096Bytes<SplitCorner>();
                var nextPieces = new FixedList512Bytes<SplitPiece>();
                for (int k = 0; k < planes.Length; k++)
                {
                    nextPool.Clear();
                    nextPieces.Clear();
                    for (int p = 0; p < pieces.Length; p++)
                    {
                        if (!SplitPieceByPlane(ref pool, pieces[p], planes[k], k, ref nextPool, ref nextPieces))
                        {
                            return false;
                        }
                    }
                    pool = nextPool;
                    pieces = nextPieces;
                }
                for (int p = 0; p < pieces.Length; p++)
                {
                    if (!EmitSkin(ref pool, pieces[p], ref planes, ref vertices, ref keys, ref indices))
                    {
                        return false;
                    }
                }
                return true;
            }

            private bool CollectPlanes(int t, ref FixedList512Bytes<float4> planes)
            {
                for (int i = 0; i < 3; i++)
                {
                    int2 range = CellPlanes[EmbedCell[Triangles[t + i]]];
                    for (int j = range.x; j < range.x + range.y; j++)
                    {
                        float4 plane = Planes[j];
                        bool duplicate = false;
                        for (int o = 0; o < planes.Length; o++)
                        {
                            duplicate |= SameCut(plane, planes[o], t);
                        }
                        if (duplicate)
                        {
                            continue;
                        }
                        if (planes.Length == planes.Capacity || planes.Length == 30)
                        {
                            return false;
                        }
                        planes.Add(plane);
                    }
                }
                return true;
            }

            private bool SameCut(float4 a, float4 b, int t)
            {
                float alignment = math.dot(a.xyz, b.xyz);
                if (math.abs(alignment) < 0.98f)
                {
                    return false;
                }
                b = alignment < 0.0f ? -b : b;
                for (int i = 0; i < 3; i++)
                {
                    float3 v = RestPositions[Triangles[t + i]];
                    if (math.abs(math.dot(a.xyz - b.xyz, v) - (a.w - b.w)) > 0.25f * CellSize)
                    {
                        return false;
                    }
                }
                return true;
            }

            private static bool SplitPieceByPlane(ref FixedList4096Bytes<SplitCorner> pool, SplitPiece piece, float4 plane,
                int k, ref FixedList4096Bytes<SplitCorner> output, ref FixedList512Bytes<SplitPiece> pieces)
            {
                var positive = new FixedList4096Bytes<SplitCorner>();
                var negative = new FixedList4096Bytes<SplitCorner>();
                for (int i = 0; i < piece.Count; i++)
                {
                    SplitCorner a = pool[piece.Start + i];
                    SplitCorner c = pool[piece.Start + (i + 1) % piece.Count];
                    float da = math.dot(plane.xyz, a.Rest) - plane.w;
                    float dc = math.dot(plane.xyz, c.Rest) - plane.w;
                    if (positive.Length + 2 > positive.Capacity || negative.Length + 2 > negative.Capacity)
                    {
                        return false;
                    }
                    if (da >= 0.0f)
                    {
                        positive.Add(a);
                    }
                    else
                    {
                        negative.Add(a);
                    }
                    if ((da >= 0.0f) == (dc >= 0.0f))
                    {
                        continue;
                    }
                    float s = da / (da - dc);
                    var cross = new SplitCorner
                    {
                        Vertex = -1,
                        EdgeLo = -1,
                        EdgeHi = -1,
                        Rest = math.lerp(a.Rest, c.Rest, s),
                        Normal = math.normalizesafe(math.lerp(a.Normal, c.Normal, s)),
                        Uv = math.lerp(a.Uv, c.Uv, math.saturate(s)),
                    };
                    if (a.Vertex >= 0 && c.Vertex >= 0)
                    {
                        bool forward = a.Vertex < c.Vertex;
                        cross.EdgeLo = forward ? a.Vertex : c.Vertex;
                        cross.EdgeHi = forward ? c.Vertex : a.Vertex;
                        cross.EdgeT = forward ? s : 1.0f - s;
                    }
                    positive.Add(cross);
                    negative.Add(cross);
                }
                if (positive.Length >= 3 && negative.Length >= 3)
                {
                    return AddPiece(ref positive, piece.Signs | (1 << k), ref output, ref pieces) &&
                           AddPiece(ref negative, piece.Signs, ref output, ref pieces);
                }
                var original = new FixedList4096Bytes<SplitCorner>();
                for (int i = 0; i < piece.Count; i++)
                {
                    if (original.Length == original.Capacity)
                    {
                        return false;
                    }
                    original.Add(pool[piece.Start + i]);
                }
                return AddPiece(ref original, positive.Length >= 3 ? piece.Signs | (1 << k) : piece.Signs, ref output, ref pieces);
            }

            private static bool AddPiece(ref FixedList4096Bytes<SplitCorner> corners, int signs,
                ref FixedList4096Bytes<SplitCorner> output, ref FixedList512Bytes<SplitPiece> pieces)
            {
                if (output.Length + corners.Length > output.Capacity || pieces.Length == pieces.Capacity)
                {
                    return false;
                }
                pieces.Add(new SplitPiece { Start = output.Length, Count = corners.Length, Signs = signs });
                for (int i = 0; i < corners.Length; i++)
                {
                    output.Add(corners[i]);
                }
                return true;
            }

            private bool EmitSkin(ref FixedList4096Bytes<SplitCorner> pool, SplitPiece piece, ref FixedList512Bytes<float4> planes,
                ref FixedList4096Bytes<SplitVertex> vertices, ref FixedList4096Bytes<int4> keys, ref FixedList512Bytes<int> indices)
            {
                float3 side = float3.zero;
                for (int i = 0; i < piece.Count; i++)
                {
                    side += pool[piece.Start + i].Rest;
                }
                side /= piece.Count;

                int first = -1, previous = -1;
                for (int i = 0; i < piece.Count; i++)
                {
                    int index = SkinVertex(pool[piece.Start + i], piece.Signs, ref planes, side, ref vertices, ref keys);
                    if (index < 0)
                    {
                        return false;
                    }
                    if (i == 0)
                    {
                        first = index;
                    }
                    else if (i >= 2)
                    {
                        if (indices.Length + 3 > indices.Capacity)
                        {
                            return false;
                        }
                        indices.Add(first);
                        indices.Add(previous);
                        indices.Add(index);
                    }
                    previous = index;
                }
                return true;
            }

            /// <summary>Local vertex of the split triangle, or -1 when the lists are full.</summary>
            private int SkinVertex(SplitCorner corner, int signs, ref FixedList512Bytes<float4> planes, float3 side,
                ref FixedList4096Bytes<SplitVertex> vertices, ref FixedList4096Bytes<int4> keys)
            {
                int cell = corner.Vertex >= 0 ? EmbedCell[corner.Vertex] : FindCell(corner.Rest);
                int copy = planes.Length > 0
                    ? ResolveBySides(cell, corner.Rest, signs, ref planes, side)
                    : ResolveCopy(cell, corner.Rest, side);
                int ordinal = copy - CellCopyRange[cell].x;
                int4 key = corner.Vertex >= 0
                    ? new int4(corner.Vertex, -1, ordinal, 0)
                    : new int4(corner.EdgeLo, corner.EdgeHi, ordinal, (int)math.round(corner.EdgeT * 65536.0f));
                bool shareable = corner.Vertex >= 0 || corner.EdgeLo >= 0;
                if (shareable)
                {
                    for (int i = 0; i < keys.Length; i++)
                    {
                        if (math.all(keys[i] == key) && vertices[i].Cell == cell)
                        {
                            return i;
                        }
                    }
                }
                if (vertices.Length == vertices.Capacity || keys.Length == keys.Capacity)
                {
                    return -1;
                }
                keys.Add(shareable ? key : new int4(-2, -2, -2, keys.Length));
                vertices.Add(new SplitVertex
                {
                    Cell = cell,
                    Ordinal = ordinal,
                    Source = corner.Vertex >= 0 && ordinal == 0 ? corner.Vertex : -1,
                    U = (corner.Rest - CellMin(cell)) / CellSize,
                    Normal = corner.Normal,
                    Uv = corner.Uv,
                    Rest = corner.Rest,
                });
                return vertices.Length - 1;
            }

            private int ResolveBySides(int cell, float3 query, int signs, ref FixedList512Bytes<float4> planes, float3 side)
            {
                int2 range = CellCopyRange[cell];
                if (range.y == 1)
                {
                    return range.x;
                }
                float3 cellMin = CellMin(cell);
                int best = -1;
                float bestDistance = float.MaxValue;
                for (int k = 0; k < 8; k++)
                {
                    float3 p = cellMin + (float3)Lattice.CornerOffset(k) * CellSize;
                    int copy = CopyOwning(range, k);
                    bool ok = true;
                    for (int i = 0; i < planes.Length && ok; i++)
                    {
                        ok = CornerSide(cell, copy, p, planes[i]) == (((signs >> i) & 1) != 0);
                    }
                    float distance = math.distancesq(p, query);
                    if (ok && distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = k;
                    }
                }
                return best < 0 ? ResolveCopy(cell, query, side) : CopyOwning(range, best);
            }

            private bool CornerSide(int cell, int copy, float3 p, float4 plane)
            {
                float3 cellMin = CellMin(cell);
                int mask = Copies[copy].Mask;
                int severed = Severed[cell];
                float along = 0.0f;
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    int a = EdgeCorners[2 * e], c = EdgeCorners[2 * e + 1];
                    bool realA = ((mask >> a) & 1) != 0, realC = ((mask >> c) & 1) != 0;
                    if (realA == realC || (severed & (1 << e)) == 0 ||
                        math.abs(math.dot(EdgeNormal[cell * Lattice.EdgeCount + e], plane.xyz)) < 0.9f)
                    {
                        continue;
                    }
                    float3 hermite = cellMin + HermitePoint(cell, e) * CellSize;
                    if (math.abs(math.dot(plane.xyz, hermite) - plane.w) > 0.5f * CellSize)
                    {
                        continue;
                    }
                    float3 inward = Lattice.CornerOffset(realA ? a : c) - Lattice.CornerOffset(realA ? c : a);
                    along += math.dot(inward, plane.xyz);
                }
                if (math.abs(along) >= 0.5f)
                {
                    return along > 0.0f;
                }
                return math.dot(plane.xyz, p) - plane.w >= 0.0f;
            }

            private int ResolveCopy(int cell, float3 query, float3 side)
            {
                int2 range = CellCopyRange[cell];
                if (range.y == 1)
                {
                    return range.x;
                }
                int2 blockers = CellBlockers[cell];
                int sideCell = FindCell(side);
                int2 sideBlockers = sideCell != cell ? CellBlockers[sideCell] : int2.zero;

                var order = new FixedList64Bytes<int>();
                var distance = new FixedList64Bytes<float>();
                float3 cellMin = CellMin(cell);
                for (int k = 0; k < 8; k++)
                {
                    order.Add(k);
                    distance.Add(math.distancesq(cellMin + (float3)Lattice.CornerOffset(k) * CellSize, query));
                }
                for (int i = 1; i < 8; i++)
                {
                    for (int j = i; j > 0 && distance[order[j]] < distance[order[j - 1]]; j--)
                    {
                        (order[j], order[j - 1]) = (order[j - 1], order[j]);
                    }
                }
                int corner = order[0];
                for (int i = 0; i < 8; i++)
                {
                    float3 target = cellMin + (float3)Lattice.CornerOffset(order[i]) * CellSize;
                    if (!Blocked(blockers, side, target) && !Blocked(sideBlockers, side, target))
                    {
                        corner = order[i];
                        break;
                    }
                }
                return CopyOwning(range, corner);
            }

            private bool Blocked(int2 range, float3 from, float3 to)
            {
                for (int i = range.x; i < range.x + range.y; i += 3)
                {
                    if (Geometry.SegmentTriangle(from, to, Blockers[i], Blockers[i + 1], Blockers[i + 2]) >= 0.0f)
                    {
                        return true;
                    }
                }
                return false;
            }

            private int CopyOwning(int2 range, int corner)
            {
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    if (((Copies[q].Mask >> corner) & 1) != 0)
                    {
                        return q;
                    }
                }
                return range.x;
            }

            private float3 HermitePoint(int cell, int edge)
            {
                float t = EdgeT[cell * Lattice.EdgeCount + edge];
                return math.lerp((float3)Lattice.CornerOffset(EdgeCorners[2 * edge]),
                    Lattice.CornerOffset(EdgeCorners[2 * edge + 1]), t);
            }

            private float3 CellMin(int cell) => Origin + (float3)CellCoord[cell] * CellSize;

            private int FindCell(float3 p)
            {
                int3 c = (int3)math.floor((p - Origin) / CellSize);
                int found = -1;
                float best = float.MaxValue;
                for (int ring = 0; ring <= 2 && found < 0; ring++)
                {
                    for (int z = -ring; z <= ring; z++)
                    for (int y = -ring; y <= ring; y++)
                    for (int x = -ring; x <= ring; x++)
                    {
                        int3 n = c + new int3(x, y, z);
                        if (math.any(n < 0) || math.any(n >= Dims))
                        {
                            continue;
                        }
                        int index = GridToCell[n.x + Dims.x * (n.y + Dims.y * n.z)];
                        if (index < 0)
                        {
                            continue;
                        }
                        float d = math.distancesq(Origin + ((float3)n + 0.5f) * CellSize, p);
                        if (d < best)
                        {
                            best = d;
                            found = index;
                        }
                    }
                }
                return found >= 0 ? found : 0;
            }
        }

        /// <summary>
        /// Appends the split triangles in order: vertices not taken from the source mesh become new render vertices
        /// (from <see cref="FirstVertex"/> on), local indices are mapped to render vertices. Triangles the split job
        /// could not handle are listed in <see cref="Overflow"/> (positions in the dynamic triangle list).
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct MergeSplitsJob : IJob
        {
            public NativeStream.Reader Input;
            public int Count;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            public int FirstVertex;
            public NativeList<int> VertexCopies;
            public NativeList<float3> VertexU;
            public NativeList<float3> VertexNormals;
            public NativeList<float2> VertexUv0;
            public NativeList<float3> VertexUv1;
            public NativeList<int> Indices;
            public NativeList<int> Overflow;

            public void Execute()
            {
                var map = new FixedList512Bytes<int>();
                for (int i = 0; i < Count; i++)
                {
                    Input.BeginForEachIndex(i);
                    int2 counts = Input.Read<int2>();
                    if (counts.x < 0)
                    {
                        Overflow.Add(i);
                        Input.EndForEachIndex();
                        continue;
                    }
                    map.Clear();
                    for (int k = 0; k < counts.x; k++)
                    {
                        SplitVertex v = Input.Read<SplitVertex>();
                        if (v.Source >= 0)
                        {
                            map.Add(v.Source);
                            continue;
                        }
                        map.Add(FirstVertex + VertexCopies.Length);
                        VertexCopies.Add(CellCopyRange[v.Cell].x + v.Ordinal);
                        VertexU.Add(v.U);
                        VertexNormals.Add(v.Normal);
                        VertexUv0.Add(v.Uv);
                        VertexUv1.Add(v.Rest);
                    }
                    for (int k = 0; k < counts.y; k++)
                    {
                        Indices.Add(map[Input.Read<int>()]);
                    }
                    Input.EndForEachIndex();
                }
            }
        }
    }
}
