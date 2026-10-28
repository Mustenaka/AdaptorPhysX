using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    // Burst version of the cut geometry (BuildCutGeometry, EmitCrackFaces, FitPlanes and ComputeVisibleCopies,
    // which remain as the reference the tests compare with). Same decisions, step for step.
    public sealed partial class SurfaceMesh
    {
        /// <summary>Stream records of <see cref="CutGeometryJob"/>.</summary>
        private const int RecordEnd = 0, RecordBlocker = 1, RecordFace = 2;

        /// <summary>A vertex of a cut or crack face.</summary>
        private struct CutFaceVertex
        {
            public int Copy;
            public float3 U;
            public float3 Normal;
            public float2 Uv0;
            public float3 Rest;
        }

        /// <summary>A copy is drawn when its piece has at least <see cref="MinVisiblePieceNodes"/> real nodes.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct VisibleCopiesJob : IJob
        {
            [ReadOnly] public NativeArray<CellCopy> Copies;
            [ReadOnly] public NativeArray<int> CopyNodes;
            [ReadOnly] public NativeArray<int> PieceId;
            public NativeArray<bool> Visible;
            /// <summary>[0]: number of hidden copies.</summary>
            public NativeArray<int> Hidden;

            public void Execute()
            {
                var pieceSize = new NativeArray<int>(PieceId.Length, Allocator.Temp);
                for (int n = 0; n < PieceId.Length; n++)
                {
                    if (PieceId[n] >= 0)
                    {
                        pieceSize[PieceId[n]]++;
                    }
                }
                int hidden = 0;
                for (int q = 0; q < Copies.Length; q++)
                {
                    int mask = Copies[q].Mask;
                    bool visible = false;
                    for (int k = 0; k < 8; k++)
                    {
                        if (((mask >> k) & 1) == 0)
                        {
                            continue;
                        }
                        int piece = PieceId[CopyNodes[q * 8 + k]];
                        visible = piece >= 0 && pieceSize[piece] >= MinVisiblePieceNodes;
                        break;
                    }
                    Visible[q] = visible;
                    hidden += visible ? 0 : 1;
                }
                Hidden[0] = hidden;
                pieceSize.Dispose();
            }
        }

        /// <summary>Cells with severed edges, in cell order.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct CutCellsJob : IJob
        {
            [ReadOnly] public NativeArray<ushort> Severed;
            public NativeList<int> Cells;

            public void Execute()
            {
                for (int c = 0; c < Severed.Length; c++)
                {
                    if (Severed[c] != 0)
                    {
                        Cells.Add(c);
                    }
                }
            }
        }

        /// <summary>
        /// For each cell with severed edges (foreach index = position in Cells): the number of cut planes and the
        /// planes (cells with several copies only), then records: blocker triangles and faces, then the end record.
        /// Cells with several copies get a cut face per copy and virtual corner group; cells with one copy get
        /// two-sided crack faces per group of parallel severed edges.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct CutGeometryJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> Cells;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            [ReadOnly] public NativeArray<CellCopy> Copies;
            [ReadOnly] public NativeArray<ushort> Severed;
            [ReadOnly] public NativeArray<float> EdgeT;
            [ReadOnly] public NativeArray<float3> EdgeNormal;
            [ReadOnly] public NativeArray<int3> CellCoord;
            [ReadOnly] public NativeArray<float> PointSdf;
            [ReadOnly] public NativeArray<int> EdgeCorners;
            public float3 Origin;
            public int3 PointDims;
            public float CellSize;
            public NativeStream.Writer Output;

            public void Execute(int index)
            {
                Output.BeginForEachIndex(index);
                int cell = Cells[index];
                int2 range = CellCopyRange[cell];
                if (range.y < 2)
                {
                    Output.Write(0);
                    EmitCrackFaces(cell);
                }
                else
                {
                    FitPlanes(cell);
                    EmitCutFaces(cell, range);
                }
                Output.Write(RecordEnd);
                Output.EndForEachIndex();
            }

            private void EmitCutFaces(int cell, int2 range)
            {
                int severed = Severed[cell];
                int3 coord = CellCoord[cell];
                float3 cellMin = Origin + (float3)coord * CellSize;
                var polygon = new FixedList4096Bytes<CutVertex>();
                var clipped = new FixedList4096Bytes<CutVertex>();
                var group = new FixedList64Bytes<int>();
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = Copies[q].Mask;
                    // Group virtual corners that touch through cube edges; each group gets its own polygon.
                    group.Clear();
                    for (int k = 0; k < 8; k++)
                    {
                        group.Add(k);
                    }
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        int a = EdgeCorners[2 * e], c = EdgeCorners[2 * e + 1];
                        if (((mask >> a) & 1) == 0 && ((mask >> c) & 1) == 0)
                        {
                            Merge(ref group, a, c);
                        }
                    }
                    for (int g = 0; g < 8; g++)
                    {
                        if (((mask >> g) & 1) != 0 || Root(ref group, g) != g)
                        {
                            continue;
                        }
                        polygon.Clear();
                        float3 faceNormal = float3.zero;
                        for (int e = 0; e < Lattice.EdgeCount; e++)
                        {
                            int a = EdgeCorners[2 * e], c = EdgeCorners[2 * e + 1];
                            bool realA = ((mask >> a) & 1) != 0, realC = ((mask >> c) & 1) != 0;
                            if (realA == realC || (severed & (1 << e)) == 0)
                            {
                                continue;
                            }
                            int virtualCorner = realA ? c : a;
                            if (Root(ref group, virtualCorner) != g)
                            {
                                continue;
                            }
                            float t = EdgeT[cell * Lattice.EdgeCount + e];
                            float3 outward = (float3)Lattice.CornerOffset(virtualCorner) - Lattice.CornerOffset(realA ? a : c);
                            float3 n = EdgeNormal[cell * Lattice.EdgeCount + e];
                            n = math.dot(n, outward) < 0.0f ? -n : n;
                            faceNormal += n;
                            polygon.Add(new CutVertex
                            {
                                U = math.lerp((float3)Lattice.CornerOffset(a), Lattice.CornerOffset(c), t),
                                Normal = n,
                                Sdf = math.lerp(Sdf(coord + Lattice.CornerOffset(a)), Sdf(coord + Lattice.CornerOffset(c)), t),
                            });
                        }
                        if (polygon.Length < 3)
                        {
                            continue;
                        }
                        SortAroundNormal(ref polygon, math.normalizesafe(faceNormal));
                        for (int i = 1; i + 1 < polygon.Length; i++)
                        {
                            Output.Write(RecordBlocker);
                            Output.Write(cellMin + polygon[0].U * CellSize);
                            Output.Write(cellMin + polygon[i].U * CellSize);
                            Output.Write(cellMin + polygon[i + 1].U * CellSize);
                        }
                        ClipInside(ref polygon, ref clipped);
                        if (clipped.Length < 3)
                        {
                            continue;
                        }
                        Output.Write(RecordFace);
                        Output.Write(new int2(clipped.Length, 3 * (clipped.Length - 2)));
                        for (int i = 0; i < clipped.Length; i++)
                        {
                            CutVertex cv = clipped[i];
                            Output.Write(new CutFaceVertex
                            {
                                Copy = q,
                                U = cv.U,
                                Normal = math.normalizesafe(cv.Normal),
                                Uv0 = new float2(-cv.Sdf, 0.0f),
                                Rest = cellMin + cv.U * CellSize,
                            });
                        }
                        for (int i = 1; i + 1 < clipped.Length; i++)
                        {
                            Output.Write(0);
                            Output.Write(i);
                            Output.Write(i + 1);
                        }
                    }
                }
            }

            private void EmitCrackFaces(int cell)
            {
                int severed = Severed[cell];
                int3 coord = CellCoord[cell];
                float3 cellMin = Origin + (float3)coord * CellSize;
                int copy = CellCopyRange[cell].x;
                var polygon = new FixedList4096Bytes<CutVertex>();
                var clipped = new FixedList4096Bytes<CutVertex>();
                int used = 0;
                for (int seed = 0; seed < Lattice.EdgeCount; seed++)
                {
                    if ((severed & (1 << seed)) == 0 || (used & (1 << seed)) != 0)
                    {
                        continue;
                    }
                    float3 reference = EdgeNormal[cell * Lattice.EdgeCount + seed];
                    polygon.Clear();
                    float3 normal = float3.zero;
                    for (int e = seed; e < Lattice.EdgeCount; e++)
                    {
                        if ((severed & (1 << e)) == 0 || (used & (1 << e)) != 0)
                        {
                            continue;
                        }
                        float3 n = EdgeNormal[cell * Lattice.EdgeCount + e];
                        float alignment = math.dot(n, reference);
                        if (math.abs(alignment) < 0.9f)
                        {
                            continue;
                        }
                        used |= 1 << e;
                        n = alignment < 0.0f ? -n : n;
                        normal += n;
                        int a = EdgeCorners[2 * e], c = EdgeCorners[2 * e + 1];
                        float t = EdgeT[cell * Lattice.EdgeCount + e];
                        polygon.Add(new CutVertex
                        {
                            U = math.lerp((float3)Lattice.CornerOffset(a), Lattice.CornerOffset(c), t),
                            Normal = n,
                            Sdf = math.lerp(Sdf(coord + Lattice.CornerOffset(a)), Sdf(coord + Lattice.CornerOffset(c)), t),
                        });
                    }
                    if (polygon.Length < 3)
                    {
                        continue;
                    }
                    normal = math.normalizesafe(normal);
                    SortAroundNormal(ref polygon, normal);
                    ClipInside(ref polygon, ref clipped);
                    if (clipped.Length < 3)
                    {
                        continue;
                    }
                    for (int side = 0; side < 2; side++)
                    {
                        float3 faceNormal = side == 0 ? normal : -normal;
                        Output.Write(RecordFace);
                        Output.Write(new int2(clipped.Length, 3 * (clipped.Length - 2)));
                        for (int i = 0; i < clipped.Length; i++)
                        {
                            CutVertex cv = clipped[i];
                            Output.Write(new CutFaceVertex
                            {
                                Copy = copy,
                                U = cv.U,
                                Normal = faceNormal,
                                Uv0 = new float2(-cv.Sdf, 0.0f),
                                Rest = cellMin + cv.U * CellSize,
                            });
                        }
                        for (int i = 1; i + 1 < clipped.Length; i++)
                        {
                            Output.Write(0);
                            Output.Write(side == 0 ? i : i + 1);
                            Output.Write(side == 0 ? i + 1 : i);
                        }
                    }
                }
            }

            /// <summary>Writes the plane count and the planes (see SurfaceMesh.FitPlanes).</summary>
            private void FitPlanes(int cell)
            {
                int severed = Severed[cell];
                int2 range = CellCopyRange[cell];
                float3 cellMin = Origin + (float3)CellCoord[cell] * CellSize;
                var pairs = new FixedList64Bytes<int>();
                var normals = new FixedList512Bytes<float3>();
                var sums = new FixedList512Bytes<float3>();
                var counts = new FixedList64Bytes<int>();
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0)
                    {
                        continue;
                    }
                    int pair = CopyPair(range, EdgeCorners[2 * e], EdgeCorners[2 * e + 1]);
                    float3 n = EdgeNormal[cell * Lattice.EdgeCount + e];
                    float t = EdgeT[cell * Lattice.EdgeCount + e];
                    float3 hermite = math.lerp((float3)Lattice.CornerOffset(EdgeCorners[2 * e]),
                        Lattice.CornerOffset(EdgeCorners[2 * e + 1]), t);
                    float3 p = cellMin + hermite * CellSize;
                    int cluster = -1;
                    for (int i = 0; i < normals.Length && cluster < 0; i++)
                    {
                        if (pairs[i] == pair && math.abs(math.dot(n, math.normalize(normals[i]))) > 0.9f)
                        {
                            cluster = i;
                        }
                    }
                    if (cluster < 0)
                    {
                        pairs.Add(pair);
                        normals.Add(n);
                        sums.Add(float3.zero);
                        counts.Add(0);
                        cluster = normals.Length - 1;
                    }
                    else
                    {
                        normals[cluster] += math.dot(n, normals[cluster]) < 0.0f ? -n : n;
                    }
                    sums[cluster] += p;
                    counts[cluster]++;
                }
                int planes = 0;
                for (int i = 0; i < normals.Length; i++)
                {
                    planes += FitsPlane(pairs[i], counts[i]) ? 1 : 0;
                }
                Output.Write(planes);
                for (int i = 0; i < normals.Length; i++)
                {
                    if (!FitsPlane(pairs[i], counts[i]))
                    {
                        continue;
                    }
                    float3 n = math.normalize(normals[i]);
                    Output.Write(new float4(n, math.dot(n, sums[i] / counts[i])));
                }
            }

            /// <summary><see cref="PlaneGroup"/> of the copies owning corners a and c.</summary>
            private int CopyPair(int2 range, int a, int c) => PlaneGroup(Ordinal(range, a), Ordinal(range, c));

            /// <summary>Ordinal within the cell of the copy owning a corner.</summary>
            private int Ordinal(int2 range, int corner)
            {
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    if (((Copies[q].Mask >> corner) & 1) != 0)
                    {
                        return q - range.x;
                    }
                }
                return 0;
            }

            private float Sdf(int3 point) => PointSdf[point.x + PointDims.x * (point.y + PointDims.y * point.z)];

            private static int Root(ref FixedList64Bytes<int> parent, int x)
            {
                while (parent[x] != x)
                {
                    x = parent[x];
                }
                return x;
            }

            private static void Merge(ref FixedList64Bytes<int> parent, int a, int b)
            {
                a = Root(ref parent, a);
                b = Root(ref parent, b);
                if (a != b)
                {
                    parent[math.max(a, b)] = math.min(a, b);
                }
            }

            /// <summary>
            /// Orders points counter-clockwise around n as the managed List.Sort does for polygons of up to 16
            /// points (pairs and triples by compare-and-swap, longer ones by insertion sort).
            /// </summary>
            private static void SortAroundNormal(ref FixedList4096Bytes<CutVertex> polygon, float3 n)
            {
                float3 center = float3.zero;
                for (int i = 0; i < polygon.Length; i++)
                {
                    center += polygon[i].U;
                }
                center /= polygon.Length;
                float3 axis = math.abs(n.x) < 0.9f ? new float3(1, 0, 0) : new float3(0, 1, 0);
                float3 t1 = math.normalize(math.cross(axis, n));
                float3 t2 = math.cross(n, t1);
                var angle = new FixedList512Bytes<float>();
                for (int i = 0; i < polygon.Length; i++)
                {
                    float3 d = polygon[i].U - center;
                    angle.Add(math.atan2(math.dot(d, t2), math.dot(d, t1)));
                }
                int count = polygon.Length;
                if (count == 2)
                {
                    SwapIfGreater(ref polygon, ref angle, 0, 1);
                }
                else if (count == 3)
                {
                    SwapIfGreater(ref polygon, ref angle, 0, 1);
                    SwapIfGreater(ref polygon, ref angle, 0, 2);
                    SwapIfGreater(ref polygon, ref angle, 1, 2);
                }
                else
                {
                    for (int i = 0; i < count - 1; i++)
                    {
                        int j = i;
                        CutVertex v = polygon[i + 1];
                        float key = angle[i + 1];
                        while (j >= 0 && key < angle[j])
                        {
                            polygon[j + 1] = polygon[j];
                            angle[j + 1] = angle[j];
                            j--;
                        }
                        polygon[j + 1] = v;
                        angle[j + 1] = key;
                    }
                }
            }

            private static void SwapIfGreater(ref FixedList4096Bytes<CutVertex> polygon, ref FixedList512Bytes<float> angle,
                int i, int j)
            {
                if (angle[i] > angle[j])
                {
                    (polygon[i], polygon[j]) = (polygon[j], polygon[i]);
                    (angle[i], angle[j]) = (angle[j], angle[i]);
                }
            }

            /// <summary>Sutherland-Hodgman clip against the inside (sdf &lt;= 0) of the original surface.</summary>
            private static void ClipInside(ref FixedList4096Bytes<CutVertex> polygon, ref FixedList4096Bytes<CutVertex> output)
            {
                output.Clear();
                for (int i = 0; i < polygon.Length; i++)
                {
                    CutVertex a = polygon[i];
                    CutVertex c = polygon[(i + 1) % polygon.Length];
                    bool insideA = a.Sdf <= 0.0f, insideC = c.Sdf <= 0.0f;
                    if (insideA)
                    {
                        output.Add(a);
                    }
                    if (insideA != insideC)
                    {
                        float s = a.Sdf / (a.Sdf - c.Sdf);
                        output.Add(new CutVertex
                        {
                            U = math.lerp(a.U, c.U, s),
                            Normal = math.lerp(a.Normal, c.Normal, s),
                            Sdf = 0.0f,
                        });
                    }
                }
            }
        }

        /// <summary>
        /// Gathers the cut geometry in cell order: planes and blocker triangles per cell (for the split job), and the
        /// face vertices (new render vertices from <see cref="FirstVertex"/> on) and indices of the cut submesh.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct MergeCutGeometryJob : IJob
        {
            public NativeStream.Reader Input;
            [ReadOnly] public NativeArray<int> Cells;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            public int FirstVertex;
            public NativeArray<int2> CellPlanes;
            public NativeList<float4> Planes;
            public NativeArray<int2> CellBlockers;
            public NativeList<float3> Blockers;
            public NativeList<int> VertexCopies;
            public NativeList<float3> VertexU;
            public NativeList<float3> VertexNormals;
            public NativeList<float2> VertexUv0;
            public NativeList<float3> VertexUv1;
            public NativeList<int> Indices;

            public void Execute()
            {
                for (int i = 0; i < Cells.Length; i++)
                {
                    int cell = Cells[i];
                    bool cut = CellCopyRange[cell].y >= 2;
                    Input.BeginForEachIndex(i);
                    int planes = Input.Read<int>();
                    if (cut)
                    {
                        CellPlanes[cell] = new int2(Planes.Length, planes);
                    }
                    for (int p = 0; p < planes; p++)
                    {
                        Planes.Add(Input.Read<float4>());
                    }
                    int blockerStart = Blockers.Length;
                    for (int record = Input.Read<int>(); record != RecordEnd; record = Input.Read<int>())
                    {
                        if (record == RecordBlocker)
                        {
                            Blockers.Add(Input.Read<float3>());
                            Blockers.Add(Input.Read<float3>());
                            Blockers.Add(Input.Read<float3>());
                            continue;
                        }
                        int2 counts = Input.Read<int2>();
                        int first = FirstVertex + VertexCopies.Length;
                        for (int k = 0; k < counts.x; k++)
                        {
                            CutFaceVertex v = Input.Read<CutFaceVertex>();
                            VertexCopies.Add(v.Copy);
                            VertexU.Add(v.U);
                            VertexNormals.Add(v.Normal);
                            VertexUv0.Add(v.Uv0);
                            VertexUv1.Add(v.Rest);
                        }
                        for (int k = 0; k < counts.y; k++)
                        {
                            Indices.Add(first + Input.Read<int>());
                        }
                    }
                    if (cut)
                    {
                        CellBlockers[cell] = new int2(blockerStart, Blockers.Length - blockerStart);
                    }
                    Input.EndForEachIndex();
                }
            }
        }
    }
}
