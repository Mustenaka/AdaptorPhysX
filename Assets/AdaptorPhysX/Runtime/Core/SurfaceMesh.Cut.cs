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

        /// <summary>
        /// The cuts through a cell: its severed edges in at most two groups, with the sums of their Hermite points and of their
        /// normals turned to the group's seed. The seeds are the normal of the first severed edge and, when some normal is less
        /// aligned with it than <see cref="CreaseAlignment"/>, the least aligned one; each edge joins the seed it is more aligned
        /// with. A blade that turns sharply inside a cell gives normals in between: they join one of the two cuts.
        /// </summary>
        private struct CellCuts
        {
            public int Count;
            public float3 Seed0, Seed1;
            public float3 PointSum0, PointSum1;
            public float3 NormalSum0, NormalSum1;
            public int Edges0, Edges1;
        }

        /// <summary>Least |cos| between the normals of the severed edges of one cut (<see cref="CellCuts"/>).</summary>
        private const float CreaseAlignment = 0.9f;

        /// <summary>The cuts of a cell from the Hermite points and normals of its severed edges, in edge order.</summary>
        private static CellCuts CutsOf(in FixedList512Bytes<float3> points, in FixedList512Bytes<float3> normals)
        {
            var cuts = new CellCuts();
            if (normals.Length == 0)
            {
                return cuts;
            }
            cuts.Count = 1;
            cuts.Seed0 = normals[0];
            float least = CreaseAlignment;
            for (int i = 1; i < normals.Length; i++)
            {
                float alignment = math.abs(math.dot(normals[i], cuts.Seed0));
                if (alignment < least)
                {
                    least = alignment;
                    cuts.Seed1 = normals[i];
                    cuts.Count = 2;
                }
            }
            for (int i = 0; i < normals.Length; i++)
            {
                float3 n = normals[i];
                if (cuts.Count == 1 || CutOf(cuts, n) == 0)
                {
                    cuts.PointSum0 += points[i];
                    cuts.NormalSum0 += math.dot(n, cuts.Seed0) < 0.0f ? -n : n;
                    cuts.Edges0++;
                }
                else
                {
                    cuts.PointSum1 += points[i];
                    cuts.NormalSum1 += math.dot(n, cuts.Seed1) < 0.0f ? -n : n;
                    cuts.Edges1++;
                }
            }
            return cuts;
        }

        /// <summary>The cut of a cell with two (<see cref="CellCuts"/>) a severed edge belongs to: 0 or 1.</summary>
        private static int CutOf(in CellCuts cuts, float3 normal) =>
            math.abs(math.dot(normal, cuts.Seed0)) >= math.abs(math.dot(normal, cuts.Seed1)) ? 0 : 1;

        /// <summary>
        /// Where the two cuts of a cell meet inside it: the segment from <paramref name="a"/> to <paramref name="b"/> (cell
        /// coordinates) between the two faces of the cell on which the lines of both cuts cross (<see cref="CrossOnFace"/>).
        /// The point depends on the face only, so the cells on either side of it put the end of their crease at the same place.
        /// False unless exactly two faces have such a point. <paramref name="points"/> and <paramref name="normals"/> hold the
        /// Hermite point and normal of every edge of the cell.
        /// </summary>
        private static bool Crease(int severed, in FixedList512Bytes<float3> points, in FixedList512Bytes<float3> normals,
            out float3 a, out float3 b)
        {
            a = b = float3.zero;
            int found = 0;
            var face = new FixedList32Bytes<int>();
            for (int axis = 0; axis < 3; axis++)
            {
                for (int side = 0; side < 2; side++)
                {
                    face.Clear();
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        if (e / 4 != axis && Lattice.CornerOffset(Lattice.EdgeCorners[2 * e])[axis] == side && (severed & (1 << e)) != 0)
                        {
                            face.Add(e);
                        }
                    }
                    if (!CrossOnFace(axis, points, normals, face, out float3 point))
                    {
                        continue;
                    }
                    if (found == 0)
                    {
                        a = point;
                    }
                    else if (found == 1)
                    {
                        b = point;
                    }
                    found++;
                }
            }
            return found == 2;
        }

        /// <summary>
        /// The point where the lines of two cuts cross on a face of a cell (normal along <paramref name="axis"/>), from its
        /// severed edges: of four, the pairing with the more aligned normals gives two cuts of two points; of three, the most
        /// aligned pair is one cut and the third edge the other; two edges whose normals are less aligned than
        /// <see cref="CreaseAlignment"/> are one cut each. A cut of two points runs through them; a cut of one point, where
        /// a cut ends inside the cell, runs through it across its normal. False otherwise, for parallel lines and for a
        /// crossing outside the face.
        /// </summary>
        private static bool CrossOnFace(int axis, in FixedList512Bytes<float3> points, in FixedList512Bytes<float3> normals,
            in FixedList32Bytes<int> face, out float3 point)
        {
            point = float3.zero;
            int p0, p1, q0, q1;
            if (face.Length == 4)
            {
                int bestPair = 0;
                float best = -1.0f;
                for (int pair = 0; pair < 3; pair++)
                {
                    int j0 = pair == 0 ? 2 : 1, j1 = pair == 2 ? 2 : 3;
                    float alignment = math.min(math.abs(math.dot(normals[face[0]], normals[face[pair + 1]])),
                        math.abs(math.dot(normals[face[j0]], normals[face[j1]])));
                    if (alignment > best)
                    {
                        best = alignment;
                        bestPair = pair;
                    }
                }
                p0 = face[0];
                p1 = face[bestPair + 1];
                q0 = face[bestPair == 0 ? 2 : 1];
                q1 = face[bestPair == 2 ? 2 : 3];
            }
            else if (face.Length == 3)
            {
                int alone = 0;
                float best = -1.0f;
                for (int i = 0; i < 3; i++)
                {
                    float alignment = math.abs(math.dot(normals[face[(i + 1) % 3]], normals[face[(i + 2) % 3]]));
                    if (alignment > best)
                    {
                        best = alignment;
                        alone = i;
                    }
                }
                p0 = face[(alone + 1) % 3];
                p1 = face[(alone + 2) % 3];
                q0 = q1 = face[alone];
            }
            else if (face.Length == 2 && math.abs(math.dot(normals[face[0]], normals[face[1]])) < CreaseAlignment)
            {
                p0 = p1 = face[0];
                q0 = q1 = face[1];
            }
            else
            {
                return false;
            }
            // In the plane of the face: the coordinates along the two other axes.
            int u = (axis + 1) % 3, v = (axis + 2) % 3;
            if (!LineOnFace(u, v, points, normals, p0, p1, out float2 origin, out float2 r) ||
                !LineOnFace(u, v, points, normals, q0, q1, out float2 other, out float2 s))
            {
                return false;
            }
            float denominator = r.x * s.y - r.y * s.x;
            if (math.abs(denominator) < 1e-6f)
            {
                return false;
            }
            float2 w = other - origin;
            float t = (w.x * s.y - w.y * s.x) / denominator;
            float2 cross = origin + t * r;
            if (math.any(cross < 0.0f) || math.any(cross > 1.0f))
            {
                return false;
            }
            point = points[face[0]];
            point[u] = cross.x;
            point[v] = cross.y;
            return true;
        }

        /// <summary>
        /// The line of a cut on a face, in the face's coordinates <paramref name="u"/>, <paramref name="v"/>: through the
        /// Hermite points of edges <paramref name="e0"/> and <paramref name="e1"/>, or through the point of one edge across the
        /// face's part of its normal.
        /// </summary>
        private static bool LineOnFace(int u, int v, in FixedList512Bytes<float3> points, in FixedList512Bytes<float3> normals,
            int e0, int e1, out float2 origin, out float2 direction)
        {
            origin = new float2(points[e0][u], points[e0][v]);
            direction = e0 != e1 ? new float2(points[e1][u], points[e1][v]) - origin
                : new float2(-normals[e0][v], normals[e0][u]);
            return math.lengthsq(direction) > 1e-8f;
        }

        /// <summary>
        /// Splits the cut polygon of a copy bounded by both cuts of its cell (a crossing, or a turn sharper than the alignment)
        /// into one polygon per cut, each closed along the crease where the two meet (<see cref="Crease"/>). One polygon through
        /// the Hermite points of both cuts cut across the corner between them: the copies around a crossing left a hole at the
        /// floor of the incisions, where their faces did not meet. False when the polygon has points of one cut only.
        /// </summary>
        private static bool SplitAtCrease(in FixedList4096Bytes<CutVertex> polygon, in CellCuts cuts, float3 a, float3 b, float sdfA,
            float sdfB, ref FixedList4096Bytes<CutVertex> part0, ref FixedList4096Bytes<CutVertex> part1, out float3 normal0,
            out float3 normal1)
        {
            part0.Clear();
            part1.Clear();
            normal0 = normal1 = float3.zero;
            for (int i = 0; i < polygon.Length; i++)
            {
                CutVertex v = polygon[i];
                if (CutOf(cuts, v.Normal) == 0)
                {
                    part0.Add(v);
                    normal0 += v.Normal;
                }
                else
                {
                    part1.Add(v);
                    normal1 += v.Normal;
                }
            }
            if (part0.Length == 0 || part1.Length == 0)
            {
                return false;
            }
            normal0 = math.normalizesafe(normal0);
            normal1 = math.normalizesafe(normal1);
            int copy0 = part0[0].Copy, copy1 = part1[0].Copy;
            part0.Add(new CutVertex { U = a, Normal = normal0, Sdf = sdfA, Copy = copy0 });
            part0.Add(new CutVertex { U = b, Normal = normal0, Sdf = sdfB, Copy = copy0 });
            part1.Add(new CutVertex { U = a, Normal = normal1, Sdf = sdfA, Copy = copy1 });
            part1.Add(new CutVertex { U = b, Normal = normal1, Sdf = sdfB, Copy = copy1 });
            return true;
        }

        /// <summary>Trilinear interpolation at <paramref name="u"/> (cell coordinates) of the SDF at a cell's corners.</summary>
        private static float CornerLerp(float3 u, float sdf0, float sdf1, float sdf2, float sdf3, float sdf4, float sdf5, float sdf6, float sdf7)
        {
            float x0 = math.lerp(math.lerp(sdf0, sdf1, u.x), math.lerp(sdf2, sdf3, u.x), u.y);
            float x1 = math.lerp(math.lerp(sdf4, sdf5, u.x), math.lerp(sdf6, sdf7, u.x), u.y);
            return math.lerp(x0, x1, u.z);
        }

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
            [ReadOnly] public NativeArray<int> CopyNodes;
            [ReadOnly] public NativeArray<int> PieceId;
            [ReadOnly] public NativeArray<ushort> Severed;
            [ReadOnly] public NativeArray<float> EdgeT;
            [ReadOnly] public NativeArray<float3> EdgeNormal;
            [ReadOnly] public NativeArray<int3> CellCoord;
            [ReadOnly] public NativeArray<int> GridToCell;
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
                    var region = new FixedList64Bytes<int>();
                    CutRegions(cell, range, ref region);
                    FitPlanes(cell, ref region);
                    EmitCutFaces(cell, range, ref region);
                }
                Output.Write(RecordEnd);
                Output.EndForEachIndex();
            }

            private void EmitCutFaces(int cell, int2 range, ref FixedList64Bytes<int> region)
            {
                int severed = Severed[cell];
                int3 coord = CellCoord[cell];
                float3 cellMin = Origin + (float3)coord * CellSize;
                var polygon = new FixedList4096Bytes<CutVertex>();
                var clipped = new FixedList4096Bytes<CutVertex>();
                var part0 = new FixedList4096Bytes<CutVertex>();
                var part1 = new FixedList4096Bytes<CutVertex>();
                var group = new FixedList64Bytes<int>();
                var edgePoints = new FixedList512Bytes<float3>();
                var edgeNormals = new FixedList512Bytes<float3>();
                var cutPoints = new FixedList512Bytes<float3>();
                var cutNormals = new FixedList512Bytes<float3>();
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    edgePoints.Add(math.lerp((float3)Lattice.CornerOffset(EdgeCorners[2 * e]), Lattice.CornerOffset(EdgeCorners[2 * e + 1]),
                        EdgeT[cell * Lattice.EdgeCount + e]));
                    edgeNormals.Add(EdgeNormal[cell * Lattice.EdgeCount + e]);
                    if ((severed & (1 << e)) != 0)
                    {
                        cutPoints.Add(edgePoints[e]);
                        cutNormals.Add(edgeNormals[e]);
                    }
                }
                CellCuts cuts = CutsOf(cutPoints, cutNormals);
                float3 creaseA = float3.zero, creaseB = float3.zero;
                bool crease = cuts.Count == 2 && Crease(severed, edgePoints, edgeNormals, out creaseA, out creaseB);
                float sdfA = crease ? CellSdf(coord, creaseA) : 0.0f, sdfB = crease ? CellSdf(coord, creaseB) : 0.0f;
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    if (Root(ref region, q - range.x) != q - range.x)
                    {
                        continue;
                    }
                    int mask = RegionMask(range, ref region, q - range.x);
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
                                Copy = range.x + Ordinal(range, realA ? a : c),
                            });
                        }
                        if (crease && SplitAtCrease(polygon, cuts, creaseA, creaseB, sdfA, sdfB, ref part0, ref part1,
                                out float3 normal0, out float3 normal1))
                        {
                            EmitFace(ref part0, normal0, cellMin, ref clipped);
                            EmitFace(ref part1, normal1, cellMin, ref clipped);
                            continue;
                        }
                        if (polygon.Length < 3)
                        {
                            continue;
                        }
                        EmitFace(ref polygon, math.normalizesafe(faceNormal), cellMin, ref clipped);
                    }
                }
            }

            /// <summary>A cut face: the polygon sorted around its normal as blocker triangles, then clipped and written as a face.</summary>
            private void EmitFace(ref FixedList4096Bytes<CutVertex> polygon, float3 normal, float3 cellMin,
                ref FixedList4096Bytes<CutVertex> clipped)
            {
                SortAroundNormal(ref polygon, normal);
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
                    return;
                }
                Output.Write(RecordFace);
                Output.Write(new int2(clipped.Length, 3 * (clipped.Length - 2)));
                for (int i = 0; i < clipped.Length; i++)
                {
                    CutVertex cv = clipped[i];
                    Output.Write(new CutFaceVertex
                    {
                        Copy = cv.Copy,
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

            /// <summary>The SDF at <paramref name="u"/> (cell coordinates) in the cell at <paramref name="coord"/>.</summary>
            private float CellSdf(int3 coord, float3 u) => CornerLerp(u, Sdf(coord), Sdf(coord + new int3(1, 0, 0)),
                Sdf(coord + new int3(0, 1, 0)), Sdf(coord + new int3(1, 1, 0)), Sdf(coord + new int3(0, 0, 1)),
                Sdf(coord + new int3(1, 0, 1)), Sdf(coord + new int3(0, 1, 1)), Sdf(coord + new int3(1, 1, 1)));

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
                            Copy = copy,
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
            private void FitPlanes(int cell, ref FixedList64Bytes<int> region)
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
                    int qa = Ordinal(range, EdgeCorners[2 * e]), qc = Ordinal(range, EdgeCorners[2 * e + 1]);
                    if (qa != qc && Root(ref region, qa) == Root(ref region, qc))
                    {
                        continue;
                    }
                    int pair = PlaneGroup(qa, qc);
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

            /// <summary>See SurfaceMesh.CutRegions.</summary>
            private void CutRegions(int cell, int2 range, ref FixedList64Bytes<int> region)
            {
                region.Clear();
                for (int i = 0; i < 8; i++)
                {
                    region.Add(i);
                }
                CopyPairs(cell, range, out ulong inPlane, out ulong crossing);
                for (int a = 0; a < range.y; a++)
                {
                    for (int c = a + 1; c < range.y; c++)
                    {
                        ulong bit = PairBit(a, c);
                        if ((inPlane & bit) != 0 && (crossing & bit) == 0 && PieceOf(range.x + a) == PieceOf(range.x + c) &&
                            !SeparatedNearby(cell, range, a, c))
                        {
                            Merge(ref region, a, c);
                        }
                    }
                }
            }

            /// <summary>See SurfaceMesh.CopyPairs.</summary>
            private void CopyPairs(int cell, int2 range, out ulong inPlane, out ulong crossing)
            {
                inPlane = 0;
                crossing = 0;
                int severed = Severed[cell];
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0)
                    {
                        continue;
                    }
                    int qa = Ordinal(range, EdgeCorners[2 * e]), qc = Ordinal(range, EdgeCorners[2 * e + 1]);
                    if (qa == qc)
                    {
                        continue;
                    }
                    if (math.abs(EdgeNormal[cell * Lattice.EdgeCount + e][e / 4]) < InPlaneEdge)
                    {
                        inPlane |= PairBit(qa, qc);
                    }
                    else
                    {
                        crossing |= PairBit(qa, qc);
                    }
                }
            }

            /// <summary>See SurfaceMesh.SeparatedNearby.</summary>
            private bool SeparatedNearby(int cell, int2 range, int a, int c)
            {
                int severed = Severed[cell];
                int3 dims = PointDims - 1;
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0 ||
                        PairBit(Ordinal(range, EdgeCorners[2 * e]), Ordinal(range, EdgeCorners[2 * e + 1])) != PairBit(a, c))
                    {
                        continue;
                    }
                    int axis = e / 4;
                    int3 start = CellCoord[cell] + Lattice.CornerOffset(EdgeCorners[2 * e]);
                    for (int k = 0; k < 4; k++)
                    {
                        int3 offset = int3.zero;
                        offset[(axis + 1) % 3] = k & 1;
                        offset[(axis + 2) % 3] = k >> 1;
                        int3 coord = start - offset;
                        if (math.any(coord < 0) || math.any(coord >= dims))
                        {
                            continue;
                        }
                        int other = GridToCell[coord.x + dims.x * (coord.y + dims.y * coord.z)];
                        if (other < 0 || other == cell)
                        {
                            continue;
                        }
                        int2 otherRange = CellCopyRange[other];
                        int edge = EdgeFromStart(axis, Lattice.CornerIndex(offset));
                        int qa = Ordinal(otherRange, EdgeCorners[2 * edge]), qc = Ordinal(otherRange, EdgeCorners[2 * edge + 1]);
                        if (qa == qc)
                        {
                            continue;
                        }
                        CopyPairs(other, otherRange, out _, out ulong crossing);
                        if ((crossing & PairBit(qa, qc)) != 0)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            private int EdgeFromStart(int axis, int start)
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

            /// <summary>Real corners of the copies in the region of copy ordinal <paramref name="ordinal"/>.</summary>
            private int RegionMask(int2 range, ref FixedList64Bytes<int> region, int ordinal)
            {
                int mask = 0;
                for (int i = 0; i < range.y; i++)
                {
                    if (Root(ref region, i) == Root(ref region, ordinal))
                    {
                        mask |= Copies[range.x + i].Mask;
                    }
                }
                return mask;
            }

            private int PieceOf(int copy)
            {
                int mask = Copies[copy].Mask;
                for (int k = 0; k < 8; k++)
                {
                    if (((mask >> k) & 1) != 0)
                    {
                        return PieceId[CopyNodes[copy * 8 + k]];
                    }
                }
                return -1;
            }

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
                            Copy = insideA ? a.Copy : c.Copy,
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
