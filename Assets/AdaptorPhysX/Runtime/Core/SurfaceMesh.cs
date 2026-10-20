using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace AdaptorPhysX
{
    /// <summary>
    /// Renders a soft body: the original surface mesh embedded in lattice cell copies
    /// (trilinear) and split along the cut planes of cut cells, plus cut faces generated from
    /// the Hermite data of severed edges and trimmed to the inside of the original surface.
    /// Submesh 0 is the skin, submesh 1 the cut surface; UV1 holds the rest-space position
    /// for triplanar interior shading.
    /// </summary>
    public sealed class SurfaceMesh : IDisposable
    {
        private readonly SoftBodySimulation _sim;
        private readonly float3[] _restPositions;
        private readonly float3[] _restNormals;
        private readonly Vector2[] _uvs;
        private readonly int[] _triangles;
        private readonly int[] _embedCell;

        // Per-rebuild cut geometry of cells with more than one copy (rest space).
        private readonly Dictionary<int, List<float3>> _cutTriangles = new Dictionary<int, List<float3>>();
        private readonly Dictionary<int, List<float4>> _cutPlanes = new Dictionary<int, List<float4>>();

        private NativeArray<int> _vertexCopy;
        private NativeArray<float3> _vertexU;
        private NativeArray<float3> _vertexRestNormal;
        private NativeArray<float3> _outPositions;
        private NativeArray<float3> _outNormals;
        private NativeArray<float3x3> _nodeJacobian;
        private int _version = -1;

        /// <summary>Pieces with fewer real nodes are tiny fragments of crossing cuts and are not drawn.</summary>
        public const int MinVisiblePieceNodes = 8;

        private bool[] _copyVisible = Array.Empty<bool>();

        public Mesh Mesh { get; }
        public int SkinTriangleCount { get; private set; }
        public int CutTriangleCount { get; private set; }

        /// <param name="restPositions">Surface vertices in the simulation rest space.</param>
        public SurfaceMesh(SoftBodySimulation sim, float3[] restPositions, float3[] restNormals, Vector2[] uvs,
            int[] triangles)
        {
            _sim = sim;
            _restPositions = restPositions;
            _restNormals = restNormals;
            _uvs = uvs;
            _triangles = triangles;
            _embedCell = new int[restPositions.Length];
            for (int v = 0; v < restPositions.Length; v++)
            {
                _embedCell[v] = FindCell(restPositions[v]);
            }
            Mesh = new Mesh { name = "ApxSoftBody", indexFormat = IndexFormat.UInt32 };
            Mesh.MarkDynamic();
        }

        public void Dispose()
        {
            DisposeArrays();
            if (Mesh == null)
            {
                return;
            }
            if (Application.isPlaying)
            {
                UnityEngine.Object.Destroy(Mesh);
            }
            else
            {
                UnityEngine.Object.DestroyImmediate(Mesh);
            }
        }

        /// <summary>Rebuilds render topology if the simulation topology changed, then deforms the mesh.</summary>
        public void Update(float4x4 worldToLocal)
        {
            _sim.CompleteStep();
            if (_version != _sim.TopologyVersion)
            {
                RebuildTopology();
            }
            JobHandle jacobians = new NodeJacobianJob
            {
                Positions = _sim.Positions,
                Copies = _sim.Copies.AsArray(),
                CopyNodes = _sim.CopyNodes.AsArray(),
                NodeJacobian = _nodeJacobian,
            }.Schedule();
            new SkinJob
            {
                Positions = _sim.Positions,
                CopyNodes = _sim.CopyNodes.AsArray(),
                NodeJacobian = _nodeJacobian,
                VertexCopy = _vertexCopy,
                VertexU = _vertexU,
                VertexRestNormal = _vertexRestNormal,
                WorldToLocal = worldToLocal,
                NormalToLocal = math.transpose(math.inverse(new float3x3(
                    worldToLocal.c0.xyz, worldToLocal.c1.xyz, worldToLocal.c2.xyz))),
                OutPositions = _outPositions,
                OutNormals = _outNormals,
            }.Schedule(_vertexCopy.Length, 128, jacobians).Complete();
            Mesh.SetVertices(_outPositions);
            Mesh.SetNormals(_outNormals);
            Mesh.RecalculateBounds();
        }

        private sealed class Builder
        {
            public readonly List<int> Copies = new List<int>();
            public readonly List<float3> Us = new List<float3>();
            public readonly List<float3> Normals = new List<float3>();
            public readonly List<Vector2> Uv0 = new List<Vector2>();
            public readonly List<Vector3> Uv1 = new List<Vector3>();
            public readonly List<int> Skin = new List<int>();
            public readonly List<int> Cut = new List<int>();
            public readonly Dictionary<(int, int, int, int), int> Lookup = new Dictionary<(int, int, int, int), int>();

            public int Add(int copy, float3 u, float3 normal, Vector2 uv, float3 rest)
            {
                Copies.Add(copy);
                Us.Add(u);
                Normals.Add(normal);
                Uv0.Add(uv);
                Uv1.Add(rest);
                return Copies.Count - 1;
            }
        }

        /// <summary>A corner of a (possibly split) skin triangle.</summary>
        private struct SkinCorner
        {
            /// <summary>Source vertex, or -1 for a point created by a split.</summary>
            public int Vertex;
            /// <summary>For split points on a source edge: the edge's vertices (lo, hi) and parameter; else -1.</summary>
            public int EdgeLo, EdgeHi;
            public float EdgeT;
            public float3 Rest;
            public float3 Normal;
            public Vector2 Uv;
        }

        /// <summary>A convex piece of a skin triangle and the side of each plane it was split by.</summary>
        private sealed class Piece
        {
            public List<SkinCorner> Corners;
            public readonly List<float4> Planes = new List<float4>();
            public readonly List<bool> Signs = new List<bool>();
        }

        private void RebuildTopology()
        {
            _version = _sim.TopologyVersion;
            ComputeVisibleCopies();
            var b = new Builder();
            BuildCutGeometry(b);

            var planes = new List<float4>(4);
            var pieces = new List<Piece>(4);
            var next = new List<Piece>(4);
            for (int t = 0; t < _triangles.Length; t += 3)
            {
                var corners = new List<SkinCorner>(3);
                for (int i = 0; i < 3; i++)
                {
                    int v = _triangles[t + i];
                    corners.Add(new SkinCorner
                    {
                        Vertex = v,
                        EdgeLo = -1,
                        EdgeHi = -1,
                        Rest = _restPositions[v],
                        Normal = _restNormals[v],
                        Uv = _uvs[v],
                    });
                }
                pieces.Clear();
                pieces.Add(new Piece { Corners = corners });
                if (TouchesCut(t))
                {
                    CollectPlanes(t, planes);
                    foreach (float4 plane in planes)
                    {
                        next.Clear();
                        foreach (Piece piece in pieces)
                        {
                            Split(piece, plane, next);
                        }
                        (pieces, next) = (next, pieces);
                    }
                }
                foreach (Piece piece in pieces)
                {
                    EmitSkin(b, piece);
                }
            }
            if (b.Skin.Count > 0)
            {
                RemoveHiddenTriangles(b.Skin, b.Copies);
                RemoveHiddenTriangles(b.Cut, b.Copies);
            }

            DisposeArrays();
            int count = b.Copies.Count;
            _vertexCopy = new NativeArray<int>(b.Copies.ToArray(), Allocator.Persistent);
            _vertexU = new NativeArray<float3>(b.Us.ToArray(), Allocator.Persistent);
            _vertexRestNormal = new NativeArray<float3>(b.Normals.ToArray(), Allocator.Persistent);
            _outPositions = new NativeArray<float3>(count, Allocator.Persistent);
            _outNormals = new NativeArray<float3>(count, Allocator.Persistent);
            _nodeJacobian = new NativeArray<float3x3>(_sim.NodeCount, Allocator.Persistent);

            Mesh.Clear();
            Mesh.SetVertices(_outPositions);
            Mesh.SetNormals(_outNormals);
            Mesh.SetUVs(0, b.Uv0);
            Mesh.SetUVs(1, b.Uv1);
            Mesh.subMeshCount = 2;
            Mesh.SetTriangles(b.Skin, 0, false);
            Mesh.SetTriangles(b.Cut, 1, false);
            SkinTriangleCount = b.Skin.Count / 3;
            CutTriangleCount = b.Cut.Count / 3;
        }

        private void ComputeVisibleCopies()
        {
            var pieceSize = new Dictionary<int, int>();
            for (int i = 0; i < _sim.NodeCount; i++)
            {
                int piece = _sim.PieceId[i];
                if (piece >= 0)
                {
                    pieceSize[piece] = pieceSize.TryGetValue(piece, out int c) ? c + 1 : 1;
                }
            }
            _copyVisible = new bool[_sim.CopyCount];
            for (int q = 0; q < _sim.CopyCount; q++)
            {
                int mask = _sim.Copies[q].Mask;
                for (int k = 0; k < 8; k++)
                {
                    if (((mask >> k) & 1) == 0)
                    {
                        continue;
                    }
                    int piece = _sim.PieceId[_sim.CopyNodes[q * 8 + k]];
                    _copyVisible[q] = piece >= 0 && pieceSize[piece] >= MinVisiblePieceNodes;
                    break;
                }
            }
        }

        private void RemoveHiddenTriangles(List<int> indices, List<int> vertexCopies)
        {
            int write = 0;
            for (int t = 0; t < indices.Count; t += 3)
            {
                if (!_copyVisible[vertexCopies[indices[t]]] || !_copyVisible[vertexCopies[indices[t + 1]]] ||
                    !_copyVisible[vertexCopies[indices[t + 2]]])
                {
                    continue;
                }
                indices[write++] = indices[t];
                indices[write++] = indices[t + 1];
                indices[write++] = indices[t + 2];
            }
            indices.RemoveRange(write, indices.Count - write);
        }

        private bool TouchesCut(int t)
        {
            for (int i = 0; i < 3; i++)
            {
                if (_sim.CellCopyRange[_embedCell[_triangles[t + i]]].y > 1)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>Distinct cut planes of the cut cells containing the triangle's vertices.</summary>
        private void CollectPlanes(int t, List<float4> planes)
        {
            planes.Clear();
            for (int i = 0; i < 3; i++)
            {
                if (!_cutPlanes.TryGetValue(_embedCell[_triangles[t + i]], out List<float4> cellPlanes))
                {
                    continue;
                }
                foreach (float4 plane in cellPlanes)
                {
                    bool duplicate = false;
                    foreach (float4 other in planes)
                    {
                        duplicate |= math.dot(plane.xyz, other.xyz) > 0.98f &&
                                     math.abs(plane.w - other.w) < 0.1f * _sim.CellSize;
                    }
                    if (!duplicate)
                    {
                        planes.Add(plane);
                    }
                }
            }
        }

        /// <summary>Splits a piece by a plane; each resulting piece records its side of the plane.</summary>
        private static void Split(Piece piece, float4 plane, List<Piece> output)
        {
            var positive = new List<SkinCorner>(piece.Corners.Count + 1);
            var negative = new List<SkinCorner>(piece.Corners.Count + 1);
            for (int i = 0; i < piece.Corners.Count; i++)
            {
                SkinCorner a = piece.Corners[i];
                SkinCorner c = piece.Corners[(i + 1) % piece.Corners.Count];
                float da = math.dot(plane.xyz, a.Rest) - plane.w;
                float dc = math.dot(plane.xyz, c.Rest) - plane.w;
                (da >= 0.0f ? positive : negative).Add(a);
                if ((da >= 0.0f) == (dc >= 0.0f))
                {
                    continue;
                }
                float s = da / (da - dc);
                var cross = new SkinCorner
                {
                    Vertex = -1,
                    EdgeLo = -1,
                    EdgeHi = -1,
                    Rest = math.lerp(a.Rest, c.Rest, s),
                    Normal = math.normalizesafe(math.lerp(a.Normal, c.Normal, s)),
                    Uv = Vector2.Lerp(a.Uv, c.Uv, s),
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
            if (positive.Count >= 3 && negative.Count >= 3)
            {
                output.Add(WithSide(piece, positive, plane, true));
                output.Add(WithSide(piece, negative, plane, false));
            }
            else
            {
                output.Add(WithSide(piece, piece.Corners, plane, positive.Count >= 3));
            }
        }

        private static Piece WithSide(Piece parent, List<SkinCorner> corners, float4 plane, bool sign)
        {
            var piece = new Piece { Corners = corners };
            piece.Planes.AddRange(parent.Planes);
            piece.Signs.AddRange(parent.Signs);
            piece.Planes.Add(plane);
            piece.Signs.Add(sign);
            return piece;
        }

        private void EmitSkin(Builder b, Piece piece)
        {
            float3 side = float3.zero;
            foreach (SkinCorner c in piece.Corners)
            {
                side += c.Rest;
            }
            side /= piece.Corners.Count;

            int first = -1, previous = -1;
            for (int i = 0; i < piece.Corners.Count; i++)
            {
                int index = SkinVertex(b, piece, piece.Corners[i], side);
                if (i == 0)
                {
                    first = index;
                }
                else if (i >= 2)
                {
                    b.Skin.Add(first);
                    b.Skin.Add(previous);
                    b.Skin.Add(index);
                }
                previous = index;
            }
        }

        private int SkinVertex(Builder b, Piece piece, SkinCorner corner, float3 side)
        {
            int cell = corner.Vertex >= 0 ? _embedCell[corner.Vertex] : FindCell(corner.Rest);
            int copy = piece.Planes.Count > 0
                ? ResolveBySides(cell, corner.Rest, piece, side)
                : ResolveCopy(cell, corner.Rest, side);
            (int, int, int, int) key;
            bool shared = true;
            if (corner.Vertex >= 0)
            {
                key = (corner.Vertex, -1, copy, 0);
            }
            else if (corner.EdgeLo >= 0)
            {
                key = (corner.EdgeLo, corner.EdgeHi, copy, (int)math.round(corner.EdgeT * 65536.0f));
            }
            else
            {
                key = default;
                shared = false;
            }
            if (shared && b.Lookup.TryGetValue(key, out int index))
            {
                return index;
            }
            float3 u = (corner.Rest - CellMin(cell)) / _sim.CellSize;
            index = b.Add(copy, u, corner.Normal, corner.Uv, corner.Rest);
            if (shared)
            {
                b.Lookup.Add(key, index);
            }
            return index;
        }

        /// <summary>Nearest corner of the cell that lies on the recorded side of every splitting plane.</summary>
        private int ResolveBySides(int cell, float3 query, Piece piece, float3 side)
        {
            int2 range = _sim.CellCopyRange[cell];
            if (range.y == 1)
            {
                return range.x;
            }
            float3 cellMin = CellMin(cell);
            int best = -1;
            float bestDistance = float.MaxValue;
            for (int k = 0; k < 8; k++)
            {
                float3 p = cellMin + (float3)Lattice.CornerOffset(k) * _sim.CellSize;
                bool ok = true;
                for (int i = 0; i < piece.Planes.Count && ok; i++)
                {
                    float d = math.dot(piece.Planes[i].xyz, p) - piece.Planes[i].w;
                    ok = (d >= 0.0f) == piece.Signs[i];
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

        /// <summary>
        /// Copy of <paramref name="cell"/> owning the material at <paramref name="query"/> as seen
        /// from <paramref name="side"/> (rest space): the nearest corner reachable from the side
        /// point without crossing a cut face decides.
        /// </summary>
        private int ResolveCopy(int cell, float3 query, float3 side)
        {
            int2 range = _sim.CellCopyRange[cell];
            if (range.y == 1)
            {
                return range.x;
            }
            _cutTriangles.TryGetValue(cell, out List<float3> blockers);
            int sideCell = FindCell(side);
            List<float3> sideBlockers = null;
            if (sideCell != cell)
            {
                _cutTriangles.TryGetValue(sideCell, out sideBlockers);
            }

            Span<int> order = stackalloc int[8];
            Span<float> distance = stackalloc float[8];
            float3 cellMin = CellMin(cell);
            for (int k = 0; k < 8; k++)
            {
                order[k] = k;
                distance[k] = math.distancesq(cellMin + (float3)Lattice.CornerOffset(k) * _sim.CellSize, query);
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
                float3 target = cellMin + (float3)Lattice.CornerOffset(order[i]) * _sim.CellSize;
                if (!Blocked(blockers, side, target) && !Blocked(sideBlockers, side, target))
                {
                    corner = order[i];
                    break;
                }
            }
            return CopyOwning(range, corner);
        }

        private int CopyOwning(int2 range, int corner)
        {
            for (int q = range.x; q < range.x + range.y; q++)
            {
                if (((_sim.Copies[q].Mask >> corner) & 1) != 0)
                {
                    return q;
                }
            }
            return range.x;
        }

        private static bool Blocked(List<float3> triangles, float3 from, float3 to)
        {
            if (triangles == null)
            {
                return false;
            }
            for (int i = 0; i < triangles.Count; i += 3)
            {
                if (Geometry.SegmentTriangle(from, to, triangles[i], triangles[i + 1], triangles[i + 2]) >= 0.0f)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// For every cell with several copies: the cut surface (one polygon per copy and
        /// virtual corner group, from the Hermite points of its boundary edges) as blocker
        /// triangles, one plane per distinct cut through the cell, and the rendered cut faces
        /// clipped to the inside of the original surface.
        /// </summary>
        private void BuildCutGeometry(Builder b)
        {
            _cutTriangles.Clear();
            _cutPlanes.Clear();
            VoxelGrid grid = _sim.Grid;
            float h = _sim.CellSize;
            var polygon = new List<CutVertex>(12);
            var clipped = new List<CutVertex>(16);
            var group = new int[8];
            for (int cell = 0; cell < _sim.CellCount; cell++)
            {
                int2 range = _sim.CellCopyRange[cell];
                if (range.y < 2)
                {
                    continue;
                }
                int severed = _sim.Severed[cell];
                int3 coord = _sim.CellCoord[cell];
                float3 cellMin = CellMin(cell);
                var blockers = new List<float3>();
                _cutTriangles.Add(cell, blockers);
                _cutPlanes.Add(cell, FitPlanes(cell, severed, cellMin));

                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = _sim.Copies[q].Mask;
                    // Group virtual corners that touch through cube edges; each group gets its own polygon.
                    for (int k = 0; k < 8; k++)
                    {
                        group[k] = k;
                    }
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        int a = Lattice.EdgeCorners[2 * e], c = Lattice.EdgeCorners[2 * e + 1];
                        if (((mask >> a) & 1) == 0 && ((mask >> c) & 1) == 0)
                        {
                            Merge(group, a, c);
                        }
                    }
                    for (int g = 0; g < 8; g++)
                    {
                        if (((mask >> g) & 1) != 0 || Root(group, g) != g)
                        {
                            continue;
                        }
                        polygon.Clear();
                        float3 faceNormal = float3.zero;
                        for (int e = 0; e < Lattice.EdgeCount; e++)
                        {
                            int a = Lattice.EdgeCorners[2 * e], c = Lattice.EdgeCorners[2 * e + 1];
                            bool realA = ((mask >> a) & 1) != 0, realC = ((mask >> c) & 1) != 0;
                            if (realA == realC || (severed & (1 << e)) == 0)
                            {
                                continue;
                            }
                            int virtualCorner = realA ? c : a;
                            if (Root(group, virtualCorner) != g)
                            {
                                continue;
                            }
                            float t = _sim.EdgeT[cell * Lattice.EdgeCount + e];
                            float3 outward = (float3)Lattice.CornerOffset(virtualCorner) -
                                             Lattice.CornerOffset(realA ? a : c);
                            float3 n = _sim.EdgeNormal[cell * Lattice.EdgeCount + e];
                            n = math.dot(n, outward) < 0.0f ? -n : n;
                            faceNormal += n;
                            float sdfA = grid.Sdf[grid.PointIndex(coord + Lattice.CornerOffset(a))];
                            float sdfC = grid.Sdf[grid.PointIndex(coord + Lattice.CornerOffset(c))];
                            polygon.Add(new CutVertex
                            {
                                U = math.lerp((float3)Lattice.CornerOffset(a), Lattice.CornerOffset(c), t),
                                Normal = n,
                                Sdf = math.lerp(sdfA, sdfC, t),
                            });
                        }
                        if (polygon.Count < 3)
                        {
                            continue;
                        }
                        SortAroundNormal(polygon, math.normalizesafe(faceNormal));
                        for (int i = 1; i + 1 < polygon.Count; i++)
                        {
                            blockers.Add(cellMin + polygon[0].U * h);
                            blockers.Add(cellMin + polygon[i].U * h);
                            blockers.Add(cellMin + polygon[i + 1].U * h);
                        }
                        ClipInside(polygon, clipped);
                        if (clipped.Count < 3)
                        {
                            continue;
                        }
                        int first = b.Copies.Count;
                        foreach (CutVertex cv in clipped)
                        {
                            b.Add(q, cv.U, math.normalizesafe(cv.Normal), Vector2.zero, cellMin + cv.U * h);
                        }
                        for (int i = 1; i + 1 < clipped.Count; i++)
                        {
                            b.Cut.Add(first);
                            b.Cut.Add(first + i);
                            b.Cut.Add(first + i + 1);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// One plane (n, d) per group of severed edges with parallel Hermite normals. A cell
        /// crossed by two cuts (cross or wedge apex) yields two planes.
        /// </summary>
        private List<float4> FitPlanes(int cell, int severed, float3 cellMin)
        {
            var normals = new List<float3>(2);
            var sums = new List<float3>(2);
            var counts = new List<int>(2);
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                if ((severed & (1 << e)) == 0)
                {
                    continue;
                }
                float3 n = _sim.EdgeNormal[cell * Lattice.EdgeCount + e];
                float3 p = cellMin + HermitePoint(cell, e) * _sim.CellSize;
                int cluster = -1;
                for (int i = 0; i < normals.Count && cluster < 0; i++)
                {
                    if (math.abs(math.dot(n, math.normalize(normals[i]))) > 0.9f)
                    {
                        cluster = i;
                    }
                }
                if (cluster < 0)
                {
                    normals.Add(float3.zero);
                    sums.Add(float3.zero);
                    counts.Add(0);
                    cluster = normals.Count - 1;
                    normals[cluster] = n;
                }
                else
                {
                    normals[cluster] += math.dot(n, normals[cluster]) < 0.0f ? -n : n;
                }
                sums[cluster] += p;
                counts[cluster]++;
            }
            var planes = new List<float4>(normals.Count);
            for (int i = 0; i < normals.Count; i++)
            {
                if (counts[i] < 2)
                {
                    continue;
                }
                float3 n = math.normalize(normals[i]);
                planes.Add(new float4(n, math.dot(n, sums[i] / counts[i])));
            }
            return planes;
        }

        private struct CutVertex
        {
            public float3 U;
            public float3 Normal;
            public float Sdf;
        }

        /// <summary>Orders points counter-clockwise around n, which Unity renders front-facing towards n.</summary>
        private static void SortAroundNormal(List<CutVertex> polygon, float3 n)
        {
            float3 center = float3.zero;
            foreach (CutVertex v in polygon)
            {
                center += v.U;
            }
            center /= polygon.Count;
            float3 axis = math.abs(n.x) < 0.9f ? new float3(1, 0, 0) : new float3(0, 1, 0);
            float3 t1 = math.normalize(math.cross(axis, n));
            float3 t2 = math.cross(n, t1);
            polygon.Sort((x, y) =>
            {
                float ax = math.atan2(math.dot(x.U - center, t2), math.dot(x.U - center, t1));
                float ay = math.atan2(math.dot(y.U - center, t2), math.dot(y.U - center, t1));
                return ax.CompareTo(ay);
            });
        }

        /// <summary>Sutherland-Hodgman clip against the inside (sdf &lt;= 0) of the original surface.</summary>
        private static void ClipInside(List<CutVertex> polygon, List<CutVertex> output)
        {
            output.Clear();
            for (int i = 0; i < polygon.Count; i++)
            {
                CutVertex a = polygon[i];
                CutVertex c = polygon[(i + 1) % polygon.Count];
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

        private float3 HermitePoint(int cell, int edge)
        {
            float t = _sim.EdgeT[cell * Lattice.EdgeCount + edge];
            return math.lerp((float3)Lattice.CornerOffset(Lattice.EdgeCorners[2 * edge]),
                Lattice.CornerOffset(Lattice.EdgeCorners[2 * edge + 1]), t);
        }

        private float3 CellMin(int cell) => _sim.Grid.Origin + (float3)_sim.CellCoord[cell] * _sim.CellSize;

        /// <summary>Cell containing <paramref name="p"/>, else the nearest cell within two rings.</summary>
        private int FindCell(float3 p)
        {
            VoxelGrid grid = _sim.Grid;
            int3 c = (int3)math.floor((p - grid.Origin) / grid.CellSize);
            int found = -1;
            float best = float.MaxValue;
            for (int ring = 0; ring <= 2 && found < 0; ring++)
            {
                for (int z = -ring; z <= ring; z++)
                for (int y = -ring; y <= ring; y++)
                for (int x = -ring; x <= ring; x++)
                {
                    int3 n = c + new int3(x, y, z);
                    if (math.any(n < 0) || math.any(n >= grid.Dims))
                    {
                        continue;
                    }
                    int index = _sim.GridToCell[n.x + grid.Dims.x * (n.y + grid.Dims.y * n.z)];
                    if (index < 0)
                    {
                        continue;
                    }
                    float d = math.distancesq(grid.Origin + ((float3)n + 0.5f) * grid.CellSize, p);
                    if (d < best)
                    {
                        best = d;
                        found = index;
                    }
                }
            }
            return found >= 0 ? found : 0;
        }

        private static int Root(int[] parent, int x)
        {
            while (parent[x] != x)
            {
                x = parent[x];
            }
            return x;
        }

        private static void Merge(int[] parent, int a, int b)
        {
            a = Root(parent, a);
            b = Root(parent, b);
            if (a != b)
            {
                parent[math.max(a, b)] = math.min(a, b);
            }
        }

        private void DisposeArrays()
        {
            if (_vertexCopy.IsCreated) _vertexCopy.Dispose();
            if (_vertexU.IsCreated) _vertexU.Dispose();
            if (_vertexRestNormal.IsCreated) _vertexRestNormal.Dispose();
            if (_outPositions.IsCreated) _outPositions.Dispose();
            if (_outNormals.IsCreated) _outNormals.Dispose();
            if (_nodeJacobian.IsCreated) _nodeJacobian.Dispose();
        }

        /// <summary>
        /// Weighted sum of the cell-centre deformation Jacobians around each node, so shading
        /// normals are continuous across cell faces (only the direction is used).
        /// </summary>
        [BurstCompile(CompileSynchronously = true)]
        private struct NodeJacobianJob : IJob
        {
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<CellCopy> Copies;
            [ReadOnly] public NativeArray<int> CopyNodes;
            public NativeArray<float3x3> NodeJacobian;

            public void Execute()
            {
                for (int i = 0; i < NodeJacobian.Length; i++)
                {
                    NodeJacobian[i] = float3x3.zero;
                }
                float3 center = new float3(0.5f);
                for (int q = 0; q < Copies.Length; q++)
                {
                    float3x3 j = float3x3.zero;
                    for (int k = 0; k < 8; k++)
                    {
                        float3 p = Positions[CopyNodes[q * 8 + k]];
                        float3 g = Geometry.TrilinearGradient(k, center);
                        j.c0 += p * g.x;
                        j.c1 += p * g.y;
                        j.c2 += p * g.z;
                    }
                    j *= Copies[q].Weight;
                    for (int k = 0; k < 8; k++)
                    {
                        int n = CopyNodes[q * 8 + k];
                        NodeJacobian[n] += j;
                    }
                }
            }
        }

        /// <summary>Trilinear skinning; normals use the cofactor of the interpolated node Jacobians.</summary>
        [BurstCompile(CompileSynchronously = true)]
        private struct SkinJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Positions;
            [ReadOnly] public NativeArray<int> CopyNodes;
            [ReadOnly] public NativeArray<float3x3> NodeJacobian;
            [ReadOnly] public NativeArray<int> VertexCopy;
            [ReadOnly] public NativeArray<float3> VertexU;
            [ReadOnly] public NativeArray<float3> VertexRestNormal;
            public float4x4 WorldToLocal;
            public float3x3 NormalToLocal;
            public NativeArray<float3> OutPositions;
            public NativeArray<float3> OutNormals;

            public void Execute(int i)
            {
                int nodeBase = VertexCopy[i] * 8;
                float3 u = VertexU[i];
                float3 x = float3.zero;
                float3x3 j = float3x3.zero;
                for (int k = 0; k < 8; k++)
                {
                    int n = CopyNodes[nodeBase + k];
                    float w = Geometry.TrilinearWeight(k, u);
                    x += w * Positions[n];
                    j += w * NodeJacobian[n];
                }
                float3 rn = VertexRestNormal[i];
                float3 normal = rn.x * math.cross(j.c1, j.c2) + rn.y * math.cross(j.c2, j.c0) +
                                rn.z * math.cross(j.c0, j.c1);
                OutPositions[i] = math.transform(WorldToLocal, x);
                OutNormals[i] = math.normalizesafe(math.mul(NormalToLocal, normal), rn);
            }
        }
    }
}
