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
        private readonly float3[] _embedU;
        private readonly Vector3[] _restUv1;
        private readonly int[] _gridToCell;
        private readonly int3[] _cellCoord;

        // Managed snapshot of the simulation topology, refreshed on every rebuild.
        private int2[] _range = Array.Empty<int2>();
        private CellCopy[] _copies = Array.Empty<CellCopy>();
        private int[] _copyNodes = Array.Empty<int>();
        private int[] _pieceId = Array.Empty<int>();
        private ushort[] _severed = Array.Empty<ushort>();
        private float[] _edgeT = Array.Empty<float>();
        private float3[] _edgeNormal = Array.Empty<float3>();

        // Per-rebuild cut geometry of cells with more than one copy (rest space).
        private readonly Dictionary<int, List<float3>> _cutTriangles = new Dictionary<int, List<float3>>();
        private readonly Dictionary<int, List<float4>> _cutPlanes = new Dictionary<int, List<float4>>();

        private NativeArray<int> _nativeTriangles;
        private NativeArray<int> _nativeEmbedCell;
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
            _gridToCell = sim.GridToCell.ToArray();
            _cellCoord = sim.CellCoord.ToArray();
            _embedCell = new int[restPositions.Length];
            _embedU = new float3[restPositions.Length];
            _restUv1 = new Vector3[restPositions.Length];
            for (int v = 0; v < restPositions.Length; v++)
            {
                _embedCell[v] = FindCell(restPositions[v]);
                _embedU[v] = (restPositions[v] - CellMin(_embedCell[v])) / sim.CellSize;
                _restUv1[v] = restPositions[v];
            }
            _nativeTriangles = new NativeArray<int>(triangles, Allocator.Persistent);
            _nativeEmbedCell = new NativeArray<int>(_embedCell, Allocator.Persistent);
            Mesh = new Mesh { name = "ApxSoftBody", indexFormat = IndexFormat.UInt32 };
            Mesh.MarkDynamic();
        }

        public void Dispose()
        {
            DisposeArrays();
            if (_nativeTriangles.IsCreated) _nativeTriangles.Dispose();
            if (_nativeEmbedCell.IsCreated) _nativeEmbedCell.Dispose();
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

        /// <summary>Render vertices beyond the source vertices, plus skin and cut index lists.</summary>
        private sealed class Builder
        {
            public readonly int BaseCount;
            public readonly List<int> Copies = new List<int>();
            public readonly List<float3> Us = new List<float3>();
            public readonly List<float3> Normals = new List<float3>();
            public readonly List<Vector2> Uv0 = new List<Vector2>();
            public readonly List<Vector3> Uv1 = new List<Vector3>();
            public readonly List<int> Skin;
            public readonly List<int> Cut = new List<int>();

            public int2[] Range;

            public Builder(int baseCount, int skinCapacity)
            {
                BaseCount = baseCount;
                Skin = new List<int>(skinCapacity);
            }

            public int Add(int copy, float3 u, float3 normal, Vector2 uv, float3 rest)
            {
                Copies.Add(copy);
                Us.Add(u);
                Normals.Add(normal);
                Uv0.Add(uv);
                Uv1.Add(rest);
                return BaseCount + Copies.Count - 1;
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

        /// <summary>
        /// Split result of one skin triangle near a cut, stored per cell and copy ordinal within
        /// the cell, so it stays valid while copy indices of other cells are renumbered.
        /// </summary>
        private sealed class SplitTriangle
        {
            public readonly List<int> Cell = new List<int>(8);
            public readonly List<int> Ordinal = new List<int>(8);
            /// <summary>Source vertex reused when evaluated in its cell's first copy, else -1.</summary>
            public readonly List<int> Source = new List<int>(8);
            public readonly List<float3> U = new List<float3>(8);
            public readonly List<float3> Normal = new List<float3>(8);
            public readonly List<Vector2> Uv = new List<Vector2>(8);
            public readonly List<float3> Rest = new List<float3>(8);
            public readonly List<int> Indices = new List<int>(12);
            public readonly List<(int, int, int, int)> Keys = new List<(int, int, int, int)>(8);
        }

        private readonly Dictionary<int, SplitTriangle> _splitCache = new Dictionary<int, SplitTriangle>();
        private ushort[] _previousSevered = Array.Empty<ushort>();

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
            ushort[] previousSevered = _previousSevered;
            _range = _sim.CellCopyRange.ToArray();
            _copies = _sim.Copies.AsArray().ToArray();
            _copyNodes = _sim.CopyNodes.AsArray().ToArray();
            _pieceId = _sim.PieceId.ToArray();
            _severed = _sim.Severed.ToArray();
            _edgeT = _sim.EdgeT.ToArray();
            _edgeNormal = _sim.EdgeNormal.ToArray();
            ComputeVisibleCopies();
            bool[] dirty = DirtyCells(previousSevered);
            _previousSevered = _severed;

            // Source vertex v is render vertex v, evaluated in the (first) copy of its cell.
            // Triangles away from cuts keep their source indices; only triangles touching a
            // cut cell are split and get extra vertices.
            int baseCount = _restPositions.Length;
            var b = new Builder(baseCount, _triangles.Length + 64) { Range = _range };
            BuildCutGeometry(b);

            // Triangles away from cut cells keep their indices (classified in Burst); the rest are split here.
            int[] dynamicTriangles;
            using (var staticIndices = new NativeList<int>(_triangles.Length, Allocator.TempJob))
            using (var dynamicList = new NativeList<int>(256, Allocator.TempJob))
            using (var range = new NativeArray<int2>(_range, Allocator.TempJob))
            {
                new ClassifyTrianglesJob
                {
                    Triangles = _nativeTriangles,
                    EmbedCell = _nativeEmbedCell,
                    CellCopyRange = range,
                    StaticIndices = staticIndices,
                    DynamicTriangles = dynamicList,
                }.Run();
                b.Skin.AddRange(staticIndices.AsArray().ToArray());
                dynamicTriangles = dynamicList.AsArray().ToArray();
            }

            var planes = new List<float4>(4);
            var pieces = new List<Piece>(4);
            var next = new List<Piece>(4);
            var used = new HashSet<int>();
            foreach (int t in dynamicTriangles)
            {
                used.Add(t);
                if (!_splitCache.TryGetValue(t, out SplitTriangle split) || IsDirty(t, dirty))
                {
                    split = SplitSkinTriangle(t, planes, pieces, next);
                    _splitCache[t] = split;
                }
                AppendSplit(b, split);
            }
            if (used.Count != _splitCache.Count)
            {
                var stale = new List<int>();
                foreach (int t in _splitCache.Keys)
                {
                    if (!used.Contains(t)) stale.Add(t);
                }
                foreach (int t in stale) _splitCache.Remove(t);
            }
            int extra = b.Copies.Count;
            int count = baseCount + extra;
            var copies = new int[count];
            for (int v = 0; v < baseCount; v++)
            {
                copies[v] = _range[_embedCell[v]].x;
            }
            b.Copies.CopyTo(copies, baseCount);
            if (Array.IndexOf(_copyVisible, false) >= 0)
            {
                RemoveHiddenTriangles(b.Skin, copies);
                RemoveHiddenTriangles(b.Cut, copies);
            }

            DisposeArrays();
            _vertexCopy = new NativeArray<int>(copies, Allocator.Persistent);
            _vertexU = Concat(_embedU, b.Us);
            _vertexRestNormal = Concat(_restNormals, b.Normals);
            _outPositions = new NativeArray<float3>(count, Allocator.Persistent);
            _outNormals = new NativeArray<float3>(count, Allocator.Persistent);
            _nodeJacobian = new NativeArray<float3x3>(_sim.NodeCount, Allocator.Persistent);

            var uv0 = new Vector2[count];
            Array.Copy(_uvs, uv0, baseCount);
            b.Uv0.CopyTo(uv0, baseCount);
            var uv1 = new Vector3[count];
            Array.Copy(_restUv1, uv1, baseCount);
            b.Uv1.CopyTo(uv1, baseCount);
            Mesh.Clear();
            Mesh.SetVertices(_outPositions);
            Mesh.SetNormals(_outNormals);
            Mesh.SetUVs(0, uv0);
            Mesh.SetUVs(1, uv1);
            Mesh.subMeshCount = 2;
            Mesh.SetTriangles(b.Skin, 0, false);
            Mesh.SetTriangles(b.Cut, 1, false);
            SkinTriangleCount = b.Skin.Count / 3;
            CutTriangleCount = b.Cut.Count / 3;
        }

        private void ComputeVisibleCopies()
        {
            var pieceSize = new int[_pieceId.Length];
            foreach (int piece in _pieceId)
            {
                if (piece >= 0)
                {
                    pieceSize[piece]++;
                }
            }
            _copyVisible = new bool[_copies.Length];
            for (int q = 0; q < _copies.Length; q++)
            {
                int mask = _copies[q].Mask;
                for (int k = 0; k < 8; k++)
                {
                    if (((mask >> k) & 1) == 0)
                    {
                        continue;
                    }
                    int piece = _pieceId[_copyNodes[q * 8 + k]];
                    _copyVisible[q] = piece >= 0 && pieceSize[piece] >= MinVisiblePieceNodes;
                    break;
                }
            }
        }

        private static NativeArray<float3> Concat(float3[] head, List<float3> tail)
        {
            var array = new NativeArray<float3>(head.Length + tail.Count, Allocator.Persistent);
            NativeArray<float3>.Copy(head, array, head.Length);
            for (int i = 0; i < tail.Count; i++)
            {
                array[head.Length + i] = tail[i];
            }
            return array;
        }

        private void RemoveHiddenTriangles(List<int> indices, int[] vertexCopies)
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

        /// <summary>Cells whose severed edges changed since the last rebuild, dilated by one cell.</summary>
        private bool[] DirtyCells(ushort[] previousSevered)
        {
            var dirty = new bool[_cellCoord.Length];
            if (previousSevered.Length != _severed.Length)
            {
                for (int c = 0; c < dirty.Length; c++) dirty[c] = true;
                return dirty;
            }
            int3 dims = _sim.Grid.Dims;
            for (int c = 0; c < _severed.Length; c++)
            {
                if (_severed[c] == previousSevered[c])
                {
                    continue;
                }
                int3 p = _cellCoord[c];
                for (int z = -1; z <= 1; z++)
                for (int y = -1; y <= 1; y++)
                for (int x = -1; x <= 1; x++)
                {
                    int3 n = p + new int3(x, y, z);
                    if (math.any(n < 0) || math.any(n >= dims)) continue;
                    int cell = _gridToCell[n.x + dims.x * (n.y + dims.y * n.z)];
                    if (cell >= 0) dirty[cell] = true;
                }
            }
            return dirty;
        }

        private bool IsDirty(int t, bool[] dirty) =>
            dirty[_embedCell[_triangles[t]]] || dirty[_embedCell[_triangles[t + 1]]] || dirty[_embedCell[_triangles[t + 2]]];

        private SplitTriangle SplitSkinTriangle(int t, List<float4> planes, List<Piece> pieces, List<Piece> next)
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
            var split = new SplitTriangle();
            foreach (Piece piece in pieces)
            {
                EmitSkin(split, piece);
            }
            return split;
        }

        private void EmitSkin(SplitTriangle split, Piece piece)
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
                int index = SkinVertex(split, piece, piece.Corners[i], side);
                if (i == 0)
                {
                    first = index;
                }
                else if (i >= 2)
                {
                    split.Indices.Add(first);
                    split.Indices.Add(previous);
                    split.Indices.Add(index);
                }
                previous = index;
            }
        }

        /// <summary>Local vertex of the split triangle; split points shared by its pieces are merged.</summary>
        private int SkinVertex(SplitTriangle split, Piece piece, SkinCorner corner, float3 side)
        {
            int cell = corner.Vertex >= 0 ? _embedCell[corner.Vertex] : FindCell(corner.Rest);
            int copy = piece.Planes.Count > 0
                ? ResolveBySides(cell, corner.Rest, piece, side)
                : ResolveCopy(cell, corner.Rest, side);
            int ordinal = copy - _range[cell].x;
            (int, int, int, int) key = corner.Vertex >= 0
                ? (corner.Vertex, -1, ordinal, 0)
                : (corner.EdgeLo, corner.EdgeHi, ordinal, (int)math.round(corner.EdgeT * 65536.0f));
            bool shareable = corner.Vertex >= 0 || corner.EdgeLo >= 0;
            if (shareable)
            {
                for (int i = 0; i < split.Keys.Count; i++)
                {
                    if (split.Keys[i].Equals(key) && split.Cell[i] == cell)
                    {
                        return i;
                    }
                }
            }
            split.Keys.Add(shareable ? key : (-2, -2, -2, split.Keys.Count));
            split.Cell.Add(cell);
            split.Ordinal.Add(ordinal);
            split.Source.Add(corner.Vertex >= 0 && ordinal == 0 ? corner.Vertex : -1);
            split.U.Add((corner.Rest - CellMin(cell)) / _sim.CellSize);
            split.Normal.Add(corner.Normal);
            split.Uv.Add(corner.Uv);
            split.Rest.Add(corner.Rest);
            return split.Cell.Count - 1;
        }

        /// <summary>Adds a cached split to the mesh with copy indices of the current topology.</summary>
        private static void AppendSplit(Builder b, SplitTriangle split)
        {
            int count = split.Cell.Count;
            Span<int> map = count <= 64 ? stackalloc int[count] : new int[count];
            for (int i = 0; i < count; i++)
            {
                map[i] = split.Source[i] >= 0
                    ? split.Source[i]
                    : b.Add(b.Range[split.Cell[i]].x + split.Ordinal[i], split.U[i], split.Normal[i], split.Uv[i],
                        split.Rest[i]);
            }
            foreach (int index in split.Indices)
            {
                b.Skin.Add(map[index]);
            }
        }

        /// <summary>Nearest corner of the cell that lies on the recorded side of every splitting plane.</summary>
        private int ResolveBySides(int cell, float3 query, Piece piece, float3 side)
        {
            int2 range = _range[cell];
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
            int2 range = _range[cell];
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
                if (((_copies[q].Mask >> corner) & 1) != 0)
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
            for (int cell = 0; cell < _cellCoord.Length; cell++)
            {
                int2 range = _range[cell];
                if (range.y < 2)
                {
                    continue;
                }
                int severed = _severed[cell];
                int3 coord = _cellCoord[cell];
                float3 cellMin = CellMin(cell);
                var blockers = new List<float3>();
                _cutTriangles.Add(cell, blockers);
                _cutPlanes.Add(cell, FitPlanes(cell, severed, cellMin));

                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = _copies[q].Mask;
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
                            float t = _edgeT[cell * Lattice.EdgeCount + e];
                            float3 outward = (float3)Lattice.CornerOffset(virtualCorner) -
                                             Lattice.CornerOffset(realA ? a : c);
                            float3 n = _edgeNormal[cell * Lattice.EdgeCount + e];
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
                        int first = b.BaseCount + b.Copies.Count;
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
                float3 n = _edgeNormal[cell * Lattice.EdgeCount + e];
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
            float t = _edgeT[cell * Lattice.EdgeCount + edge];
            return math.lerp((float3)Lattice.CornerOffset(Lattice.EdgeCorners[2 * edge]),
                Lattice.CornerOffset(Lattice.EdgeCorners[2 * edge + 1]), t);
        }

        private float3 CellMin(int cell) => _sim.Grid.Origin + (float3)_cellCoord[cell] * _sim.CellSize;

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
                    int index = _gridToCell[n.x + grid.Dims.x * (n.y + grid.Dims.y * n.z)];
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

        /// <summary>Splits skin triangles into those touching a cut cell and the untouched rest.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct ClassifyTrianglesJob : IJob
        {
            [ReadOnly] public NativeArray<int> Triangles;
            [ReadOnly] public NativeArray<int> EmbedCell;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            public NativeList<int> StaticIndices;
            public NativeList<int> DynamicTriangles;

            public void Execute()
            {
                for (int t = 0; t < Triangles.Length; t += 3)
                {
                    int a = Triangles[t], b = Triangles[t + 1], c = Triangles[t + 2];
                    if (CellCopyRange[EmbedCell[a]].y > 1 || CellCopyRange[EmbedCell[b]].y > 1 ||
                        CellCopyRange[EmbedCell[c]].y > 1)
                    {
                        DynamicTriangles.Add(t);
                        continue;
                    }
                    StaticIndices.Add(a);
                    StaticIndices.Add(b);
                    StaticIndices.Add(c);
                }
            }
        }

        /// <summary>
        /// Weighted sum of the cell-centre deformation Jacobians around each node, so shading
        /// normals are continuous across cell faces (only the direction is used).
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
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
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
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
