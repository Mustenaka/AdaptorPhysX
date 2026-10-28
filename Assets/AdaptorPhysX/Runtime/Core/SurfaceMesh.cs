using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
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
    public sealed partial class SurfaceMesh : IDisposable
    {
        private readonly SoftBodySimulation _sim;
        private readonly float3[] _restPositions;
        private readonly float3[] _restNormals;
        private readonly Vector2[] _uvs;
        private readonly int[] _triangles;
        private readonly int[] _embedCell;
        private readonly float3[] _embedU;
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
        private NativeArray<float3> _nativeRestPositions;
        private NativeArray<float3> _nativeRestNormals;
        private NativeArray<float2> _nativeUvs;
        private NativeArray<int> _nativeEdgeCorners;
        private NativeArray<int> _vertexCopy;
        /// <summary>Cell coordinates and rest normals of the source vertices (render vertices 0 .. source count).</summary>
        private NativeArray<float3> _sourceU;
        private NativeArray<float3> _sourceRestNormal;
        /// <summary>The same for the render vertices added by the last rebuild (split points, cut faces).</summary>
        private NativeArray<float3> _extraU;
        private NativeArray<float3> _extraRestNormal;
        /// <summary>UV0 and rest position (UV1) of the source vertices, the start of the static vertex stream.</summary>
        private NativeArray<StaticVertex> _sourceStatic;
        private NativeArray<DeformedVertex> _outVertices;
        private NativeArray<float3x3> _nodeJacobian;
        private int _version = -1;
        private int _renderedStep = -1;
        private float4x4 _renderedMatrix;

        /// <summary>Pieces with fewer real nodes are tiny fragments of crossing cuts and are not drawn.</summary>
        public const int MinVisiblePieceNodes = 8;

        private bool[] _copyVisible = Array.Empty<bool>();

        /// <summary>Stream 0, rewritten every update.</summary>
        [StructLayout(LayoutKind.Sequential)]
        private struct DeformedVertex
        {
            public float3 Position;
            public float3 Normal;
        }

        /// <summary>Stream 1, written when the render topology is rebuilt.</summary>
        [StructLayout(LayoutKind.Sequential)]
        private struct StaticVertex
        {
            public float2 Uv0;
            public float3 Uv1;
        }

        private static readonly VertexAttributeDescriptor[] VertexLayout =
        {
            new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3, 0),
            new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3, 0),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2, 1),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord1, VertexAttributeFormat.Float32, 3, 1),
        };

        /// <summary>Vertex writes skip index validation and bounds, which are set from the lattice nodes.</summary>
        private const MeshUpdateFlags VertexWriteFlags = MeshUpdateFlags.DontValidateIndices |
            MeshUpdateFlags.DontRecalculateBounds | MeshUpdateFlags.DontResetBoneBounds;

        private readonly System.Diagnostics.Stopwatch _timer = new System.Diagnostics.Stopwatch();

        /// <summary>Wall-clock duration of the stages of the last <see cref="Update"/> that did work, in milliseconds.</summary>
        public struct UpdateTimings
        {
            /// <summary>Copying the simulation topology, cut faces and cut planes (render topology rebuild only).</summary>
            public float CutGeometry;
            /// <summary>Classifying the skin triangles and splitting those touching cut cells (rebuild only).</summary>
            public float Split;
            /// <summary>Vertex attributes and index lists of the rebuilt mesh (rebuild only).</summary>
            public float Topology;
            /// <summary>Vertex and index buffers of the rebuilt mesh handed to the Mesh (rebuild only).</summary>
            public float Upload;
            /// <summary>Node Jacobians and skinning jobs.</summary>
            public float Skin;
            /// <summary>Writing the deformed positions and normals to the mesh, and its bounds.</summary>
            public float Write;

            public float Rebuild => CutGeometry + Split + Topology + Upload;
            public float Total => Rebuild + Skin + Write;
        }

        public UpdateTimings LastTimings { get; private set; }

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
            var sourceStatic = new StaticVertex[restPositions.Length];
            for (int v = 0; v < restPositions.Length; v++)
            {
                _embedCell[v] = FindCell(restPositions[v]);
                _embedU[v] = (restPositions[v] - CellMin(_embedCell[v])) / sim.CellSize;
                sourceStatic[v] = new StaticVertex { Uv0 = uvs[v], Uv1 = restPositions[v] };
            }
            _sourceStatic = new NativeArray<StaticVertex>(sourceStatic, Allocator.Persistent);
            _sourceU = new NativeArray<float3>(_embedU, Allocator.Persistent);
            _sourceRestNormal = new NativeArray<float3>(restNormals, Allocator.Persistent);
            _nativeTriangles = new NativeArray<int>(triangles, Allocator.Persistent);
            _nativeEmbedCell = new NativeArray<int>(_embedCell, Allocator.Persistent);
            _nativeRestPositions = new NativeArray<float3>(restPositions, Allocator.Persistent);
            _nativeRestNormals = new NativeArray<float3>(restNormals, Allocator.Persistent);
            _nativeUvs = new NativeArray<float2>(Array.ConvertAll(uvs, uv => (float2)uv), Allocator.Persistent);
            _nativeEdgeCorners = new NativeArray<int>(Lattice.EdgeCorners, Allocator.Persistent);
            Mesh = new Mesh { name = "ApxSoftBody", indexFormat = IndexFormat.UInt32 };
            Mesh.MarkDynamic();
        }

        public void Dispose()
        {
            DisposeArrays();
            if (_nativeTriangles.IsCreated) _nativeTriangles.Dispose();
            if (_nativeEmbedCell.IsCreated) _nativeEmbedCell.Dispose();
            if (_nativeRestPositions.IsCreated) _nativeRestPositions.Dispose();
            if (_nativeRestNormals.IsCreated) _nativeRestNormals.Dispose();
            if (_nativeUvs.IsCreated) _nativeUvs.Dispose();
            if (_nativeEdgeCorners.IsCreated) _nativeEdgeCorners.Dispose();
            if (_sourceStatic.IsCreated) _sourceStatic.Dispose();
            if (_sourceU.IsCreated) _sourceU.Dispose();
            if (_sourceRestNormal.IsCreated) _sourceRestNormal.Dispose();
            if (_nodeJacobian.IsCreated) _nodeJacobian.Dispose();
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

        /// <summary>
        /// Rebuilds render topology if the simulation topology changed, then deforms the mesh.
        /// Does nothing when the body has not stepped (e.g. sleeps) and the transform is unchanged.
        /// </summary>
        public void Update(float4x4 worldToLocal)
        {
            _sim.CompleteStep();
            if (_version == _sim.TopologyVersion && _renderedStep == _sim.StepCount && _renderedMatrix.Equals(worldToLocal))
            {
                return;
            }
            _renderedStep = _sim.StepCount;
            _renderedMatrix = worldToLocal;
            var timings = new UpdateTimings();
            _timer.Restart();
            if (_version != _sim.TopologyVersion)
            {
                RebuildTopology(ref timings);
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
                SourceCount = _sourceU.Length,
                SourceU = _sourceU,
                SourceRestNormal = _sourceRestNormal,
                ExtraU = _extraU,
                ExtraRestNormal = _extraRestNormal,
                WorldToLocal = worldToLocal,
                NormalToLocal = math.transpose(math.inverse(new float3x3(
                    worldToLocal.c0.xyz, worldToLocal.c1.xyz, worldToLocal.c2.xyz))),
                OutVertices = _outVertices,
            }.Schedule(_vertexCopy.Length, 128, jacobians).Complete();
            timings.Skin = Lap();
            Mesh.SetVertexBufferData(_outVertices, 0, 0, _outVertices.Length, 0, VertexWriteFlags);
            Mesh.bounds = LocalBounds(worldToLocal);
            timings.Write = Lap();
            LastTimings = timings;
        }

        /// <summary>
        /// Bounds of the lattice nodes in the mesh's space, one cell larger: skin vertices are trilinear
        /// combinations of the nodes of their cell copy (slightly extrapolated near cells they lie just outside).
        /// </summary>
        private Bounds LocalBounds(float4x4 worldToLocal)
        {
            float3 center = 0.5f * (_sim.BoundsMin + _sim.BoundsMax);
            float3 extents = 0.5f * (_sim.BoundsMax - _sim.BoundsMin) + _sim.CellSize;
            var linear = new float3x3(worldToLocal.c0.xyz, worldToLocal.c1.xyz, worldToLocal.c2.xyz);
            float3 localExtents = math.abs(linear.c0) * extents.x + math.abs(linear.c1) * extents.y +
                                  math.abs(linear.c2) * extents.z;
            return new Bounds(math.transform(worldToLocal, center), 2.0f * localExtents);
        }

        private float Lap()
        {
            float ms = (float)_timer.Elapsed.TotalMilliseconds;
            _timer.Restart();
            return ms;
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
        /// Split result of one skin triangle near a cut, per cell and copy ordinal within the cell
        /// (managed path, for triangles the split job cannot hold, see <see cref="SplitTrianglesJob"/>).
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


        /// <summary>A convex piece of a skin triangle and the side of each plane it was split by.</summary>
        private sealed class Piece
        {
            public List<SkinCorner> Corners;
            public readonly List<float4> Planes = new List<float4>();
            public readonly List<bool> Signs = new List<bool>();
        }

        private void RebuildTopology(ref UpdateTimings timings)
        {
            _version = _sim.TopologyVersion;
            _hasManagedSnapshot = false;

            // Source vertex v is render vertex v, evaluated in the (first) copy of its cell.
            // Triangles away from cuts keep their source indices; only triangles touching a
            // cut cell are split and get extra vertices.
            int baseCount = _restPositions.Length;
            int cellCount = _cellCoord.Length;
            var b = new Builder(baseCount, 64);
            using var visible = new NativeArray<bool>(_sim.CopyCount, Allocator.TempJob);
            using var cellPlanes = new NativeArray<int2>(cellCount, Allocator.TempJob);
            using var cellBlockers = new NativeArray<int2>(cellCount, Allocator.TempJob);
            using var planes = new NativeList<float4>(Allocator.TempJob);
            using var blockers = new NativeList<float3>(Allocator.TempJob);
            bool anyHidden;
            if (ManagedRebuild)
            {
                TakeManagedSnapshot();
                b.Range = _range;
                ComputeVisibleCopies();
                BuildCutGeometry(b);
                visible.CopyFrom(_copyVisible);
                anyHidden = Array.IndexOf(_copyVisible, false) >= 0;
            }
            else
            {
                anyHidden = BuildCutGeometryInJobs(b, visible, cellPlanes, planes, cellBlockers, blockers);
            }
            timings.CutGeometry = Lap();

            // Triangles away from cut cells keep their indices; the rest are split in Burst, in parallel.
            using var staticIndices = new NativeList<int>(_triangles.Length, Allocator.TempJob);
            using var dynamicTriangles = new NativeList<int>(256, Allocator.TempJob);
            new ClassifyTrianglesJob
            {
                Triangles = _nativeTriangles,
                EmbedCell = _nativeEmbedCell,
                CellCopyRange = _sim.CellCopyRange,
                StaticIndices = staticIndices,
                DynamicTriangles = dynamicTriangles,
            }.Run();
            SplitDynamicTriangles(b, dynamicTriangles.AsArray(), cellPlanes, planes.AsArray(), cellBlockers, blockers.AsArray());
            timings.Split = Lap();
            int extra = b.Copies.Count;
            int count = baseCount + extra;
            DisposeArrays();
            _vertexCopy = new NativeArray<int>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            new SourceCopyJob
            {
                EmbedCell = _nativeEmbedCell,
                CellCopyRange = _sim.CellCopyRange,
                VertexCopy = _vertexCopy,
            }.Schedule(baseCount, 1024).Complete();
            if (extra > 0)
            {
                NativeArray<int>.Copy(b.Copies.ToArray(), 0, _vertexCopy, baseCount, extra);
            }
            _extraU = new NativeArray<float3>(b.Us.ToArray(), Allocator.Persistent);
            _extraRestNormal = new NativeArray<float3>(b.Normals.ToArray(), Allocator.Persistent);
            _outVertices = new NativeArray<DeformedVertex>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            if (!_nodeJacobian.IsCreated || _nodeJacobian.Length != _sim.NodeCount)
            {
                if (_nodeJacobian.IsCreated) _nodeJacobian.Dispose();
                _nodeJacobian = new NativeArray<float3x3>(_sim.NodeCount, Allocator.Persistent);
            }

            // Index buffer: untouched skin triangles, split skin triangles, cut faces.
            int staticCount = staticIndices.Length;
            using var indices = new NativeArray<int>(staticCount + b.Skin.Count + b.Cut.Count, Allocator.TempJob,
                NativeArrayOptions.UninitializedMemory);
            NativeArray<int>.Copy(staticIndices.AsArray(), indices, staticCount);
            if (b.Skin.Count > 0)
            {
                NativeArray<int>.Copy(b.Skin.ToArray(), 0, indices, staticCount, b.Skin.Count);
            }
            if (b.Cut.Count > 0)
            {
                NativeArray<int>.Copy(b.Cut.ToArray(), 0, indices, staticCount + b.Skin.Count, b.Cut.Count);
            }
            int skinCount = staticCount + b.Skin.Count, cutCount = b.Cut.Count;
            if (anyHidden)
            {
                using var counts = new NativeArray<int>(2, Allocator.TempJob);
                new RemoveHiddenTrianglesJob
                {
                    Indices = indices,
                    SkinCount = skinCount,
                    VertexCopy = _vertexCopy,
                    CopyVisible = visible,
                    Counts = counts,
                }.Run();
                skinCount = counts[0];
                cutCount = counts[1];
            }

            var extraStatic = new StaticVertex[extra];
            for (int i = 0; i < extra; i++)
            {
                extraStatic[i] = new StaticVertex { Uv0 = b.Uv0[i], Uv1 = b.Uv1[i] };
            }
            timings.Topology = Lap();
            Mesh.Clear();
            Mesh.SetVertexBufferParams(count, VertexLayout);
            Mesh.SetVertexBufferData(_sourceStatic, 0, 0, baseCount, 1, VertexWriteFlags);
            if (extra > 0)
            {
                Mesh.SetVertexBufferData(extraStatic, 0, baseCount, extra, 1, VertexWriteFlags);
            }
            Mesh.SetIndexBufferParams(skinCount + cutCount, IndexFormat.UInt32);
            Mesh.SetIndexBufferData(indices, 0, 0, skinCount + cutCount, MeshUpdateFlags.DontRecalculateBounds);
            Mesh.subMeshCount = 2;
            Mesh.SetSubMesh(0, new SubMeshDescriptor(0, skinCount), MeshUpdateFlags.DontRecalculateBounds);
            Mesh.SetSubMesh(1, new SubMeshDescriptor(skinCount, cutCount), MeshUpdateFlags.DontRecalculateBounds);
            SkinTriangleCount = skinCount / 3;
            CutTriangleCount = cutCount / 3;
            timings.Upload = Lap();
        }

        /// <summary>
        /// Splits the skin triangles touching cut cells (<see cref="SplitTrianglesJob"/>) and appends them to the
        /// builder in order; triangles the job could not hold go through the managed split.
        /// </summary>
        private void SplitDynamicTriangles(Builder b, NativeArray<int> dynamicTriangles, NativeArray<int2> cellPlanes,
            NativeArray<float4> planes, NativeArray<int2> cellBlockers, NativeArray<float3> blockers)
        {
            if (ManagedRebuild)
            {
                var managedPlanes = new List<float4>(4);
                var managedPieces = new List<Piece>(4);
                var managedNext = new List<Piece>(4);
                foreach (int t in dynamicTriangles.ToArray())
                {
                    AppendSplit(b, SplitSkinTriangle(t, managedPlanes, managedPieces, managedNext));
                }
                SplitOverflowCount = 0;
                return;
            }
            var stream = new NativeStream(math.max(dynamicTriangles.Length, 1), Allocator.TempJob);
            using var vertexCopies = new NativeList<int>(Allocator.TempJob);
            using var vertexU = new NativeList<float3>(Allocator.TempJob);
            using var vertexNormals = new NativeList<float3>(Allocator.TempJob);
            using var vertexUv0 = new NativeList<float2>(Allocator.TempJob);
            using var vertexUv1 = new NativeList<float3>(Allocator.TempJob);
            using var indices = new NativeList<int>(Allocator.TempJob);
            using var overflow = new NativeList<int>(Allocator.TempJob);
            try
            {
                VoxelGrid grid = _sim.Grid;
                JobHandle split = new SplitTrianglesJob
                {
                    DynamicTriangles = dynamicTriangles,
                    Triangles = _nativeTriangles,
                    RestPositions = _nativeRestPositions,
                    RestNormals = _nativeRestNormals,
                    Uvs = _nativeUvs,
                    EmbedCell = _nativeEmbedCell,
                    CellCopyRange = _sim.CellCopyRange,
                    Copies = _sim.Copies.AsArray(),
                    CopyNodes = _sim.CopyNodes.AsArray(),
                    Severed = _sim.Severed,
                    EdgeT = _sim.EdgeT,
                    EdgeNormal = _sim.EdgeNormal,
                    CellCoord = _sim.CellCoord,
                    GridToCell = _sim.GridToCell,
                    EdgeCorners = _nativeEdgeCorners,
                    CellPlanes = cellPlanes,
                    Planes = planes,
                    CellBlockers = cellBlockers,
                    Blockers = blockers,
                    Origin = grid.Origin,
                    Dims = grid.Dims,
                    CellSize = grid.CellSize,
                    Output = stream.AsWriter(),
                }.Schedule(dynamicTriangles.Length, 16);
                new MergeSplitsJob
                {
                    Input = stream.AsReader(),
                    Count = dynamicTriangles.Length,
                    CellCopyRange = _sim.CellCopyRange,
                    FirstVertex = b.BaseCount + b.Copies.Count,
                    VertexCopies = vertexCopies,
                    VertexU = vertexU,
                    VertexNormals = vertexNormals,
                    VertexUv0 = vertexUv0,
                    VertexUv1 = vertexUv1,
                    Indices = indices,
                    Overflow = overflow,
                }.Schedule(split).Complete();
            }
            finally
            {
                stream.Dispose();
            }

            AppendVertices(b, vertexCopies, vertexU, vertexNormals, vertexUv0, vertexUv1);
            b.Skin.AddRange(indices.AsArray().ToArray());
            if (overflow.Length > 0)
            {
                // The managed split reads the topology and cut geometry in managed form.
                TakeManagedSnapshot();
                b.Range = _range;
                _cutPlanes.Clear();
                _cutTriangles.Clear();
                int2[] planeRanges = cellPlanes.ToArray(), blockerRanges = cellBlockers.ToArray();
                float4[] planeArray = planes.ToArray();
                float3[] blockerArray = blockers.ToArray();
                for (int cell = 0; cell < planeRanges.Length; cell++)
                {
                    if (planeRanges[cell].y > 0)
                    {
                        _cutPlanes[cell] = new List<float4>(new ArraySegment<float4>(planeArray, planeRanges[cell].x, planeRanges[cell].y));
                    }
                    if (blockerRanges[cell].y > 0)
                    {
                        _cutTriangles[cell] = new List<float3>(new ArraySegment<float3>(blockerArray, blockerRanges[cell].x, blockerRanges[cell].y));
                    }
                }
                var planeList = new List<float4>(4);
                var pieces = new List<Piece>(4);
                var next = new List<Piece>(4);
                foreach (int i in overflow.AsArray().ToArray())
                {
                    AppendSplit(b, SplitSkinTriangle(dynamicTriangles[i], planeList, pieces, next));
                }
            }
            SplitOverflowCount = overflow.Length;
        }

        /// <summary>Skin triangles of the last rebuild that the split job could not hold (handled by the managed split).</summary>
        public int SplitOverflowCount { get; private set; }

        /// <summary>Copy that render vertex <paramref name="vertex"/> is evaluated in (for tests).</summary>
        internal int VertexCopy(int vertex) => _vertexCopy[vertex];

        /// <summary>
        /// Builds the cut geometry and splits every triangle with the managed code, to compare the jobs with it in
        /// tests (<see cref="CutGeometryJob"/>, <see cref="SplitTrianglesJob"/>).
        /// </summary>
        internal static bool ManagedRebuild;

        private bool _hasManagedSnapshot;

        /// <summary>Managed copies of the simulation topology, read by the managed cut geometry and split.</summary>
        private void TakeManagedSnapshot()
        {
            if (_hasManagedSnapshot)
            {
                return;
            }
            _hasManagedSnapshot = true;
            _range = _sim.CellCopyRange.ToArray();
            _copies = _sim.Copies.AsArray().ToArray();
            _copyNodes = _sim.CopyNodes.AsArray().ToArray();
            _pieceId = _sim.PieceId.ToArray();
            _severed = _sim.Severed.ToArray();
            _edgeT = _sim.EdgeT.ToArray();
            _edgeNormal = _sim.EdgeNormal.ToArray();
        }

        /// <summary>
        /// Visible copies, cut planes, blocker triangles and cut faces (<see cref="CutGeometryJob"/>); the cut faces
        /// go into the builder. Returns whether any copy is hidden.
        /// </summary>
        private bool BuildCutGeometryInJobs(Builder b, NativeArray<bool> visible, NativeArray<int2> cellPlanes,
            NativeList<float4> planes, NativeArray<int2> cellBlockers, NativeList<float3> blockers)
        {
            using var hidden = new NativeArray<int>(1, Allocator.TempJob);
            using var cells = new NativeList<int>(Allocator.TempJob);
            using var vertexCopies = new NativeList<int>(Allocator.TempJob);
            using var vertexU = new NativeList<float3>(Allocator.TempJob);
            using var vertexNormals = new NativeList<float3>(Allocator.TempJob);
            using var vertexUv0 = new NativeList<float2>(Allocator.TempJob);
            using var vertexUv1 = new NativeList<float3>(Allocator.TempJob);
            using var indices = new NativeList<int>(Allocator.TempJob);
            JobHandle copies = new VisibleCopiesJob
            {
                Copies = _sim.Copies.AsArray(),
                CopyNodes = _sim.CopyNodes.AsArray(),
                PieceId = _sim.PieceId,
                Visible = visible,
                Hidden = hidden,
            }.Schedule();
            new CutCellsJob { Severed = _sim.Severed, Cells = cells }.Run();
            var stream = new NativeStream(math.max(cells.Length, 1), Allocator.TempJob);
            try
            {
                VoxelGrid grid = _sim.Grid;
                JobHandle geometry = new CutGeometryJob
                {
                    Cells = cells.AsArray(),
                    CellCopyRange = _sim.CellCopyRange,
                    Copies = _sim.Copies.AsArray(),
                    CopyNodes = _sim.CopyNodes.AsArray(),
                    PieceId = _sim.PieceId,
                    Severed = _sim.Severed,
                    EdgeT = _sim.EdgeT,
                    EdgeNormal = _sim.EdgeNormal,
                    CellCoord = _sim.CellCoord,
                    GridToCell = _sim.GridToCell,
                    PointSdf = _sim.PointSdf,
                    EdgeCorners = _nativeEdgeCorners,
                    Origin = grid.Origin,
                    PointDims = grid.PointDims,
                    CellSize = grid.CellSize,
                    Output = stream.AsWriter(),
                }.Schedule(cells.Length, 4);
                geometry = new MergeCutGeometryJob
                {
                    Input = stream.AsReader(),
                    Cells = cells.AsArray(),
                    CellCopyRange = _sim.CellCopyRange,
                    FirstVertex = b.BaseCount + b.Copies.Count,
                    CellPlanes = cellPlanes,
                    Planes = planes,
                    CellBlockers = cellBlockers,
                    Blockers = blockers,
                    VertexCopies = vertexCopies,
                    VertexU = vertexU,
                    VertexNormals = vertexNormals,
                    VertexUv0 = vertexUv0,
                    VertexUv1 = vertexUv1,
                    Indices = indices,
                }.Schedule(geometry);
                JobHandle.CombineDependencies(copies, geometry).Complete();
            }
            finally
            {
                stream.Dispose();
            }
            AppendVertices(b, vertexCopies, vertexU, vertexNormals, vertexUv0, vertexUv1);
            b.Cut.AddRange(indices.AsArray().ToArray());
            return hidden[0] > 0;
        }

        private static void AppendVertices(Builder b, NativeList<int> copies, NativeList<float3> u, NativeList<float3> normals,
            NativeList<float2> uv0, NativeList<float3> uv1)
        {
            b.Copies.AddRange(copies.AsArray().ToArray());
            b.Us.AddRange(u.AsArray().ToArray());
            b.Normals.AddRange(normals.AsArray().ToArray());
            foreach (float2 uv in uv0.AsArray().ToArray())
            {
                b.Uv0.Add(uv);
            }
            foreach (float3 rest in uv1.AsArray().ToArray())
            {
                b.Uv1.Add(rest);
            }
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

        /// <summary>
        /// Distinct cut planes of the cut cells containing the triangle's vertices. Neighbouring cells fit
        /// slightly different planes to the same cut; they count as one when parallel (either orientation)
        /// and close at the triangle. Two such planes would cut a sliver out of the triangle whose corners
        /// lie on opposite sides of the cut in their two cells.
        /// </summary>
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
                        duplicate |= SameCut(plane, other, t);
                    }
                    if (!duplicate)
                    {
                        planes.Add(plane);
                    }
                }
            }
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
                float3 v = _restPositions[_triangles[t + i]];
                if (math.abs(math.dot(a.xyz - b.xyz, v) - (a.w - b.w)) > 0.25f * _sim.CellSize)
                {
                    return false;
                }
            }
            return true;
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

            int count = piece.Corners.Count;
            Span<int> cells = stackalloc int[count];
            Span<int> copies = stackalloc int[count];
            for (int i = 0; i < count; i++)
            {
                SkinCorner corner = piece.Corners[i];
                cells[i] = corner.Vertex >= 0 ? _embedCell[corner.Vertex] : FindCell(corner.Rest);
                copies[i] = Resolve(cells[i], corner.Rest, piece, side);
            }
            // The copy the corners are held together with: the one at the centre of the piece, unless a corner's
            // copy holds more of them together.
            int referenceCell = FindCell(side);
            int referenceCopy = Resolve(referenceCell, side, piece, side);
            int held = Held(cells, copies, piece, referenceCell, referenceCopy);
            for (int i = 0; i < count && held < count; i++)
            {
                int candidate = Held(cells, copies, piece, cells[i], copies[i]);
                if (candidate > held)
                {
                    held = candidate;
                    referenceCell = cells[i];
                    referenceCopy = copies[i];
                }
            }
            for (int i = 0; i < count; i++)
            {
                int together = Together(cells[i], piece.Corners[i].Rest, copies[i], referenceCell, referenceCopy);
                copies[i] = together >= 0 ? together : copies[i];
            }

            int first = -1, previous = -1;
            for (int i = 0; i < count; i++)
            {
                int index = SkinVertex(split, piece.Corners[i], cells[i], copies[i]);
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

        private int Resolve(int cell, float3 query, Piece piece, float3 side) =>
            piece.Planes.Count > 0 ? ResolveBySides(cell, query, piece, side) : ResolveCopy(cell, query, side);

        /// <summary>Number of corners of the piece that can be evaluated in copies holding together with the reference.</summary>
        private int Held(Span<int> cells, Span<int> copies, Piece piece, int referenceCell, int referenceCopy)
        {
            int held = 0;
            for (int i = 0; i < cells.Length; i++)
            {
                held += Together(cells[i], piece.Corners[i].Rest, copies[i], referenceCell, referenceCopy) >= 0 ? 1 : 0;
            }
            return held;
        }

        /// <summary>
        /// <paramref name="copy"/> when it holds together with the reference copy of the piece (<see cref="Joined"/>),
        /// else the copy of the nearest corner of <paramref name="cell"/> that does, or -1. Each corner of a piece is
        /// resolved in its own cell against planes fitted per cell, and where a cut turns within a cell (a wavy stroke,
        /// a second stroke across or along the first) the corners of one piece could land on both sides of it: the
        /// piece then stretched across the opening incision (TODO BUG-15).
        /// </summary>
        private int Together(int cell, float3 query, int copy, int referenceCell, int referenceCopy)
        {
            if (Joined(cell, copy, referenceCell, referenceCopy))
            {
                return copy;
            }
            int2 range = _range[cell];
            float3 cellMin = CellMin(cell);
            int best = -1;
            float bestDistance = float.MaxValue;
            for (int k = 0; k < 8; k++)
            {
                int candidate = CopyOwning(range, k);
                float distance = math.distancesq(cellMin + (float3)Lattice.CornerOffset(k) * _sim.CellSize, query);
                if (distance < bestDistance && Joined(cell, candidate, referenceCell, referenceCopy))
                {
                    bestDistance = distance;
                    best = candidate;
                }
            }
            return best;
        }

        /// <summary>
        /// Whether two copies hold together: the same copy of one cell, or copies of touching cells that share a node.
        /// Copies of cells further apart are taken to.
        /// </summary>
        private bool Joined(int cellA, int copyA, int cellB, int copyB)
        {
            if (cellA == cellB)
            {
                return copyA == copyB;
            }
            if (math.any(math.abs(_cellCoord[cellA] - _cellCoord[cellB]) > 1))
            {
                return true;
            }
            for (int a = 0; a < 8; a++)
            {
                int node = _copyNodes[copyA * 8 + a];
                for (int b = 0; b < 8; b++)
                {
                    if (_copyNodes[copyB * 8 + b] == node)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>Local vertex of the split triangle; split points shared by its pieces are merged.</summary>
        private int SkinVertex(SplitTriangle split, SkinCorner corner, int cell, int copy)
        {
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

        /// <summary>
        /// Nearest corner of the cell that lies on the recorded side of every splitting plane (<see cref="CornerSide"/>).
        /// </summary>
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
                int copy = CopyOwning(range, k);
                bool ok = true;
                for (int i = 0; i < piece.Planes.Count && ok; i++)
                {
                    ok = CornerSide(cell, copy, p, piece.Planes[i]) == piece.Signs[i];
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
        /// Whether corner <paramref name="p"/>, real in <paramref name="copy"/>, lies on the positive side of a
        /// cut plane. A copy bounded by severed edges of that cut (normal parallel to the plane, Hermite point
        /// near it) lies on the side these edges point to from the cut, so all its corners agree with the
        /// topology. The fitted plane is tilted against the edges (deformed cells, edge parameters clamped
        /// away from the nodes), and a corner within a few hundredths of a cell of it can lie across it.
        /// Without such edges, the corner's side of the plane.
        /// </summary>
        private bool CornerSide(int cell, int copy, float3 p, float4 plane)
        {
            float h = _sim.CellSize;
            float3 cellMin = CellMin(cell);
            int mask = _copies[copy].Mask;
            int severed = _severed[cell];
            float along = 0.0f;
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                int a = Lattice.EdgeCorners[2 * e], c = Lattice.EdgeCorners[2 * e + 1];
                bool realA = ((mask >> a) & 1) != 0, realC = ((mask >> c) & 1) != 0;
                if (realA == realC || (severed & (1 << e)) == 0 ||
                    math.abs(math.dot(_edgeNormal[cell * Lattice.EdgeCount + e], plane.xyz)) < 0.9f)
                {
                    continue;
                }
                float3 hermite = cellMin + HermitePoint(cell, e) * h;
                if (math.abs(math.dot(plane.xyz, hermite) - plane.w) > 0.5f * h)
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
        /// For every cell with several copies: the cut surface (one polygon per region of copies
        /// (<see cref="CutRegions"/>) and virtual corner group, from the Hermite points of its boundary edges) as blocker
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
            var region = new int[8];
            for (int cell = 0; cell < _cellCoord.Length; cell++)
            {
                int2 range = _range[cell];
                if (range.y < 2)
                {
                    if (_severed[cell] != 0)
                    {
                        EmitCrackFaces(b, cell, polygon, clipped);
                    }
                    continue;
                }
                int severed = _severed[cell];
                int3 coord = _cellCoord[cell];
                float3 cellMin = CellMin(cell);
                var blockers = new List<float3>();
                _cutTriangles.Add(cell, blockers);
                CutRegions(cell, range, severed, region);
                _cutPlanes.Add(cell, FitPlanes(cell, severed, cellMin, region));

                for (int q = range.x; q < range.x + range.y; q++)
                {
                    if (Root(region, q - range.x) != q - range.x)
                    {
                        continue;
                    }
                    int mask = RegionMask(range, region, q - range.x);
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
                                Copy = CopyOwning(range, realA ? a : c),
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
                            b.Add(cv.Copy, cv.U, math.normalizesafe(cv.Normal), new Vector2(-cv.Sdf, 0.0f), cellMin + cv.U * h);
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
        /// Cells crossed by a cut that did not separate them (where a blade stopped) get a
        /// two-sided face through the Hermite points of each group of parallel severed edges,
        /// so the floor and ends of a partial incision are closed.
        /// </summary>
        private void EmitCrackFaces(Builder b, int cell, List<CutVertex> polygon, List<CutVertex> clipped)
        {
            VoxelGrid grid = _sim.Grid;
            int severed = _severed[cell];
            int3 coord = _cellCoord[cell];
            float3 cellMin = CellMin(cell);
            int copy = _range[cell].x;
            var used = 0;
            for (int seed = 0; seed < Lattice.EdgeCount; seed++)
            {
                if ((severed & (1 << seed)) == 0 || (used & (1 << seed)) != 0)
                {
                    continue;
                }
                float3 reference = _edgeNormal[cell * Lattice.EdgeCount + seed];
                polygon.Clear();
                float3 normal = float3.zero;
                for (int e = seed; e < Lattice.EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0 || (used & (1 << e)) != 0)
                    {
                        continue;
                    }
                    float3 n = _edgeNormal[cell * Lattice.EdgeCount + e];
                    float alignment = math.dot(n, reference);
                    if (math.abs(alignment) < 0.9f)
                    {
                        continue;
                    }
                    used |= 1 << e;
                    n = alignment < 0.0f ? -n : n;
                    normal += n;
                    int a = Lattice.EdgeCorners[2 * e], c = Lattice.EdgeCorners[2 * e + 1];
                    float t = _edgeT[cell * Lattice.EdgeCount + e];
                    float sdfA = grid.Sdf[grid.PointIndex(coord + Lattice.CornerOffset(a))];
                    float sdfC = grid.Sdf[grid.PointIndex(coord + Lattice.CornerOffset(c))];
                    polygon.Add(new CutVertex
                    {
                        U = math.lerp((float3)Lattice.CornerOffset(a), Lattice.CornerOffset(c), t),
                        Normal = n,
                        Sdf = math.lerp(sdfA, sdfC, t),
                        Copy = copy,
                    });
                }
                if (polygon.Count < 3)
                {
                    continue;
                }
                normal = math.normalizesafe(normal);
                SortAroundNormal(polygon, normal);
                ClipInside(polygon, clipped);
                if (clipped.Count < 3)
                {
                    continue;
                }
                for (int side = 0; side < 2; side++)
                {
                    float3 faceNormal = side == 0 ? normal : -normal;
                    int first = b.BaseCount + b.Copies.Count;
                    foreach (CutVertex cv in clipped)
                    {
                        b.Add(copy, cv.U, faceNormal, new Vector2(-cv.Sdf, 0.0f), cellMin + cv.U * _sim.CellSize);
                    }
                    for (int i = 1; i + 1 < clipped.Count; i++)
                    {
                        b.Cut.Add(first);
                        b.Cut.Add(side == 0 ? first + i : first + i + 1);
                        b.Cut.Add(side == 0 ? first + i + 1 : first + i);
                    }
                }
            }
        }

        /// <summary>
        /// One plane (n, d) per group of severed edges between the same two copies with parallel Hermite normals.
        /// A cell crossed by two cuts (cross or wedge apex) yields a plane per cut and side of the other, and two
        /// parallel cuts through one cell (a second stroke along or beside the first) a plane each: grouped by the
        /// normal alone, their edges gave one plane between the two cuts, and skin next to either was evaluated in
        /// the copy across it (TODO BUG-15). Edges inside one copy (a crack) need two for a plane.
        /// </summary>
        private List<float4> FitPlanes(int cell, int severed, float3 cellMin, int[] region)
        {
            int2 range = _range[cell];
            var pairs = new List<int>(2);
            var normals = new List<float3>(2);
            var sums = new List<float3>(2);
            var counts = new List<int>(2);
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                if ((severed & (1 << e)) == 0)
                {
                    continue;
                }
                int qa = CopyOwning(range, Lattice.EdgeCorners[2 * e]) - range.x;
                int qc = CopyOwning(range, Lattice.EdgeCorners[2 * e + 1]) - range.x;
                if (qa != qc && Root(region, qa) == Root(region, qc))
                {
                    continue;
                }
                int pair = PlaneGroup(qa, qc);
                float3 n = _edgeNormal[cell * Lattice.EdgeCount + e];
                float3 p = cellMin + HermitePoint(cell, e) * _sim.CellSize;
                int cluster = -1;
                for (int i = 0; i < normals.Count && cluster < 0; i++)
                {
                    if (pairs[i] == pair && math.abs(math.dot(n, math.normalize(normals[i]))) > 0.9f)
                    {
                        cluster = i;
                    }
                }
                if (cluster < 0)
                {
                    pairs.Add(pair);
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
                if (!FitsPlane(pairs[i], counts[i]))
                {
                    continue;
                }
                float3 n = math.normalize(normals[i]);
                planes.Add(new float4(n, math.dot(n, sums[i] / counts[i])));
            }
            return planes;
        }

        /// <summary>
        /// |n · edge| below which a severed edge lies in its own cut plane (<see cref="CutRegions"/>): the angle between
        /// the edge and the plane is under about 17°.
        /// </summary>
        private const float InPlaneEdge = 0.3f;

        /// <summary>
        /// Parent array over the copy ordinals of a cell: copies of one piece whose corners the cell separates only by severed
        /// edges lying in their own cut plane (|n · edge| &lt; <see cref="InPlaneEdge"/>) form one region, drawn with one cut
        /// face and not cut apart by a plane. A blade passing along a cell edge also severs that edge; its two corners, both on
        /// the same side of the cut, became copies of their own, each drew a triangle around its corner, and the triangle
        /// between them stayed open in the wall (TODO BUG-17). A cut crossing a layer of nodes at a slant also severs edges in
        /// its plane, but there an edge between the two copies crosses the cut as well: they lie on its two sides. The cells
        /// sharing such an edge must agree (<see cref="SeparatedNearby"/>), or the faces of neighbouring cells do not meet.
        /// </summary>
        private void CutRegions(int cell, int2 range, int severed, int[] region)
        {
            for (int i = 0; i < 8; i++)
            {
                region[i] = i;
            }
            CopyPairs(cell, range, severed, out ulong inPlane, out ulong crossing);
            for (int a = 0; a < range.y; a++)
            {
                for (int c = a + 1; c < range.y; c++)
                {
                    ulong bit = PairBit(a, c);
                    if ((inPlane & bit) != 0 && (crossing & bit) == 0 && PieceOf(range.x + a) == PieceOf(range.x + c) &&
                        !SeparatedNearby(cell, range, severed, a, c))
                    {
                        Merge(region, a, c);
                    }
                }
            }
        }

        /// <summary>
        /// The pairs of copies (<see cref="PairBit"/> of their ordinals) that a severed edge of the cell joins: lying in its
        /// cut plane, or crossing it.
        /// </summary>
        private void CopyPairs(int cell, int2 range, int severed, out ulong inPlane, out ulong crossing)
        {
            inPlane = 0;
            crossing = 0;
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                if ((severed & (1 << e)) == 0)
                {
                    continue;
                }
                int qa = CopyOwning(range, Lattice.EdgeCorners[2 * e]) - range.x;
                int qc = CopyOwning(range, Lattice.EdgeCorners[2 * e + 1]) - range.x;
                if (qa == qc)
                {
                    continue;
                }
                if (math.abs(_edgeNormal[cell * Lattice.EdgeCount + e][e / 4]) < InPlaneEdge)
                {
                    inPlane |= PairBit(qa, qc);
                }
                else
                {
                    crossing |= PairBit(qa, qc);
                }
            }
        }

        /// <summary>
        /// Whether another cell sharing one of the severed edges between copies <paramref name="a"/> and <paramref name="c"/>
        /// keeps the edge's corners in two copies joined by an edge that crosses its cut: a later cut passed between the two
        /// nodes (the edge keeps the data of the first cut), as where a second stroke leaves a notch beside the first.
        /// </summary>
        private bool SeparatedNearby(int cell, int2 range, int severed, int a, int c)
        {
            int3 dims = _sim.Grid.Dims;
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                if ((severed & (1 << e)) == 0 ||
                    PairBit(CopyOwning(range, Lattice.EdgeCorners[2 * e]) - range.x,
                        CopyOwning(range, Lattice.EdgeCorners[2 * e + 1]) - range.x) != PairBit(a, c))
                {
                    continue;
                }
                int axis = e / 4;
                int3 start = _cellCoord[cell] + Lattice.CornerOffset(Lattice.EdgeCorners[2 * e]);
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
                    int other = _gridToCell[coord.x + dims.x * (coord.y + dims.y * coord.z)];
                    if (other < 0 || other == cell)
                    {
                        continue;
                    }
                    int2 otherRange = _range[other];
                    int edge = Lattice.EdgeFromStart(axis, Lattice.CornerIndex(offset));
                    int qa = CopyOwning(otherRange, Lattice.EdgeCorners[2 * edge]) - otherRange.x;
                    int qc = CopyOwning(otherRange, Lattice.EdgeCorners[2 * edge + 1]) - otherRange.x;
                    if (qa == qc)
                    {
                        continue;
                    }
                    CopyPairs(other, otherRange, _severed[other], out _, out ulong crossing);
                    if ((crossing & PairBit(qa, qc)) != 0)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>Bit min * 8 + max of a pair of copy ordinals.</summary>
        private static ulong PairBit(int a, int c) => 1UL << (math.min(a, c) * 8 + math.max(a, c));

        /// <summary>Real corners of the copies in the region of copy ordinal <paramref name="ordinal"/>.</summary>
        private int RegionMask(int2 range, int[] region, int ordinal)
        {
            int mask = 0;
            for (int i = 0; i < range.y; i++)
            {
                if (Root(region, i) == Root(region, ordinal))
                {
                    mask |= _copies[range.x + i].Mask;
                }
            }
            return mask;
        }

        /// <summary>Piece of a copy: that of its first real node.</summary>
        private int PieceOf(int copy)
        {
            int mask = _copies[copy].Mask;
            for (int k = 0; k < 8; k++)
            {
                if (((mask >> k) & 1) != 0)
                {
                    return _pieceId[_copyNodes[copy * 8 + k]];
                }
            }
            return -1;
        }

        /// <summary>Group of a severed edge in <see cref="FitPlanes"/>: the ordinals of the copies owning its corners.</summary>
        private static int PlaneGroup(int ordinalA, int ordinalC) =>
            math.min(ordinalA, ordinalC) * Lattice.CornerCount + math.max(ordinalA, ordinalC);

        /// <summary>Whether a group of <paramref name="count"/> parallel edges gives a plane: two edges inside one copy, one between copies.</summary>
        private static bool FitsPlane(int group, int count) =>
            count >= 2 || group / Lattice.CornerCount != group % Lattice.CornerCount;

        private struct CutVertex
        {
            public float3 U;
            public float3 Normal;
            public float Sdf;
            /// <summary>Copy the vertex is evaluated in: the copy owning the real corner of its edge.</summary>
            public int Copy;
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
                        Copy = insideA ? a.Copy : c.Copy,
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
            if (_extraU.IsCreated) _extraU.Dispose();
            if (_extraRestNormal.IsCreated) _extraRestNormal.Dispose();
            if (_outVertices.IsCreated) _outVertices.Dispose();
        }

        /// <summary>
        /// Drops the triangles with a vertex in a hidden copy (fragments, see <see cref="MinVisiblePieceNodes"/>) from the
        /// skin range [0, SkinCount) and the cut range after it, compacting both in place; Counts receives their lengths.
        /// </summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct RemoveHiddenTrianglesJob : IJob
        {
            public NativeArray<int> Indices;
            public int SkinCount;
            [ReadOnly] public NativeArray<int> VertexCopy;
            [ReadOnly] public NativeArray<bool> CopyVisible;
            public NativeArray<int> Counts;

            public void Execute()
            {
                int write = 0;
                for (int t = 0; t < Indices.Length; t += 3)
                {
                    if (t == SkinCount)
                    {
                        Counts[0] = write;
                    }
                    if (!CopyVisible[VertexCopy[Indices[t]]] || !CopyVisible[VertexCopy[Indices[t + 1]]] ||
                        !CopyVisible[VertexCopy[Indices[t + 2]]])
                    {
                        continue;
                    }
                    Indices[write++] = Indices[t];
                    Indices[write++] = Indices[t + 1];
                    Indices[write++] = Indices[t + 2];
                }
                if (SkinCount == Indices.Length)
                {
                    Counts[0] = write;
                }
                Counts[1] = write - Counts[0];
            }
        }

        /// <summary>A source vertex is evaluated in the first copy of the cell it is embedded in.</summary>
        [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
        private struct SourceCopyJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<int> EmbedCell;
            [ReadOnly] public NativeArray<int2> CellCopyRange;
            [WriteOnly] public NativeArray<int> VertexCopy;

            public void Execute(int v) => VertexCopy[v] = CellCopyRange[EmbedCell[v]].x;
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
            public int SourceCount;
            [ReadOnly] public NativeArray<float3> SourceU;
            [ReadOnly] public NativeArray<float3> SourceRestNormal;
            [ReadOnly] public NativeArray<float3> ExtraU;
            [ReadOnly] public NativeArray<float3> ExtraRestNormal;
            public float4x4 WorldToLocal;
            public float3x3 NormalToLocal;
            [WriteOnly] public NativeArray<DeformedVertex> OutVertices;

            public void Execute(int i)
            {
                int nodeBase = VertexCopy[i] * 8;
                float3 u = i < SourceCount ? SourceU[i] : ExtraU[i - SourceCount];
                float3 x = float3.zero;
                float3x3 j = float3x3.zero;
                for (int k = 0; k < 8; k++)
                {
                    int n = CopyNodes[nodeBase + k];
                    float w = Geometry.TrilinearWeight(k, u);
                    x += w * Positions[n];
                    j += w * NodeJacobian[n];
                }
                float3 rn = i < SourceCount ? SourceRestNormal[i] : ExtraRestNormal[i - SourceCount];
                float3 normal = rn.x * math.cross(j.c1, j.c2) + rn.y * math.cross(j.c2, j.c0) +
                                rn.z * math.cross(j.c0, j.c1);
                OutVertices[i] = new DeformedVertex
                {
                    Position = math.transform(WorldToLocal, x),
                    Normal = math.normalizesafe(math.mul(NormalToLocal, normal), rn),
                };
            }
        }
    }
}
