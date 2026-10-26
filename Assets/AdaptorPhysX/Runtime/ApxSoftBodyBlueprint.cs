using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Precomputed geometry of an <see cref="ApxSoftBody"/>: the subdivided surface mesh and the
    /// voxelized lattice for one mesh, resolution and scale. A body whose blueprint matches its
    /// settings starts without subdividing or voxelizing; the simulation is the same as without
    /// the blueprint. Baked from the ApxSoftBody inspector.
    /// </summary>
    public sealed class ApxSoftBodyBlueprint : ScriptableObject
    {
        [SerializeField] private Mesh _mesh;
        [SerializeField] private int _resolution;
        [SerializeField] private Vector3 _scale;

        [SerializeField, HideInInspector] private Vector3[] _positions;
        [SerializeField, HideInInspector] private Vector3[] _normals;
        [SerializeField, HideInInspector] private Vector2[] _uvs;
        [SerializeField, HideInInspector] private int[] _triangles;

        [SerializeField, HideInInspector] private Vector3 _origin;
        [SerializeField, HideInInspector] private float _cellSize;
        [SerializeField, HideInInspector] private Vector3Int _dims;
        [SerializeField, HideInInspector] private float[] _sdf;
        [SerializeField, HideInInspector] private Vector3Int[] _cells;
        [SerializeField, HideInInspector] private float[] _fill;
        [SerializeField, HideInInspector] private float[] _depth;
        [SerializeField, HideInInspector] private Vector3[] _cellNormals;

        public Mesh Mesh => _mesh;
        public int Resolution => _resolution;
        public Vector3 Scale => _scale;
        public int CellCount => _cells?.Length ?? 0;
        public int SurfaceVertexCount => _positions?.Length ?? 0;

        /// <summary>Whether the blueprint was baked for this mesh, resolution and lossy scale.</summary>
        public bool Matches(Mesh mesh, int resolution, Vector3 scale) =>
            _mesh == mesh && _resolution == resolution && _scale == scale && _cells != null && _cells.Length > 0;

        public void Store(Mesh mesh, int resolution, Vector3 scale, float3[] positions, float3[] normals, Vector2[] uvs,
            int[] triangles, VoxelGrid grid)
        {
            _mesh = mesh;
            _resolution = resolution;
            _scale = scale;
            _positions = System.Array.ConvertAll(positions, p => (Vector3)p);
            _normals = System.Array.ConvertAll(normals, n => (Vector3)n);
            _uvs = (Vector2[])uvs.Clone();
            _triangles = (int[])triangles.Clone();
            _origin = grid.Origin;
            _cellSize = grid.CellSize;
            _dims = new Vector3Int(grid.Dims.x, grid.Dims.y, grid.Dims.z);
            _sdf = (float[])grid.Sdf.Clone();
            _cells = System.Array.ConvertAll(grid.Cells, c => new Vector3Int(c.x, c.y, c.z));
            _fill = (float[])grid.Fill.Clone();
            _depth = (float[])grid.Depth.Clone();
            _cellNormals = System.Array.ConvertAll(grid.Normal, n => (Vector3)n);
        }

        public void Load(out float3[] positions, out float3[] normals, out Vector2[] uvs, out int[] triangles,
            out VoxelGrid grid)
        {
            positions = System.Array.ConvertAll(_positions, p => (float3)p);
            normals = System.Array.ConvertAll(_normals, n => (float3)n);
            uvs = (Vector2[])_uvs.Clone();
            triangles = (int[])_triangles.Clone();
            grid = new VoxelGrid
            {
                Origin = _origin,
                CellSize = _cellSize,
                Dims = new int3(_dims.x, _dims.y, _dims.z),
                Sdf = (float[])_sdf.Clone(),
                Cells = System.Array.ConvertAll(_cells, c => new int3(c.x, c.y, c.z)),
                Fill = (float[])_fill.Clone(),
                Depth = (float[])_depth.Clone(),
                Normal = System.Array.ConvertAll(_cellNormals, n => (float3)n),
            };
        }
    }
}
