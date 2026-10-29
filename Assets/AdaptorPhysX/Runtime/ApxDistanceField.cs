using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Baked signed distance field of a mesh, used as the shape of a collider by
    /// <see cref="ApxColliderMaterial"/>. Bake it in the editor (inspector button) so that no
    /// field is computed at runtime; an unbaked asset bakes itself on first use.
    /// </summary>
    [CreateAssetMenu(menuName = "AdaptorPhysX/Distance Field", fileName = "DistanceField")]
    public sealed class ApxDistanceField : ScriptableObject
    {
        [Tooltip("Mesh in the local space of the collider's transform. Must be readable.")]
        public Mesh mesh;
        [Tooltip("Grid cells along the mesh's longest axis.")]
        [Range(8, 128)] public int resolution = 32;
        [Tooltip("Extra margin around the mesh in local units (at least two cells are always added).")]
        [Min(0.0f)] public float padding;

        [SerializeField, HideInInspector] private Vector3 _origin;
        [SerializeField, HideInInspector] private float _cellSize;
        [SerializeField, HideInInspector] private Vector3Int _dims;
        [SerializeField, HideInInspector] private float[] _values;
        [SerializeField, HideInInspector] private int _bakedResolution;
        [SerializeField, HideInInspector] private float _bakedPadding;
        [SerializeField, HideInInspector] private Mesh _bakedMesh;

        private DistanceField _field;

        public bool IsBaked => _values != null && _values.Length > 0 && _bakedMesh == mesh &&
                               _bakedResolution == resolution && _bakedPadding == padding;

        /// <summary>The field, baked now if the asset is not baked for its current settings.</summary>
        public DistanceField Field
        {
            get
            {
                if (!IsBaked)
                {
                    Bake();
                }
                return _field ??= new DistanceField
                {
                    Origin = _origin,
                    CellSize = _cellSize,
                    Dims = new int3(_dims.x, _dims.y, _dims.z),
                    Values = _values,
                };
            }
        }

        public void Bake()
        {
            _field = null;
            if (mesh == null || !mesh.isReadable)
            {
                Debug.LogError($"{name}: the distance field needs a readable mesh.", this);
                _values = new[] { float.MaxValue };
                _dims = Vector3Int.one;
                _cellSize = 1.0f;
                return;
            }
            DistanceField field = DistanceField.Bake(mesh.vertices, mesh.triangles, resolution, padding);
            _origin = field.Origin;
            _cellSize = field.CellSize;
            _dims = new Vector3Int(field.Dims.x, field.Dims.y, field.Dims.z);
            _values = field.Values;
            _bakedMesh = mesh;
            _bakedResolution = resolution;
            _bakedPadding = padding;
            _field = field;
        }

        private void OnValidate() => _field = null;
    }
}
