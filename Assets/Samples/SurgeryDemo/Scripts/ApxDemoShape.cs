using UnityEngine;

namespace AdaptorPhysX.Samples
{
    /// <summary>Assigns a procedural tissue mesh before <see cref="ApxSoftBody"/> starts.</summary>
    [DefaultExecutionOrder(-200)]
    [RequireComponent(typeof(MeshFilter))]
    public sealed class ApxDemoShape : MonoBehaviour
    {
        public enum Kind
        {
            Box,
            Ellipsoid,
        }

        public Kind kind = Kind.Box;
        [Tooltip("Box size or ellipsoid diameters, in metres.")]
        public Vector3 size = new Vector3(0.12f, 0.05f, 0.1f);

        private void Awake() => Apply();

        public void Apply()
        {
            Mesh mesh = kind == Kind.Box ? MeshUtil.CreateBox(size, 12) : MeshUtil.CreateEllipsoid(size * 0.5f);
            GetComponent<MeshFilter>().sharedMesh = mesh;
        }

        // The mesh only exists in play mode; outline the shape so the body is visible and selectable in the editor.
        private void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                return;
            }
            Gizmos.color = new Color(0.95f, 0.6f, 0.5f);
            Gizmos.matrix = transform.localToWorldMatrix * Matrix4x4.Scale(size);
            if (kind == Kind.Box)
            {
                Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
            }
            else
            {
                Gizmos.DrawWireSphere(Vector3.zero, 0.5f);
            }
        }
    }
}
