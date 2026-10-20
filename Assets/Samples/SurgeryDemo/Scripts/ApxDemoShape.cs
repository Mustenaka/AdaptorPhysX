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
    }
}
