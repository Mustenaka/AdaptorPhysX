using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// A blade edge between two local points. While <see cref="cutting"/> is on, the quad it
    /// sweeps each fixed step cuts every soft body it passes through. A blade that stops
    /// inside the tissue leaves a partial incision.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public sealed class ApxScalpel : MonoBehaviour
    {
        [Tooltip("Blade edge start in local space (e.g. the heel).")]
        public Vector3 edgeStart = Vector3.zero;
        [Tooltip("Blade edge end in local space (e.g. the tip).")]
        public Vector3 edgeEnd = new Vector3(0.0f, -0.03f, 0.0f);
        public bool cutting = true;

        public int LastSeveredEdges { get; private set; }

        private Vector3 _previousStart;
        private Vector3 _previousEnd;
        private bool _hasPrevious;

        /// <summary>Forget the previous blade pose, e.g. after teleporting the scalpel.</summary>
        public void ResetSweep() => _hasPrevious = false;

        private void OnDisable() => _hasPrevious = false;

        private void FixedUpdate() => Sweep();

        /// <summary>Cuts with the quad swept since the previous call. Called every fixed step.</summary>
        public void Sweep()
        {
            Vector3 start = transform.TransformPoint(edgeStart);
            Vector3 end = transform.TransformPoint(edgeEnd);
            LastSeveredEdges = 0;
            if (cutting && _hasPrevious &&
                ((start - _previousStart).sqrMagnitude > 1e-10f || (end - _previousEnd).sqrMagnitude > 1e-10f))
            {
                var sweep = new Bounds(start, Vector3.zero);
                sweep.Encapsulate(end);
                sweep.Encapsulate(_previousStart);
                sweep.Encapsulate(_previousEnd);
                foreach (ApxSoftBody body in ApxSoftBody.Active)
                {
                    if (body.WorldBounds.Intersects(sweep))
                    {
                        LastSeveredEdges += body.Cut(_previousStart, _previousEnd, start, end);
                    }
                }
            }
            _previousStart = start;
            _previousEnd = end;
            _hasPrevious = cutting;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = cutting ? Color.red : Color.gray;
            Gizmos.DrawLine(transform.TransformPoint(edgeStart), transform.TransformPoint(edgeEnd));
        }
    }
}
