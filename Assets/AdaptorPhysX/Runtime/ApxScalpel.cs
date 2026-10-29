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
        /// <summary>Edges severed since the current stroke began.</summary>
        public int StrokeSeveredEdges { get; private set; }
        /// <summary>Whether the last call to <see cref="Sweep"/> swept a quad (see <see cref="GetLastSweep"/>).</summary>
        public bool HasSweep { get; private set; }

        private Vector3 _previousStart;
        private Vector3 _previousEnd;
        private bool _hasPrevious;
        private Vector3 _sweepStart0;
        private Vector3 _sweepEnd0;
        private Vector3 _sweepStart1;
        private Vector3 _sweepEnd1;
        /// <summary>Identifies the current stroke (from ResetSweep or cutting on) to the soft bodies.</summary>
        private int _stroke;
        private static int s_strokes;

        /// <summary>Forget the previous blade pose, e.g. after teleporting the scalpel.</summary>
        public void ResetSweep()
        {
            ApxRecorder.Record(this, ApxOperationKind.ResetSweep);
            _hasPrevious = false;
        }

        /// <summary>Strokes numbered so far by all scalpels; a replay continues from the recording's count (TODO MED-14).</summary>
        internal static int Strokes
        {
            get => s_strokes;
            set => s_strokes = value;
        }

        private void OnDisable() => _hasPrevious = false;

        private void FixedUpdate() => Sweep();

        /// <summary>Cuts with the quad swept since the previous call. Called every fixed step.</summary>
        public void Sweep()
        {
            // The cuts follow from the blade's pose, which a recording holds: a replay sweeps again (TODO MED-14).
            using var unrecorded = ApxRecorder.Scope.Begin();
            Vector3 start = transform.TransformPoint(edgeStart);
            Vector3 end = transform.TransformPoint(edgeEnd);
            LastSeveredEdges = 0;
            HasSweep = false;
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
                        LastSeveredEdges += body.Cut(_previousStart, _previousEnd, start, end, _stroke);
                    }
                }
                _sweepStart0 = _previousStart;
                _sweepEnd0 = _previousEnd;
                _sweepStart1 = start;
                _sweepEnd1 = end;
                HasSweep = true;
            }
            if (!cutting && _hasPrevious)
            {
                // The stroke has ended: the bodies restore what it severed alongside earlier cuts without separating anything.
                foreach (ApxSoftBody body in ApxSoftBody.Active)
                {
                    body.EndStroke(_stroke);
                }
            }
            if (cutting && !_hasPrevious)
            {
                _stroke = ++s_strokes;
                StrokeSeveredEdges = 0;
            }
            StrokeSeveredEdges += LastSeveredEdges;
            _previousStart = start;
            _previousEnd = end;
            _hasPrevious = cutting;
        }

        /// <summary>Edge (start, end) before and after the last sweep, in world space.</summary>
        public void GetLastSweep(out Vector3 start0, out Vector3 end0, out Vector3 start1, out Vector3 end1)
        {
            start0 = _sweepStart0;
            end0 = _sweepEnd0;
            start1 = _sweepStart1;
            end1 = _sweepEnd1;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = cutting ? Color.red : Color.gray;
            Gizmos.DrawLine(transform.TransformPoint(edgeStart), transform.TransformPoint(edgeEnd));
        }
    }
}
