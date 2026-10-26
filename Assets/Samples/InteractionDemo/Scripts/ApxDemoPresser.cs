using UnityEngine;

namespace AdaptorPhysX.Samples
{
    /// <summary>
    /// Moves an instrument down by <see cref="depth"/>, holds it and lifts it back, in a loop
    /// (interaction demo: ring press, putty press, sticky paddle).
    /// </summary>
    public sealed class ApxDemoPresser : MonoBehaviour
    {
        [Tooltip("Travel below the start position, in metres.")]
        public float depth = 0.015f;
        [Tooltip("Seconds for: wait, press, hold, lift.")]
        public Vector4 phases = new Vector4(1.0f, 1.0f, 1.0f, 1.0f);
        [Tooltip("Extra rise above the start position at the end of the lift, in metres.")]
        public float overshoot;
        public bool loop = true;

        private Vector3 _start;
        private bool _started;
        private float _time;

        private void FixedUpdate() => Advance(Time.fixedDeltaTime);

        /// <summary>Moves the instrument to its pose <paramref name="dt"/> later (also used by the edit-mode capture).</summary>
        public void Advance(float dt)
        {
            if (!_started)
            {
                _start = transform.position;
                _started = true;
            }
            _time += dt;
            transform.position = _start + Vector3.down * Offset(_time);
        }

        /// <summary>Downward offset at time <paramref name="t"/> after the start.</summary>
        public float Offset(float t)
        {
            float period = phases.x + phases.y + phases.z + phases.w;
            if (loop && period > 0.0f)
            {
                t %= period;
            }
            if (t < phases.x)
            {
                return 0.0f;
            }
            t -= phases.x;
            if (t < phases.y)
            {
                return depth * Mathf.SmoothStep(0.0f, 1.0f, t / phases.y);
            }
            t -= phases.y;
            if (t < phases.z)
            {
                return depth;
            }
            t -= phases.z;
            return Mathf.Lerp(depth, -overshoot, Mathf.SmoothStep(0.0f, 1.0f, Mathf.Clamp01(t / Mathf.Max(phases.w, 1e-3f))));
        }
    }
}
