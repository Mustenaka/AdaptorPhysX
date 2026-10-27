using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Grabs the tissue under the pointer (<see cref="ApxSoftBody.BeginGrab"/>) and drags it in the
    /// plane facing the camera; the wheel lifts or lowers the grab while the button is held. Releasing
    /// with Shift keeps the grab until the next click, so the camera can be moved meanwhile.
    /// </summary>
    public sealed class ApxForcepsTool : ApxTool
    {
        [Tooltip("Rest-space radius of the grabbed material, metres.")]
        [Min(0.001f)] public float grabRadius = 0.012f;
        [Tooltip("Lift per wheel step, metres.")]
        [Min(0.0f)] public float liftStep = 0.002f;

        public override string DisplayName => "Forceps";

        public override string Help =>
            "LMB drag: grab and pull (does not cross an incision)\n" +
            "wheel while held: lift / lower\n" +
            "Shift + release: keep holding, click to let go\n" +
            "[ / ]: grab radius";

        public override string Status
        {
            get
            {
                string state = Grabbed == null ? "free" : _kept ? $"holding {Grabbed.name}" : $"grabbing {Grabbed.name}";
                if (Grabbed != null && Grabbed.TryGetGrabWrench(out ApxWrench wrench))
                {
                    Vector3 f = wrench.Force;
                    state += $"   force {f.magnitude:0.000} N";
                }
                return $"radius {grabRadius * 1000.0f:0} mm   lift {_lift * 1000.0f:0} mm   {state}";
            }
        }

        public ApxSoftBody Grabbed { get; private set; }
        public Vector3 Target => _target;

        private Plane _plane;
        private Vector3 _start;
        private Vector3 _target;
        private float _lift;
        private bool _kept;
        private bool _hover;
        private Vector3 _hoverPoint;

        protected override void OnDeactivate()
        {
            Release();
            _hover = false;
        }

        private void OnDisable() => Release();

        public override void Tick(in ApxToolInput input)
        {
            if (input.Down)
            {
                if (_kept)
                {
                    Release();
                    return;
                }
                Grab(input);
            }
            if (Grabbed != null && input.Held && !_kept)
            {
                _lift += input.Scroll * liftStep;
                if (_plane.Raycast(input.Ray, out float enter))
                {
                    _target = input.Ray.GetPoint(enter) + Vector3.up * _lift;
                }
            }
            if (Grabbed != null)
            {
                Grabbed.MoveGrab(_target);
            }
            if (Grabbed != null && input.Up && !_kept)
            {
                if (input.Shift)
                {
                    _kept = true;
                }
                else
                {
                    Release();
                }
            }
            _hover = Grabbed == null && ApxToolUtil.PickSurface(input.Ray, out _hoverPoint, out _, out _);
        }

        public override bool WantsWheel(in ApxToolInput input) => Grabbed != null && input.Held;

        private void Grab(in ApxToolInput input)
        {
            if (!ApxToolUtil.PickSurface(input.Ray, out _, out ApxSoftBody body, out _) ||
                !body.BeginGrab(input.Ray, grabRadius, out Vector3 point))
            {
                return;
            }
            Grabbed = body;
            _start = point;
            _target = point;
            _lift = 0.0f;
            _kept = false;
            Vector3 normal = input.ViewForward.sqrMagnitude > 0.0f ? -input.ViewForward : Vector3.up;
            _plane = new Plane(normal, point);
        }

        public void Release()
        {
            if (Grabbed != null)
            {
                Grabbed.EndGrab();
            }
            Grabbed = null;
            _kept = false;
        }

        public override void HandleKeys(float deltaTime, bool shift)
        {
            if (KeyDown(KeyCode.LeftBracket)) grabRadius = Mathf.Max(0.002f, grabRadius - 0.002f);
            if (KeyDown(KeyCode.RightBracket)) grabRadius = Mathf.Min(0.05f, grabRadius + 0.002f);
        }

        public override void Draw(ApxDebugView view)
        {
            var color = new Color(0.3f, 1.0f, 0.9f, 0.9f);
            if (Grabbed != null)
            {
                view.Overlay.Line(_start, _target, new Color(color.r, color.g, color.b, 0.5f));
                view.Overlay.WireSphere(_target, grabRadius, color, 24);
                view.Overlay.Cross(_target, 0.004f, color);
            }
            else if (_hover)
            {
                view.Overlay.WireSphere(_hoverPoint, grabRadius, new Color(color.r, color.g, color.b, 0.4f), 24);
            }
        }
    }
}
