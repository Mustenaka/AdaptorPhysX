using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Drives a two-jaw grasper (<see cref="ApxGrasper"/>, TODO MED-10) with the pointer. Press on the tissue: the grasper
    /// comes in horizontally along the view, one jaw above and one below the tissue, its jaw tips <see cref="bite"/> beyond
    /// the picked point and its jaws open a little wider than the tissue is thick there, and closes on it. Jaws pressed on a
    /// surface from above push it down instead of taking it between them, so the tissue needs an edge to take: a flap, a
    /// strip, the side of a thin sheet. While the button is held, dragging moves the grasper in the plane facing the camera,
    /// the wheel lifts or lowers it, Z / X tilt its jaw tips up or down, A / D turn it about the vertical and Q / E about its
    /// jaws (the jaws' width is the only lever against that turn, and the tissue slips out sooner). Releasing opens the jaws,
    /// lets go and draws the grasper back; releasing with Shift keeps the hold, also while another tool is used, until the
    /// next click. The grasper moves in fixed steps (<see cref="Advance"/>), before the recorder's and the grasper's.
    /// </summary>
    [DefaultExecutionOrder(-110)]
    public sealed class ApxGrasperTool : ApxTool
    {
        public ApxGrasper grasper;
        [Tooltip("How far the jaw tips reach beyond the picked point, metres.")]
        [Min(0.0f)] public float bite = 0.01f;
        [Tooltip("Gap the jaws close to, metres; the grasper holds at gaps up to its graspGap.")]
        [Min(0.0f)] public float closedGap = 0.005f;
        [Tooltip("Room between the open jaws and the tissue on each side, metres.")]
        [Min(0.0f)] public float clearance = 0.004f;
        [Tooltip("Widest the jaws open, metres: thicker tissue is not taken.")]
        [Min(0.0f)] public float maxOpenGap = 0.04f;
        [Tooltip("Distance the grasper comes in from and draws back by, metres.")]
        [Min(0.0f)] public float approachDistance = 0.02f;
        [Tooltip("Seconds to come in, to close the jaws, to open them and to draw back.")]
        [Min(0.01f)] public float moveTime = 0.25f;
        [Tooltip("Lift per wheel step, metres.")]
        [Min(0.0f)] public float liftStep = 0.002f;
        [Tooltip("Key turning speed, degrees per second (four times with Shift).")]
        public float turnSpeed = 45.0f;

        /// <summary>What the grasper is doing.</summary>
        public enum Motion
        {
            Idle,
            Approaching,
            Closing,
            Holding,
            Opening,
            Withdrawing,
        }

        public override string DisplayName => "Grasper";

        public override string Help =>
            "LMB on a tissue edge: grasp it (one jaw above, one below)\n" +
            "drag: move   wheel: lift / lower\n" +
            "Z / X: tilt the tips up / down   A / D: turn about the vertical\n" +
            "Q / E: turn about the jaws\n" +
            "Shift + release: keep holding, click to let go\n" +
            "[ / ]: bite depth";

        public override string Status
        {
            get
            {
                string state = _motion == Motion.Idle ? _hover && !_hoverFits ? "too thick to take here" : "free"
                    : grasper.Held != null ? $"holding {grasper.Held.name}: {grasper.HeldPoints} points, squeeze {grasper.Squeeze:0.00} N, " +
                                             $"grip {grasper.GripCapacity:0.00} N"
                    : grasper.Slipped ? "slipped" : _motion.ToString().ToLowerInvariant();
                if (_motion != Motion.Idle && grasper.TryGetWrench(out ApxWrench wrench))
                {
                    state += $"   force {((Vector3)wrench.Force).magnitude:0.000} N";
                }
                return $"bite {bite * 1000.0f:0} mm   tilt {Tilt:0}°, turned {Yaw:0}° about the vertical, {Roll:0}° about the jaws\n" +
                       $"{state}{(_kept ? "   (kept)" : "")}";
            }
        }

        public Motion State => _motion;
        /// <summary>The grasper is in use, possibly holding while another tool is active.</summary>
        public bool IsBusy => _motion != Motion.Idle;
        /// <summary>Turns since the grasp, degrees: about the vertical, of the tips upward about the axis across the jaws, and about the jaws.</summary>
        public float Yaw { get; private set; }
        public float Tilt { get; private set; }
        public float Roll { get; private set; }
        /// <summary>Where the grasper holds (it comes in to here), and the open gap for the grasped tissue.</summary>
        public Vector3 HoldPosition => _position;
        public float OpenGap => _openGap;

        private Motion _motion;
        private Quaternion _base = Quaternion.identity;
        private Vector3 _position;
        private float _openGap;
        private float _gap;
        /// <summary>0 drawn back by the approach distance, 1 at the hold position.</summary>
        private float _progress;
        private Plane _plane;
        private Vector3 _planeOffset;
        private float _lift;
        private bool _kept;
        private bool _hover;
        private bool _hoverFits;
        private Vector3 _hoverPosition;
        private Quaternion _hoverRotation;
        private float _hoverGap;

        private void FixedUpdate() => Advance(Time.fixedDeltaTime);

        private void OnDisable() => Park();

        protected override void OnDeactivate()
        {
            // A kept hold stays while another tool is used.
            if (!_kept)
            {
                LetGo();
            }
            _hover = false;
        }

        public override void Tick(in ApxToolInput input)
        {
            if (input.Down)
            {
                if (_kept)
                {
                    _kept = false;
                    LetGo();
                    return;
                }
                if (_motion == Motion.Idle)
                {
                    Grasp(input);
                }
            }
            bool holding = _motion == Motion.Approaching || _motion == Motion.Closing || _motion == Motion.Holding;
            if (holding && input.Held && !_kept)
            {
                _lift += input.Scroll * liftStep;
                if (_plane.Raycast(input.Ray, out float enter))
                {
                    _position = input.Ray.GetPoint(enter) + _planeOffset + Vector3.up * _lift;
                }
            }
            if (holding && input.Up && !_kept)
            {
                if (input.Shift)
                {
                    _kept = true;
                }
                else
                {
                    LetGo();
                }
            }
            _hover = _motion == Motion.Idle &&
                     Plan(input.Ray, input.ViewForward, out _hoverPosition, out _hoverRotation, out _hoverGap, out _hoverFits, out _);
        }

        public override bool WantsWheel(in ApxToolInput input) => _motion != Motion.Idle && input.Held && !_kept;

        public override void HandleKeys(float deltaTime, bool shift)
        {
            float step = turnSpeed * deltaTime * (shift ? 4.0f : 1.0f);
            if (Key(KeyCode.A)) Turn(step, 0.0f, 0.0f);
            if (Key(KeyCode.D)) Turn(-step, 0.0f, 0.0f);
            if (Key(KeyCode.Z)) Turn(0.0f, step, 0.0f);
            if (Key(KeyCode.X)) Turn(0.0f, -step, 0.0f);
            if (Key(KeyCode.Q)) Turn(0.0f, 0.0f, step);
            if (Key(KeyCode.E)) Turn(0.0f, 0.0f, -step);
            if (KeyDown(KeyCode.LeftBracket)) bite = Mathf.Max(0.002f, bite - 0.002f);
            if (KeyDown(KeyCode.RightBracket)) bite = Mathf.Min(0.02f, bite + 0.002f);
        }

        /// <summary>
        /// Turns the grasper while it is in use, degrees: <paramref name="yaw"/> about the vertical, <paramref name="tilt"/> its tips
        /// upward about the axis across its jaws, <paramref name="roll"/> about its jaws.
        /// </summary>
        public void Turn(float yaw, float tilt, float roll)
        {
            if (_motion == Motion.Approaching || _motion == Motion.Closing || _motion == Motion.Holding)
            {
                Yaw += yaw;
                Tilt += tilt;
                Roll += roll;
            }
        }

        /// <summary>
        /// Moves the grasper by one fixed step: in, closing, holding at the pointer's position and turn, opening and back out;
        /// then parks it. Called by FixedUpdate; edit-mode code calls it before the grasper's <see cref="ApxGrasper.Step"/>.
        /// </summary>
        public void Advance(float deltaTime)
        {
            if (grasper == null || _motion == Motion.Idle)
            {
                return;
            }
            float rate = deltaTime / moveTime;
            float travel = (_openGap - closedGap) * rate;
            switch (_motion)
            {
                case Motion.Approaching:
                    _progress = Mathf.Min(1.0f, _progress + rate);
                    _motion = _progress >= 1.0f ? Motion.Closing : _motion;
                    break;
                case Motion.Closing:
                    _gap = Mathf.Max(closedGap, _gap - travel);
                    _motion = _gap <= closedGap ? Motion.Holding : _motion;
                    break;
                case Motion.Opening:
                    _gap = Mathf.Min(_openGap, _gap + travel);
                    _motion = _gap >= _openGap ? Motion.Withdrawing : _motion;
                    break;
                case Motion.Withdrawing:
                    _progress = Mathf.Max(0.0f, _progress - rate);
                    if (_progress <= 0.0f)
                    {
                        Park();
                        return;
                    }
                    break;
            }
            Quaternion rotation = Rotation;
            grasper.transform.SetPositionAndRotation(_position - rotation * Vector3.forward * (approachDistance * (1.0f - _progress)),
                rotation);
            grasper.gap = _gap;
        }

        /// <summary>The grasper's rotation: the grasp's, turned about the vertical, tilted and turned about its jaws.</summary>
        private Quaternion Rotation => Quaternion.AngleAxis(Yaw, Vector3.up) * _base * Quaternion.AngleAxis(-Tilt, Vector3.right) *
                                       Quaternion.AngleAxis(Roll, Vector3.forward);

        private void Grasp(in ApxToolInput input)
        {
            if (!Plan(input.Ray, input.ViewForward, out _position, out _base, out _openGap, out bool fits, out _) || !fits)
            {
                return;
            }
            _gap = _openGap;
            _progress = 0.0f;
            _lift = 0.0f;
            Yaw = 0.0f;
            Tilt = 0.0f;
            Roll = 0.0f;
            _kept = false;
            Vector3 normal = input.ViewForward.sqrMagnitude > 0.0f ? -input.ViewForward : Vector3.up;
            _plane = new Plane(normal, _position);
            _planeOffset = _plane.Raycast(input.Ray, out float enter) ? _position - input.Ray.GetPoint(enter) : Vector3.zero;
            _motion = Motion.Approaching;
            grasper.gameObject.SetActive(true);
            Advance(0.0f);
        }

        /// <summary>Opens the jaws and draws the grasper back (it lets go once the jaws open beyond its graspGap).</summary>
        public void LetGo()
        {
            if (_motion == Motion.Approaching || _motion == Motion.Closing || _motion == Motion.Holding)
            {
                _motion = Motion.Opening;
            }
            _kept = false;
        }

        private void Park()
        {
            _motion = Motion.Idle;
            _kept = false;
            if (grasper != null)
            {
                grasper.Release();
                grasper.gameObject.SetActive(false);
            }
        }

        /// <summary>
        /// Pose and open gap of a grasp of the tissue under the ray: coming in horizontally along the view, jaws above and below
        /// the tissue at the bite point, centred on its real nodes there. False when the ray picks no tissue; <paramref name="fits"/>
        /// is false when the tissue is thicker than the jaws open.
        /// </summary>
        public bool Plan(Ray ray, Vector3 viewForward, out Vector3 position, out Quaternion rotation, out float openGap, out bool fits,
            out ApxSoftBody body)
        {
            position = default;
            rotation = Quaternion.identity;
            openGap = 0.0f;
            fits = false;
            body = null;
            if (grasper == null || !ApxToolUtil.PickSurface(ray, out Vector3 point, out body, out _))
            {
                return false;
            }
            Vector3 approach = Vector3.ProjectOnPlane(viewForward, Vector3.up);
            approach = approach.sqrMagnitude > 1e-6f ? approach.normalized : Vector3.forward;
            Vector3 tip = point + approach * bite;
            if (!Thickness(body, tip, out float low, out float high) && !Thickness(body, point, out low, out high))
            {
                low = high = point.y;
            }
            openGap = Mathf.Max(high - low, 0.0f) + 2.0f * clearance;
            fits = openGap <= maxOpenGap;
            rotation = Quaternion.LookRotation(approach, Vector3.up);
            position = new Vector3(tip.x, 0.5f * (low + high), tip.z) - approach * (0.5f * grasper.jawSize.z);
            return true;
        }

        /// <summary>Heights of the lowest and highest real nodes within a cell of the vertical through <paramref name="point"/>.</summary>
        private static bool Thickness(ApxSoftBody body, Vector3 point, out float low, out float high)
        {
            low = high = point.y;
            SoftBodySimulation sim = body.Simulation;
            Bounds bounds = body.WorldBounds;
            float h = sim.CellSize;
            int top = sim.PickNode(new Vector3(point.x, bounds.max.y + h, point.z), Vector3.down, h);
            int bottom = sim.PickNode(new Vector3(point.x, bounds.min.y - h, point.z), Vector3.up, h);
            if (top < 0 || bottom < 0)
            {
                return false;
            }
            high = sim.Positions[top].y;
            low = sim.Positions[bottom].y;
            return true;
        }

        public override void Draw(ApxDebugView view)
        {
            if (grasper == null)
            {
                return;
            }
            if (_motion != Motion.Idle)
            {
                Color color = grasper.Held != null ? new Color(1.0f, 0.85f, 0.2f, 0.9f)
                    : grasper.Slipped ? new Color(1.0f, 0.3f, 0.2f, 0.9f) : new Color(0.3f, 1.0f, 0.9f, 0.9f);
                DrawJaws(view, grasper.transform.position, grasper.transform.rotation, grasper.gap, color);
                view.Overlay.Cross(_position, 0.004f, color);
            }
            else if (_hover)
            {
                Color color = _hoverFits ? new Color(0.3f, 1.0f, 0.9f, 0.5f) : new Color(1.0f, 0.3f, 0.2f, 0.5f);
                DrawJaws(view, _hoverPosition, _hoverRotation, _hoverGap, color);
            }
        }

        private void DrawJaws(ApxDebugView view, Vector3 position, Quaternion rotation, float gap, Color color)
        {
            Vector3 half = 0.5f * grasper.jawSize;
            Vector3 offset = rotation * Vector3.up * (0.5f * (gap + grasper.jawSize.y));
            view.Overlay.WireBox(position + offset, rotation, half, color);
            view.Overlay.WireBox(position - offset, rotation, half, color);
        }
    }
}
