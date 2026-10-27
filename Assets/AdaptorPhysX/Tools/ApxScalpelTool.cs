using System.Collections.Generic;
using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Drives an <see cref="ApxScalpel"/> with the pointer. Press on the tissue (or beside it, for a
    /// cut that enters from the side) and drag: the pointer moves on the horizontal plane through the
    /// press point, and the blade trails it on a string of length <see cref="stabilizer"/>, keeps its
    /// depth below the tissue under it and, with <see cref="followStroke"/>, turns to the direction it
    /// moves in. The cut starts once the pointer has moved <see cref="startDistance"/>, so that the
    /// first heading is the stroke's. The orientation (<see cref="blade"/>) is set with the keys;
    /// Shift while dragging keeps the stroke on a straight line along the heading.
    /// </summary>
    public sealed class ApxScalpelTool : ApxTool
    {
        public ApxScalpel scalpel;
        public ApxBladePose blade = new ApxBladePose { depth = 0.015f, clearance = 0.01f };
        [Tooltip("The blade turns to the direction of the stroke; otherwise it keeps its heading.")]
        public bool followStroke = true;
        [Tooltip("Length of the string the blade trails the pointer on while cutting, metres. Pointer " +
                 "jitter shorter than the string does not reach the blade; 0 moves the blade with the pointer.")]
        [Min(0.0f)] public float stabilizer = 0.008f;
        [Tooltip("Pointer travel on the tissue before the cut starts, metres.")]
        [Min(0.0f)] public float startDistance = 0.003f;
        [Tooltip("Blade travel between heading updates while cutting, metres.")]
        [Min(1e-4f)] public float headingStep = 0.002f;
        [Tooltip("Largest heading change while cutting, degrees per second.")]
        [Min(1.0f)] public float maxTurnRate = 240.0f;
        [Tooltip("A press beside a body starts a stroke when within this horizontal distance, metres.")]
        [Min(0.0f)] public float besideMargin = 0.03f;
        [Tooltip("Key rotation speed, degrees per second (four times with Shift).")]
        public float angleSpeed = 30.0f;
        [Tooltip("Key depth speed, metres per second (four times with Shift).")]
        public float depthSpeed = 0.02f;
        [Tooltip("Deepest blade, metres.")]
        [Min(0.001f)] public float maxDepth = 0.3f;

        public override string DisplayName => "Scalpel";

        public override string Help =>
            "LMB drag: cut (press on or beside the tissue)\n" +
            "Shift + drag: straight stroke along the heading\n" +
            "Q / E or Ctrl + wheel: depth   Z / X: tilt   C / V: rake\n" +
            "A / D: heading   T: follow stroke on/off   B: upright blade\n" +
            "[ / ]: stabilizer string";

        public override string Status =>
            $"heading {blade.heading:0}°  tilt {blade.tilt:0}°  rake {blade.rake:0}°  depth {blade.depth * 1000.0f:0.0} mm\n" +
            $"{(followStroke ? "heading follows the stroke" : "fixed heading")}, stabilizer {stabilizer * 1000.0f:0} mm" +
            (_pressed ? (IsCutting ? "   CUTTING" : "   move to start") : "") +
            $"\nstroke: {StrokeLength * 1000.0f:0} mm, {SeveredThisStroke} edges severed";

        /// <summary>The blade is down and cutting.</summary>
        public bool IsCutting => _pressed && _started;
        public float StrokeLength { get; private set; }
        public int SeveredThisStroke => scalpel != null && _strokeUsed ? scalpel.StrokeSeveredEdges : 0;
        /// <summary>Point of the tissue surface the blade is held at.</summary>
        public Vector3 BladePoint => _point;
        public IReadOnlyList<Vector3> Trail => _trail;

        private bool _pressed;
        private bool _started;
        private bool _strokeUsed;
        private ApxSoftBody _body;
        /// <summary>Height of the plane the pointer moves on during the stroke.</summary>
        private float _planeHeight;
        private float _height;
        private Vector3 _point;
        private Vector3 _pointer;
        private Vector3 _origin;
        private Vector3 _headingAnchor;
        private bool _hover;
        private Vector3 _hoverPoint;
        private readonly List<Vector3> _trail = new List<Vector3>();

        private void Start()
        {
            if (scalpel != null)
            {
                scalpel.cutting = false;
            }
        }

        protected override void OnDeactivate()
        {
            EndStroke();
            _hover = false;
        }

        public override bool WantsWheel(in ApxToolInput input) => input.Ctrl;

        public override void Tick(in ApxToolInput input)
        {
            if (scalpel == null)
            {
                return;
            }
            if (input.Ctrl && input.Scroll != 0.0f)
            {
                blade.depth += input.Scroll * 0.001f;
            }
            blade.Clamp(maxDepth);
            if (input.Down && !_pressed)
            {
                BeginStroke(input.Ray);
            }
            if (_pressed && input.Held)
            {
                Drag(input);
            }
            if (_pressed && (input.Up || !input.Held))
            {
                EndStroke();
            }
            scalpel.cutting = IsCutting;
            _hover = !_pressed && ApxToolUtil.PickSurface(input.Ray, out _hoverPoint, out _, out _);
        }

        private void BeginStroke(Ray ray)
        {
            if (ApxToolUtil.PickSurface(ray, out Vector3 hit, out ApxSoftBody body, out _))
            {
                _point = hit;
            }
            else if (ApxToolUtil.PickBeside(ray, besideMargin, out hit, out body))
            {
                _point = hit;
            }
            else
            {
                return;
            }
            _body = body;
            _planeHeight = _point.y;
            _height = _point.y;
            _origin = _point;
            _pointer = _point;
            _headingAnchor = _point;
            _pressed = true;
            _started = false;
            _strokeUsed = false;
            StrokeLength = 0.0f;
            _trail.Clear();
            _trail.Add(_point);
            PlaceBlade();
            if (!followStroke || startDistance <= 0.0f)
            {
                StartCut();
            }
        }

        private void StartCut()
        {
            _started = true;
            _strokeUsed = true;
            PlaceBlade();
            // The scalpel records this pose at its next sweep and cuts from there.
            scalpel.ResetSweep();
        }

        private void Drag(in ApxToolInput input)
        {
            // The plane stays at the height of the press for the whole stroke. Following the tissue
            // height instead would move the pointer along the view direction whenever the surface
            // under the blade drops, e.g. into the opening cut, and the blade would jump with it.
            if (!new Plane(Vector3.up, new Vector3(0.0f, _planeHeight, 0.0f)).Raycast(input.Ray, out float enter))
            {
                return;
            }
            Vector3 target = input.Ray.GetPoint(enter);
            if (input.Shift && (_started || !followStroke))
            {
                Vector3 forward = blade.Forward;
                target = _origin + forward * Vector3.Dot(target - _origin, forward);
            }
            _pointer = target;
            Vector3 move = target - _point;
            move.y = 0.0f;
            if (!_started)
            {
                if (move.magnitude < startDistance)
                {
                    return;
                }
                // The stroke begins where the blade was pressed, turned along the first motion; it
                // moves on from the next frame so that the scalpel sweeps from this pose.
                blade.heading = ApxBladePose.HeadingOf(move);
                _headingAnchor = _point;
                StartCut();
                return;
            }
            // The blade trails the pointer on a string: it moves towards the pointer by what the
            // pointer is beyond the string length. Hand jitter shorter than the string does not reach
            // the blade; the lattice would turn a zig-zag of the blade into a ragged cut surface.
            float pull = move.magnitude - stabilizer;
            if (pull <= 0.0f)
            {
                return;
            }
            Vector3 next = _point + move.normalized * pull;
            if (followStroke && !input.Shift)
            {
                Vector3 travel = next - _headingAnchor;
                travel.y = 0.0f;
                if (travel.magnitude >= headingStep)
                {
                    blade.heading = Mathf.MoveTowardsAngle(blade.heading, ApxBladePose.HeadingOf(travel),
                        maxTurnRate * Mathf.Max(input.DeltaTime, 1e-3f));
                    blade.Clamp(maxDepth);
                    _headingAnchor = next;
                }
            }
            // The blade keeps its depth below the tissue under it; beyond the tissue it keeps the last height.
            if (ApxToolUtil.SurfaceHeight(_body, next, out float surface))
            {
                _height = surface;
            }
            next.y = _height;
            StrokeLength += new Vector2(next.x - _point.x, next.z - _point.z).magnitude;
            _point = next;
            if ((_point - _trail[_trail.Count - 1]).sqrMagnitude > 1e-6f)
            {
                _trail.Add(_point);
            }
            PlaceBlade();
        }

        private void EndStroke()
        {
            _pressed = false;
            _started = false;
            if (scalpel != null)
            {
                scalpel.cutting = false;
            }
        }

        /// <summary>Puts the scalpel's edge through <see cref="BladePoint"/> with the current pose.</summary>
        private void PlaceBlade()
        {
            Quaternion rotation = blade.Rotation;
            blade.GetEdge(_point, out Vector3 heel, out Vector3 tip);
            scalpel.transform.SetPositionAndRotation(_point, rotation);
            Quaternion inverse = Quaternion.Inverse(rotation);
            scalpel.edgeStart = inverse * (heel - _point);
            scalpel.edgeEnd = inverse * (tip - _point);
        }

        public override void HandleKeys(float deltaTime, bool shift)
        {
            float speed = shift ? 4.0f : 1.0f;
            float angle = angleSpeed * speed * deltaTime;
            if (Key(KeyCode.Q)) blade.depth -= depthSpeed * speed * deltaTime;
            if (Key(KeyCode.E)) blade.depth += depthSpeed * speed * deltaTime;
            if (Key(KeyCode.Z)) blade.tilt -= angle;
            if (Key(KeyCode.X)) blade.tilt += angle;
            if (Key(KeyCode.C)) blade.rake -= angle;
            if (Key(KeyCode.V)) blade.rake += angle;
            if (Key(KeyCode.A)) blade.heading -= angle;
            if (Key(KeyCode.D)) blade.heading += angle;
            if (KeyDown(KeyCode.T)) followStroke = !followStroke;
            if (KeyDown(KeyCode.LeftBracket)) stabilizer = Mathf.Max(0.0f, stabilizer - 0.002f);
            if (KeyDown(KeyCode.RightBracket)) stabilizer = Mathf.Min(0.03f, stabilizer + 0.002f);
            if (KeyDown(KeyCode.B))
            {
                blade.tilt = 0.0f;
                blade.rake = 0.0f;
            }
            blade.Clamp(maxDepth);
            if (_pressed)
            {
                PlaceBlade();
            }
        }

        public override void Draw(ApxDebugView view)
        {
            if (scalpel == null)
            {
                return;
            }
            if (_trail.Count > 1)
            {
                view.Overlay.Polyline(_trail, new Color(1.0f, 0.85f, 0.2f, 0.9f));
            }
            if (!_pressed && !_hover)
            {
                return;
            }
            Vector3 point = _pressed ? _point : _hoverPoint;
            blade.GetEdge(point, out Vector3 heel, out Vector3 tip);
            Color color = IsCutting ? new Color(1.0f, 0.2f, 0.15f) : _pressed ? new Color(1.0f, 0.6f, 0.1f) : new Color(0.2f, 0.9f, 1.0f);
            ApxBladePose.Draw(view.Overlay, heel, tip, blade.Rotation * Vector3.forward, color);
            // Surface level across the blade and the cutting direction.
            Vector3 forward = blade.Forward;
            view.Overlay.Line(point - forward * 0.006f, point + forward * 0.006f, new Color(1.0f, 1.0f, 1.0f, 0.8f));
            view.Overlay.Arrow(point, point + forward * 0.015f, new Color(1.0f, 1.0f, 1.0f, 0.5f));
            view.Overlay.Line(tip, new Vector3(tip.x, point.y, tip.z), new Color(1.0f, 1.0f, 1.0f, 0.3f));
            if (IsCutting)
            {
                // The string from the blade to the pointer.
                var pointer = new Vector3(_pointer.x, point.y, _pointer.z);
                view.Overlay.Line(point, pointer, new Color(1.0f, 0.85f, 0.2f, 0.6f));
                view.Overlay.Cross(pointer, 0.002f, new Color(1.0f, 0.85f, 0.2f, 0.9f));
            }
        }
    }
}
