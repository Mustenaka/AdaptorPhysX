using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Scene-view style camera for inspecting soft bodies. Right drag (or Alt + left drag) orbits
    /// about the pivot, middle drag (or Shift + right drag) pans, the wheel zooms toward the point
    /// under the pointer, F frames the body under the pointer (all bodies when there is none), Home
    /// returns to the start view, keypad 1 / 3 / 7 look from the front, the right and the top and
    /// keypad 5 restores the start angles.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [DefaultExecutionOrder(-90)]
    public sealed class ApxCameraRig : MonoBehaviour, IApxDebugDrawer
    {
        [Tooltip("The view the camera approaches; the input changes it.")]
        public ApxOrbit view = new ApxOrbit { Pivot = new Vector3(0.0f, 0.03f, 0.0f), Pitch = 38.0f, Distance = 0.55f };
        [Tooltip("Degrees per pixel of pointer motion while orbiting.")]
        public float orbitSpeed = 0.3f;
        [Tooltip("Distance factor per wheel step.")]
        [Range(0.02f, 0.5f)] public float zoomStep = 0.12f;
        public float minDistance = 0.02f;
        public float maxDistance = 3.0f;
        [Tooltip("Time constant for approaching the view, seconds (0: immediate).")]
        [Min(0.0f)] public float smoothTime = 0.05f;
        [Tooltip("Set each frame by a tool that uses the wheel; the camera then does not zoom.")]
        public bool blockWheel;
        [Tooltip("Presses that start in this rectangle (GUI coordinates, e.g. a panel) do not move the camera.")]
        public Rect blockedRect;

        public Camera Camera { get; private set; }
        /// <summary>The view currently shown (approaching <see cref="view"/>).</summary>
        public ApxOrbit Current => _current;
        public ApxOrbit Home => _home;
        /// <summary>Orbiting or panning with a held button.</summary>
        public bool IsNavigating => _drag != Drag.None;

        private enum Drag
        {
            None,
            Orbit,
            Pan,
        }

        private ApxOrbit _current;
        private ApxOrbit _home;
        private Drag _drag;
        private Vector2 _lastPointer;
        private float _pivotShownUntil;

        private void Awake()
        {
            Camera = GetComponent<Camera>();
            _home = view;
            _current = view;
            Apply();
        }

        private void OnEnable() => ApxDebugView.Drawers.Add(this);

        private void OnDisable() => ApxDebugView.Drawers.Remove(this);

        private void Update()
        {
#if ENABLE_LEGACY_INPUT_MANAGER
            Vector2 pointer = Input.mousePosition;
            Vector2 delta = pointer - _lastPointer;
            _lastPointer = pointer;
            bool alt = Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt);
            bool shift = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            bool overPanel = blockedRect.Contains(new Vector2(pointer.x, Screen.height - pointer.y));
            if (_drag == Drag.None && !overPanel)
            {
                if (Input.GetMouseButtonDown(2) || (shift && Input.GetMouseButtonDown(1)))
                {
                    _drag = Drag.Pan;
                }
                else if (Input.GetMouseButtonDown(1) || (alt && Input.GetMouseButtonDown(0)))
                {
                    _drag = Drag.Orbit;
                }
            }
            else if (!Input.GetMouseButton(0) && !Input.GetMouseButton(1) && !Input.GetMouseButton(2))
            {
                _drag = Drag.None;
            }
            if (_drag == Drag.Orbit)
            {
                view.Orbit(delta.x * orbitSpeed, -delta.y * orbitSpeed);
            }
            else if (_drag == Drag.Pan)
            {
                view.Pan(delta, Camera.fieldOfView, Screen.height);
            }
            if (_drag != Drag.None)
            {
                _pivotShownUntil = Time.unscaledTime + 0.5f;
            }
            float scroll = Input.mouseScrollDelta.y;
            if (scroll != 0.0f && !blockWheel && !overPanel)
            {
                ZoomAt(Camera.ScreenPointToRay(pointer), scroll);
            }
            if (Input.GetKeyDown(KeyCode.F))
            {
                FrameAt(Camera.ScreenPointToRay(pointer));
            }
            if (Input.GetKeyDown(KeyCode.Home))
            {
                ResetView();
            }
            if (Input.GetKeyDown(KeyCode.Keypad1))
            {
                SetAngles(0.0f, 0.0f);
            }
            if (Input.GetKeyDown(KeyCode.Keypad3))
            {
                SetAngles(-90.0f, 0.0f);
            }
            if (Input.GetKeyDown(KeyCode.Keypad7))
            {
                SetAngles(0.0f, 89.0f);
            }
            if (Input.GetKeyDown(KeyCode.Keypad5))
            {
                SetAngles(_home.Yaw, _home.Pitch);
            }
#endif
            blockWheel = false;
        }

        private void LateUpdate()
        {
            float t = smoothTime > 0.0f ? 1.0f - Mathf.Exp(-Time.unscaledDeltaTime / smoothTime) : 1.0f;
            _current = ApxOrbit.Lerp(_current, view, t);
            Apply();
        }

        private void Apply() => transform.SetPositionAndRotation(_current.Position, _current.Rotation);

        /// <summary>Zooms by <paramref name="steps"/> wheel steps toward the tissue (or pivot plane) under the ray.</summary>
        public void ZoomAt(Ray ray, float steps)
        {
            Vector3 target = view.Pivot;
            if (ApxToolUtil.PickSurface(ray, out Vector3 hit, out _, out _))
            {
                target = hit;
            }
            else if (new Plane(view.Rotation * Vector3.forward, view.Pivot).Raycast(ray, out float enter))
            {
                target = ray.GetPoint(enter);
            }
            view.Zoom(Mathf.Pow(1.0f - zoomStep, steps), target, minDistance, maxDistance);
        }

        /// <summary>Frames the body under the ray around the picked point, or all bodies when none is hit.</summary>
        public void FrameAt(Ray ray)
        {
            if (ApxToolUtil.PickSurface(ray, out Vector3 hit, out ApxSoftBody body, out _))
            {
                Frame(body.WorldBounds, hit);
            }
            else
            {
                FrameAll();
            }
        }

        /// <summary>Frames all active bodies.</summary>
        public void FrameAll()
        {
            bool any = false;
            var all = new Bounds();
            foreach (ApxSoftBody b in ApxSoftBody.Active)
            {
                if (b.Simulation == null)
                {
                    continue;
                }
                if (any)
                {
                    all.Encapsulate(b.WorldBounds);
                }
                else
                {
                    all = b.WorldBounds;
                    any = true;
                }
            }
            if (any)
            {
                Frame(all, all.center);
            }
        }

        /// <summary>Looks at <paramref name="pivot"/> from a distance at which <paramref name="bounds"/> fits the view.</summary>
        public void Frame(Bounds bounds, Vector3 pivot)
        {
            float radius = Mathf.Max(bounds.extents.magnitude, 0.005f);
            float half = 0.5f * Camera.fieldOfView * Mathf.Deg2Rad;
            view.Pivot = pivot;
            view.Distance = Mathf.Clamp(1.1f * radius / Mathf.Sin(half), minDistance, maxDistance);
        }

        public void SetAngles(float yaw, float pitch)
        {
            view.Yaw = yaw;
            view.Pitch = pitch;
        }

        public void ResetView() => view = _home;

        /// <summary>Jumps to <see cref="view"/> without smoothing.</summary>
        public void Snap()
        {
            _current = view;
            Apply();
        }

        public void DrawDebug(ApxDebugView debug)
        {
            if (Time.unscaledTime < _pivotShownUntil)
            {
                debug.Overlay.Cross(view.Pivot, 0.02f * view.Distance, new Color(1.0f, 1.0f, 1.0f, 0.8f));
            }
        }
    }
}
