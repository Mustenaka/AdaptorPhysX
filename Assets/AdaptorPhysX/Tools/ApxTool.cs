using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// An instrument driven by the pointer (scalpel, forceps, measuring). <see cref="ApxToolbox"/>
    /// activates one tool at a time and passes it the pointer each frame.
    /// </summary>
    public abstract class ApxTool : MonoBehaviour
    {
        public abstract string DisplayName { get; }
        /// <summary>Key and mouse bindings, one per line, for the help panel.</summary>
        public abstract string Help { get; }
        /// <summary>State lines for the panel.</summary>
        public virtual string Status => "";
        public bool IsActive { get; private set; }

        public void SetActive(bool active)
        {
            if (active == IsActive)
            {
                return;
            }
            IsActive = active;
            if (active)
            {
                OnActivate();
            }
            else
            {
                OnDeactivate();
            }
        }

        protected virtual void OnActivate()
        {
        }

        protected virtual void OnDeactivate()
        {
        }

        /// <summary>Handles the pointer for one frame.</summary>
        public abstract void Tick(in ApxToolInput input);

        /// <summary>Tool keys, read from the input manager while the tool is active.</summary>
        public virtual void HandleKeys(float deltaTime, bool shift)
        {
        }

        /// <summary>Whether the tool uses the wheel this frame (the camera then does not zoom).</summary>
        public virtual bool WantsWheel(in ApxToolInput input) => false;

        /// <summary>Adds the tool's reference lines (blade, grab, measurement) for this frame.</summary>
        public virtual void Draw(ApxDebugView view)
        {
        }

        protected static bool Key(KeyCode key)
        {
#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKey(key);
#else
            return false;
#endif
        }

        protected static bool KeyDown(KeyCode key)
        {
#if ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKeyDown(key);
#else
            return false;
#endif
        }
    }

    /// <summary>Picking on the active soft bodies.</summary>
    public static class ApxToolUtil
    {
        /// <summary>
        /// The real node nearest to the ray origin among the nodes within a cell of the ray, over all
        /// active bodies: the tissue under the pointer.
        /// </summary>
        public static bool PickSurface(Ray ray, out Vector3 point, out ApxSoftBody body, out int node)
        {
            point = default;
            body = null;
            node = -1;
            float best = float.MaxValue;
            foreach (ApxSoftBody candidate in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = candidate.Simulation;
                if (sim == null)
                {
                    continue;
                }
                Bounds bounds = candidate.WorldBounds;
                bounds.Expand(2.0f * sim.CellSize);
                if (!bounds.IntersectRay(ray))
                {
                    continue;
                }
                int n = sim.PickNode(ray.origin, ray.direction, sim.CellSize);
                if (n < 0)
                {
                    continue;
                }
                Vector3 p = sim.Positions[n];
                float along = Vector3.Dot(p - ray.origin, ray.direction);
                if (along < best)
                {
                    best = along;
                    point = p;
                    body = candidate;
                    node = n;
                }
            }
            return body != null;
        }

        /// <summary>
        /// Where the ray crosses the height of the top of a body, when that point lies within
        /// <paramref name="margin"/> of the body horizontally: a stroke that starts beside the tissue.
        /// </summary>
        public static bool PickBeside(Ray ray, float margin, out Vector3 point, out ApxSoftBody body)
        {
            point = default;
            body = null;
            float best = float.MaxValue;
            foreach (ApxSoftBody candidate in ApxSoftBody.Active)
            {
                if (candidate.Simulation == null)
                {
                    continue;
                }
                Bounds bounds = candidate.WorldBounds;
                if (!new Plane(Vector3.up, bounds.max).Raycast(ray, out float enter) || enter >= best)
                {
                    continue;
                }
                Vector3 p = ray.GetPoint(enter);
                if (p.x >= bounds.min.x - margin && p.x <= bounds.max.x + margin &&
                    p.z >= bounds.min.z - margin && p.z <= bounds.max.z + margin)
                {
                    best = enter;
                    point = p;
                    body = candidate;
                }
            }
            return body != null;
        }

        /// <summary>Height of the topmost real node of <paramref name="body"/> within a cell of the vertical through <paramref name="point"/>.</summary>
        public static bool SurfaceHeight(ApxSoftBody body, Vector3 point, out float height)
        {
            height = point.y;
            SoftBodySimulation sim = body != null ? body.Simulation : null;
            if (sim == null)
            {
                return false;
            }
            Bounds bounds = body.WorldBounds;
            int node = sim.PickNode(new Vector3(point.x, bounds.max.y + sim.CellSize, point.z), Vector3.down, sim.CellSize);
            if (node < 0)
            {
                return false;
            }
            height = sim.Positions[node].y;
            return true;
        }
    }
}
