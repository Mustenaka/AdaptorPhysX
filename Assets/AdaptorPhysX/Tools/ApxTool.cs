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

        /// <summary>
        /// Height of the surface of <paramref name="body"/> at <paramref name="point"/> along <paramref name="axis"/>: the coordinate
        /// along the axis of the outermost real node within two cells of the line through the point. Within two cells the walls
        /// of an incision that has opened (up to about four cells wide) hold the height: with one cell, the floor of the
        /// incision was the surface, and a blade drawn along it again went its depth below the floor, through a 16 mm flap
        /// (TODO BUG-19).
        /// </summary>
        public static bool SurfaceHeight(ApxSoftBody body, Vector3 point, Vector3 axis, out float height)
        {
            height = Vector3.Dot(point, axis);
            SoftBodySimulation sim = body != null ? body.Simulation : null;
            if (sim == null)
            {
                return false;
            }
            Bounds bounds = body.WorldBounds;
            // From outside the body along the axis.
            float reach = Vector3.Dot(bounds.extents, new Vector3(Mathf.Abs(axis.x), Mathf.Abs(axis.y), Mathf.Abs(axis.z))) +
                          Vector3.Dot(bounds.center - point, axis) + sim.CellSize;
            int node = sim.PickNode(point + axis * reach, -axis, 2.0f * sim.CellSize);
            if (node < 0)
            {
                return false;
            }
            height = Vector3.Dot(sim.Positions[node], axis);
            return true;
        }

        /// <summary>Height of the top of <paramref name="body"/> at <paramref name="point"/> (<see cref="SurfaceHeight(ApxSoftBody, Vector3, Vector3, out float)"/> along up).</summary>
        public static bool SurfaceHeight(ApxSoftBody body, Vector3 point, out float height) =>
            SurfaceHeight(body, point, Vector3.up, out height);

        /// <summary>
        /// Outward normal of the surface at real node <paramref name="node"/>: away from the centre of the real nodes within 1.5
        /// cells of it.
        /// </summary>
        public static Vector3 SurfaceNormal(ApxSoftBody body, int node)
        {
            SoftBodySimulation sim = body.Simulation;
            Vector3 p = sim.Positions[node];
            float radius = 1.5f * sim.CellSize;
            Vector3 sum = Vector3.zero;
            int count = 0;
            for (int n = 0; n < sim.NodeCount; n++)
            {
                if (sim.NodeReal[n] == 0)
                {
                    continue;
                }
                Vector3 q = sim.Positions[n];
                if ((q - p).sqrMagnitude <= radius * radius)
                {
                    sum += q;
                    count++;
                }
            }
            return count > 0 ? (p - sum / count).normalized : Vector3.up;
        }

        /// <summary>
        /// Reference axis of a stroke started at real node <paramref name="node"/>: world up on the top of the tissue (surface
        /// normal within 45 degrees of up), else the horizontal part of the normal: a side of the tissue.
        /// </summary>
        public static Vector3 StrokeAxis(ApxSoftBody body, int node)
        {
            Vector3 normal = SurfaceNormal(body, node);
            Vector3 horizontal = new Vector3(normal.x, 0.0f, normal.z);
            return normal.y >= 0.7071f || horizontal.sqrMagnitude < 1e-4f ? Vector3.up : horizontal.normalized;
        }
    }
}
