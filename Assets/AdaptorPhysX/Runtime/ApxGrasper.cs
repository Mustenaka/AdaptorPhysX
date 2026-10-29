using System.Collections.Generic;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// A two-jaw grasper (TODO MED-10). Two box jaws lie along the local z axis and face each other across the local y axis,
    /// <see cref="gap"/> apart at their inner faces. The jaws are colliders: they press the tissue between them like any
    /// instrument, with the grasper's friction. Once the gap is at most <see cref="graspGap"/> and the jaws have stopped
    /// closing, the tissue between the jaws is held to the grasper's transform by a dynamic attachment (TODO ECO-13): material
    /// points a half cell apart on the plane midway between the jaws, which the squeeze of the jaws does not move, so the
    /// held tissue follows the grasper's motion and rotation. The jaws can hold as much as friction gives, 2 μ N, N the force
    /// with which the tissue pushes them apart (HAP-1); each step this is shared among the held points as their break force,
    /// and a point pulled harder lets go. When all have let go the tissue has slipped out, and the grasper grasps again only
    /// after the jaws opened beyond <see cref="graspGap"/>. Opening the jaws releases the tissue.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public sealed class ApxGrasper : MonoBehaviour
    {
        [Tooltip("Distance between the inner faces of the jaws, metres.")]
        [Min(0.0f)] public float gap = 0.02f;
        [Tooltip("The jaws hold the tissue between them while the gap is at most this, metres.")]
        [Min(0.0f)] public float graspGap = 0.008f;
        [Tooltip("Size of each jaw: width (x), thickness (y) and length (z), metres.")]
        public Vector3 jawSize = new Vector3(0.006f, 0.002f, 0.02f);
        [Tooltip("Friction coefficient of the jaws against the tissue: of the contact and of the hold.")]
        [Min(0.0f)] public float friction = 0.6f;
        [Tooltip("Compliance of the hold of each point, m/N (0 rigid).")]
        [Min(0.0f)] public float compliance;

        /// <summary>The upper jaw (local +y) and the lower jaw.</summary>
        public BoxCollider UpperJaw { get; private set; }
        public BoxCollider LowerJaw { get; private set; }
        /// <summary>The body held, or null.</summary>
        public ApxSoftBody Held { get; private set; }
        /// <summary>Points still held.</summary>
        public int HeldPoints => Held != null ? Held.HeldPoints(_attachment) : 0;
        /// <summary>The tissue slipped out of the closed jaws; reset when they open.</summary>
        public bool Slipped { get; private set; }
        /// <summary>Force with which the tissue pushed the jaws apart in the last step, N.</summary>
        public float Squeeze { get; private set; }
        /// <summary>Force the jaws can hold by friction, 2 μ N.</summary>
        public float GripCapacity => 2.0f * friction * Squeeze;

        private int _attachment = -1;
        private readonly List<int> _points = new List<int>();
        private float _lastGap = float.PositiveInfinity;

        private void Awake() => CreateJaws();

        private void OnDisable() => Release();

        private void FixedUpdate() => Step();

        /// <summary>
        /// Places the jaws at the current gap and updates the hold: grasps when the jaws have closed, shares the grip among
        /// the held points, and releases when they open. Called every fixed step before the soft bodies step.
        /// </summary>
        public void Step()
        {
            CreateJaws();
            PlaceJaws();
            bool closed = gap <= graspGap;
            bool closing = gap < _lastGap;
            _lastGap = gap;
            if (!closed)
            {
                Release();
                Slipped = false;
                Squeeze = 0.0f;
                return;
            }
            Squeeze = MeasureSqueeze();
            if (Held == null)
            {
                // While the jaws still close, they squeeze the tissue past where a hold would keep it.
                if (!Slipped && !closing)
                {
                    Grasp();
                }
                return;
            }
            int held = HeldPoints;
            if (held == 0)
            {
                Release();
                Slipped = true;
                return;
            }
            Held.SetAttachmentBreakForce(_attachment, GripCapacity / held);
        }

        /// <summary>
        /// Force and torque the tissue exerted on the grasper in the last step (the jaws' contacts and the hold), with the
        /// torque about the grasper's position; false when nothing touched it.
        /// </summary>
        public bool TryGetWrench(out ApxWrench wrench)
        {
            // Sums start from an empty wrench at the grasper, so the torques are about its position.
            wrench = new ApxWrench { Origin = transform.position, ContactCentroid = transform.position };
            bool any = false;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                foreach (Collider jaw in new Collider[] { UpperJaw, LowerJaw })
                {
                    if (jaw != null && body.TryGetColliderWrench(jaw, out ApxWrench w))
                    {
                        wrench += w;
                        any = true;
                    }
                }
            }
            if (Held != null && Held.TryGetAttachmentWrench(_attachment, out ApxWrench hold))
            {
                wrench += hold;
                any = true;
            }
            return any;
        }

        /// <summary>Lets go of the tissue.</summary>
        public void Release()
        {
            if (Held != null)
            {
                Held.RemoveAttachment(_attachment);
                foreach (int point in _points)
                {
                    Held.RemoveMaterialPoint(point);
                }
            }
            Held = null;
            _attachment = -1;
            _points.Clear();
        }

        private void CreateJaws()
        {
            if (UpperJaw == null)
            {
                UpperJaw = CreateJaw("Upper Jaw");
            }
            if (LowerJaw == null)
            {
                LowerJaw = CreateJaw("Lower Jaw");
            }
        }

        private BoxCollider CreateJaw(string jawName)
        {
            var jaw = new GameObject(jawName);
            jaw.transform.SetParent(transform, false);
            var box = jaw.AddComponent<BoxCollider>();
            jaw.AddComponent<ApxColliderMaterial>();
            return box;
        }

        private void PlaceJaws()
        {
            float offset = 0.5f * (gap + jawSize.y);
            foreach ((BoxCollider jaw, float sign) in new[] { (UpperJaw, 1.0f), (LowerJaw, -1.0f) })
            {
                jaw.transform.localPosition = new Vector3(0.0f, sign * offset, 0.0f);
                jaw.transform.localRotation = Quaternion.identity;
                jaw.size = jawSize;
                jaw.GetComponent<ApxColliderMaterial>().friction = friction;
            }
        }

        /// <summary>The smaller of the two jaws' forces along the axis that opens them.</summary>
        private float MeasureSqueeze()
        {
            Vector3 up = transform.up;
            float upper = 0.0f, lower = 0.0f;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                if (body.TryGetColliderWrench(UpperJaw, out ApxWrench u))
                {
                    upper += Vector3.Dot(u.Force, up);
                }
                if (body.TryGetColliderWrench(LowerJaw, out ApxWrench l))
                {
                    lower -= Vector3.Dot(l.Force, up);
                }
            }
            return Mathf.Max(Mathf.Min(upper, lower), 0.0f);
        }

        /// <summary>
        /// Holds the tissue between the jaws: material points on a grid a half cell apart on the plane midway between the inner
        /// faces, in the body with the most of them.
        /// </summary>
        private void Grasp()
        {
            ApxSoftBody best = null;
            List<int> bestPoints = null;
            var region = new Bounds(transform.position, Vector3.zero);
            foreach (Vector3 corner in RegionCorners())
            {
                region.Encapsulate(corner);
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                if (body.Simulation == null || !body.WorldBounds.Intersects(region))
                {
                    continue;
                }
                var points = new List<int>();
                float spacing = 0.5f * body.Simulation.CellSize;
                int countX = Mathf.Max(1, Mathf.FloorToInt(jawSize.x / spacing));
                int countZ = Mathf.Max(1, Mathf.FloorToInt(jawSize.z / spacing));
                for (int z = 0; z < countZ; z++)
                for (int x = 0; x < countX; x++)
                {
                    var local = new Vector3(((x + 0.5f) / countX - 0.5f) * jawSize.x, 0.0f, ((z + 0.5f) / countZ - 0.5f) * jawSize.z);
                    int point = body.AddMaterialPoint(transform.TransformPoint(local));
                    if (point >= 0)
                    {
                        points.Add(point);
                    }
                }
                if (bestPoints == null || points.Count > bestPoints.Count)
                {
                    if (best != null)
                    {
                        foreach (int point in bestPoints)
                        {
                            best.RemoveMaterialPoint(point);
                        }
                    }
                    best = body;
                    bestPoints = points;
                }
                else
                {
                    foreach (int point in points)
                    {
                        body.RemoveMaterialPoint(point);
                    }
                }
            }
            if (best == null || bestPoints.Count == 0)
            {
                return;
            }
            Held = best;
            _points.AddRange(bestPoints);
            _attachment = best.AddAttachment(transform, bestPoints, compliance, GripCapacity / bestPoints.Count);
        }

        private IEnumerable<Vector3> RegionCorners()
        {
            for (int k = 0; k < 8; k++)
            {
                var local = new Vector3((k & 1) != 0 ? 0.5f : -0.5f, (k & 2) != 0 ? 0.5f : -0.5f, (k & 4) != 0 ? 0.5f : -0.5f);
                yield return transform.TransformPoint(Vector3.Scale(local, new Vector3(jawSize.x, gap, jawSize.z)));
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Held != null ? Color.yellow : Slipped ? Color.red : Color.cyan;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(jawSize.x, gap, jawSize.z));
        }
    }
}
