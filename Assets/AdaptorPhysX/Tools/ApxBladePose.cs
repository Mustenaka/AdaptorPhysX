using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Orientation and depth of a scalpel blade held at a point of the tissue surface. The edge runs
    /// from the heel outside the surface to the tip inside it. <see cref="heading"/> turns the cutting
    /// direction about the surface's reference axis (<see cref="normal"/>; world up on the top of the
    /// tissue), <see cref="tilt"/> leans the blade sideways about the cutting direction (a bevelled cut),
    /// <see cref="rake"/> leans the edge forward within the blade plane. Depth and clearance are measured
    /// along the reference axis, so a leaning blade still reaches the same depth.
    /// </summary>
    [System.Serializable]
    public struct ApxBladePose
    {
        /// <summary>Largest tilt or rake, degrees.</summary>
        public const float MaxLean = 70.0f;

        [Tooltip("Cutting direction, degrees about the world up axis (0: +z, 90: +x).")]
        public float heading;
        [Tooltip("Sideways lean about the cutting direction, degrees (positive: tip to the right).")]
        public float tilt;
        [Tooltip("Lean of the edge within the blade plane, degrees (positive: tip ahead).")]
        public float rake;
        [Tooltip("Tip below the surface point, metres (vertical).")]
        public float depth;
        [Tooltip("Heel above the surface point, metres (vertical).")]
        public float clearance;
        [Tooltip("Outward normal of the surface the blade enters (a side face); zero: world up, the top of the tissue.")]
        public Vector3 normal;

        /// <summary>Whether the blade enters a surface other than the top (<see cref="normal"/> set).</summary>
        public bool OnSide => normal.sqrMagnitude > 1e-12f;

        /// <summary>Turns world up to the reference axis: identity on the top of the tissue.</summary>
        public Quaternion Frame => OnSide ? Quaternion.FromToRotation(Vector3.up, normal.normalized) : Quaternion.identity;

        /// <summary>The reference axis: outward from the surface the blade enters.</summary>
        public Vector3 Up => OnSide ? normal.normalized : Vector3.up;

        public Quaternion Rotation => OnSide ? Frame * LocalRotation : LocalRotation;

        private Quaternion LocalRotation =>
            Quaternion.Euler(0.0f, heading, 0.0f) * Quaternion.AngleAxis(tilt, Vector3.forward) * Quaternion.AngleAxis(-rake, Vector3.right);

        /// <summary>Unit vector from heel to tip.</summary>
        public Vector3 EdgeDirection => Rotation * Vector3.down;

        /// <summary>Cutting direction, across the reference axis (horizontal on the top of the tissue).</summary>
        public Vector3 Forward => OnSide ? Frame * (Quaternion.Euler(0.0f, heading, 0.0f) * Vector3.forward)
            : Quaternion.Euler(0.0f, heading, 0.0f) * Vector3.forward;

        /// <summary>Edge length for the current depth, clearance and lean.</summary>
        public float EdgeLength => (depth + clearance) / VerticalCosine;

        private float VerticalCosine => Mathf.Max(OnSide ? -Vector3.Dot(EdgeDirection, Up) : -EdgeDirection.y, 0.1f);

        /// <summary>Heel and tip of the edge through <paramref name="surfacePoint"/>.</summary>
        public void GetEdge(Vector3 surfacePoint, out Vector3 heel, out Vector3 tip)
        {
            Vector3 d = EdgeDirection;
            float c = VerticalCosine;
            tip = surfacePoint + d * (depth / c);
            heel = surfacePoint - d * (clearance / c);
        }

        /// <summary>Keeps the angles and depth in range.</summary>
        public void Clamp(float maxDepth)
        {
            heading = Mathf.Repeat(heading, 360.0f);
            tilt = Mathf.Clamp(tilt, -MaxLean, MaxLean);
            rake = Mathf.Clamp(rake, -MaxLean, MaxLean);
            depth = Mathf.Clamp(depth, 0.0005f, maxDepth);
            clearance = Mathf.Max(clearance, 0.0f);
        }

        /// <summary>Heading (degrees) of a horizontal direction.</summary>
        public static float HeadingOf(Vector3 direction) => Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

        /// <summary>Heading (degrees) of a direction across the reference axis, as <see cref="heading"/> counts it.</summary>
        public float HeadingIn(Vector3 direction) => HeadingOf(OnSide ? Quaternion.Inverse(Frame) * direction : direction);

        /// <summary>
        /// Draws a blade outline: the edge, the back of the blade behind it (against
        /// <paramref name="forward"/>) and the handle above the heel.
        /// </summary>
        public static void Draw(ApxDebugLines lines, Vector3 heel, Vector3 tip, Vector3 forward, Color color)
        {
            Vector3 edge = tip - heel;
            float length = edge.magnitude;
            if (length < 1e-6f)
            {
                return;
            }
            Vector3 d = edge / length;
            Vector3 back = -Vector3.ProjectOnPlane(forward, d).normalized;
            float width = Mathf.Clamp(0.25f * length, 0.003f, 0.008f);
            lines.Line(heel, tip, color);
            var faint = new Color(color.r, color.g, color.b, 0.6f * color.a);
            lines.Line(heel, heel + back * width, faint);
            lines.Line(heel + back * width, tip + back * (0.3f * width), faint);
            lines.Line(tip + back * (0.3f * width), tip, faint);
            lines.Line(heel + back * (0.5f * width), heel + back * (0.5f * width) - d * 0.04f, faint);
        }
    }
}
