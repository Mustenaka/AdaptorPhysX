using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Orbit view: the camera looks at <see cref="Pivot"/> from <see cref="Distance"/>, turned by
    /// <see cref="Yaw"/> about the world up axis and <see cref="Pitch"/> down from the horizon
    /// (degrees). Pure math, shared by <see cref="ApxCameraRig"/> and the tests.
    /// </summary>
    [System.Serializable]
    public struct ApxOrbit
    {
        public Vector3 Pivot;
        public float Yaw;
        public float Pitch;
        public float Distance;

        public Quaternion Rotation => Quaternion.Euler(Pitch, Yaw, 0.0f);
        public Vector3 Position => Pivot - Rotation * Vector3.forward * Distance;

        /// <summary>World length of one pixel at the pivot, for a vertical field of view in degrees.</summary>
        public float WorldPerPixel(float fieldOfView, float screenHeight) =>
            2.0f * Distance * Mathf.Tan(0.5f * fieldOfView * Mathf.Deg2Rad) / Mathf.Max(screenHeight, 1.0f);

        public void Orbit(float yawDegrees, float pitchDegrees, float minPitch = -89.0f, float maxPitch = 89.0f)
        {
            Yaw = Mathf.Repeat(Yaw + yawDegrees, 360.0f);
            Pitch = Mathf.Clamp(Pitch + pitchDegrees, minPitch, maxPitch);
        }

        /// <summary>
        /// Moves the pivot so that the points in the plane through the pivot follow a pointer moved by
        /// <paramref name="pixels"/>.
        /// </summary>
        public void Pan(Vector2 pixels, float fieldOfView, float screenHeight)
        {
            float scale = WorldPerPixel(fieldOfView, screenHeight);
            Quaternion r = Rotation;
            Pivot -= (r * Vector3.right * pixels.x + r * Vector3.up * pixels.y) * scale;
        }

        /// <summary>
        /// Scales the distance by <paramref name="factor"/> (clamped) and moves the pivot toward
        /// <paramref name="target"/> in proportion: the camera moves along the line to the target, so
        /// the target stays under the pointer.
        /// </summary>
        public void Zoom(float factor, Vector3 target, float minDistance, float maxDistance)
        {
            float distance = Mathf.Clamp(Distance * factor, minDistance, maxDistance);
            float k = distance / Distance;
            Pivot = target + (Pivot - target) * k;
            Distance = distance;
        }

        /// <summary>Moves toward <paramref name="target"/> by <paramref name="t"/> in [0, 1] (distance in log space).</summary>
        public static ApxOrbit Lerp(ApxOrbit from, ApxOrbit target, float t) => new ApxOrbit
        {
            Pivot = Vector3.Lerp(from.Pivot, target.Pivot, t),
            Yaw = Mathf.LerpAngle(from.Yaw, target.Yaw, t),
            Pitch = Mathf.Lerp(from.Pitch, target.Pitch, t),
            Distance = Mathf.Exp(Mathf.Lerp(Mathf.Log(from.Distance), Mathf.Log(target.Distance), t)),
        };
    }
}
