using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Pointer state for one frame as the tools see it. <see cref="ApxToolbox"/> fills it from the
    /// input manager; tests fill it directly.
    /// </summary>
    public struct ApxToolInput
    {
        /// <summary>Ray from the camera through the pointer.</summary>
        public Ray Ray;
        /// <summary>Camera forward, for planes that face the viewer.</summary>
        public Vector3 ViewForward;
        /// <summary>Pointer position in pixels (origin bottom left).</summary>
        public Vector2 Position;
        /// <summary>Primary (left) button pressed this frame, held, released this frame.</summary>
        public bool Down;
        public bool Held;
        public bool Up;
        /// <summary>Wheel steps this frame, positive away from the user.</summary>
        public float Scroll;
        public bool Shift;
        public bool Ctrl;
        /// <summary>Real seconds since the previous frame (not scaled by a pause).</summary>
        public float DeltaTime;

        /// <summary>Input for a pointer ray with no buttons, e.g. for hovering in tests.</summary>
        public static ApxToolInput Hover(Ray ray, float deltaTime) => new ApxToolInput
        {
            Ray = ray,
            ViewForward = ray.direction,
            DeltaTime = deltaTime,
        };
    }
}
