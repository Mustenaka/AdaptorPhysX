using Unity.Mathematics;

namespace AdaptorPhysX
{
    public enum ColliderShapeType
    {
        Sphere = 0,
        Capsule = 1,
        Box = 2,
    }

    /// <summary>
    /// A kinematic collision proxy with its pose at the start (0) and end (1) of the
    /// frame; substeps interpolate between the two so fast tools do not tunnel.
    /// </summary>
    public struct ColliderShape
    {
        public ColliderShapeType Type;
        public float Radius;
        public float3 HalfExtents;
        public float Friction;
        /// <summary>Sphere/box centre or capsule first end point.</summary>
        public float3 A0, A1;
        /// <summary>Capsule second end point.</summary>
        public float3 B0, B1;
        public quaternion R0, R1;

        public static ColliderShape Sphere(float3 center, float radius, float friction) =>
            new ColliderShape
            {
                Type = ColliderShapeType.Sphere,
                Radius = radius,
                Friction = friction,
                A0 = center, A1 = center,
                R0 = quaternion.identity, R1 = quaternion.identity,
            };

        public static ColliderShape Box(float3 center, quaternion rotation, float3 halfExtents, float friction) =>
            new ColliderShape
            {
                Type = ColliderShapeType.Box,
                HalfExtents = halfExtents,
                Friction = friction,
                A0 = center, A1 = center,
                R0 = rotation, R1 = rotation,
            };

        public static ColliderShape Capsule(float3 a, float3 b, float radius, float friction) =>
            new ColliderShape
            {
                Type = ColliderShapeType.Capsule,
                Radius = radius,
                Friction = friction,
                A0 = a, A1 = a, B0 = b, B1 = b,
                R0 = quaternion.identity, R1 = quaternion.identity,
            };

        /// <summary>Signed distance and outward normal at <paramref name="p"/> for the pose at time <paramref name="t"/> in [0,1].</summary>
        public float Distance(float3 p, float t, out float3 normal)
        {
            float3 a = math.lerp(A0, A1, t);
            switch (Type)
            {
                case ColliderShapeType.Sphere:
                    return SphereDistance(p, a, Radius, out normal);
                case ColliderShapeType.Capsule:
                {
                    float3 b = math.lerp(B0, B1, t);
                    float3 ab = b - a;
                    float s = math.saturate(math.dot(p - a, ab) / math.max(math.dot(ab, ab), 1e-12f));
                    return SphereDistance(p, a + s * ab, Radius, out normal);
                }
                default:
                {
                    quaternion r = math.normalize(math.slerp(R0, R1, t));
                    float3 local = math.mul(math.inverse(r), p - a);
                    float3 q = math.abs(local) - HalfExtents;
                    if (math.cmax(q) > 0.0f)
                    {
                        float3 outside = local - math.clamp(local, -HalfExtents, HalfExtents);
                        float d = math.length(outside);
                        normal = math.mul(r, outside / math.max(d, 1e-12f));
                        return d;
                    }
                    // Inside: leave through the face with the smallest penetration.
                    int axis = q.x > q.y ? (q.x > q.z ? 0 : 2) : (q.y > q.z ? 1 : 2);
                    float3 n = float3.zero;
                    n[axis] = local[axis] >= 0.0f ? 1.0f : -1.0f;
                    normal = math.mul(r, n);
                    return q[axis];
                }
            }
        }

        /// <summary>Displacement of the shape's reference point between substep times.</summary>
        public float3 Displacement(float t0, float t1) => math.lerp(A0, A1, t1) - math.lerp(A0, A1, t0);

        private static float SphereDistance(float3 p, float3 c, float r, out float3 normal)
        {
            float3 d = p - c;
            float len = math.length(d);
            normal = len > 1e-12f ? d / len : new float3(0.0f, 1.0f, 0.0f);
            return len - r;
        }
    }
}
