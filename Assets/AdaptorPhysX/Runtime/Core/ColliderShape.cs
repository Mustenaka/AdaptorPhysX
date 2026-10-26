using Unity.Collections;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    public enum ColliderShapeType
    {
        Sphere = 0,
        Capsule = 1,
        Box = 2,
        /// <summary>Sampled signed distance field (MeshCollider or <see cref="ApxDistanceField"/>).</summary>
        DistanceField = 3,
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
        /// <summary>
        /// Adhesion: nodes within StickDistance outside the contact distance are pulled towards the
        /// surface by this fraction of the gap per substep (0 = none).
        /// </summary>
        public float Stickiness;
        public float StickDistance;
        /// <summary>Distance field: start of its values in the field buffer, grid points per axis, grid origin and spacing (local units).</summary>
        public int FieldOffset;
        public int3 FieldDims;
        public float3 FieldOrigin;
        public float FieldCellSize;
        /// <summary>Distance field: lossy scale of the collider transform (local units to metres per axis).</summary>
        public float3 FieldScale;
        /// <summary>
        /// 1 + index of the dynamic rigid body the collider belongs to (two-way coupling), 0 otherwise.
        /// The step then moves the pose substep by substep (<see cref="RigidCouplingJob"/>): A0/R0
        /// hold the pose at the end of the previous substep and A1/R1 at the end of the current one.
        /// </summary>
        public int DynamicBody;

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

        /// <summary>
        /// Distance field of <paramref name="field"/>, whose values start at <paramref name="offset"/>
        /// in the field buffer passed to <see cref="Distance(float3, float, NativeArray{float}, out float3)"/>.
        /// </summary>
        public static ColliderShape DistanceField(float3 position, quaternion rotation, float3 scale, int offset,
            DistanceField field, float friction) =>
            new ColliderShape
            {
                Type = ColliderShapeType.DistanceField,
                Friction = friction,
                A0 = position, A1 = position,
                R0 = rotation, R1 = rotation,
                FieldOffset = offset,
                FieldDims = field.Dims,
                FieldOrigin = field.Origin,
                FieldCellSize = field.CellSize,
                FieldScale = scale,
            };

        /// <summary>
        /// Signed distance and outward normal at <paramref name="p"/> for the pose at time <paramref name="t"/>,
        /// for every shape type. Distance fields are sampled in local space and converted to metres with
        /// the scale along the gradient direction, which is exact for uniform scale. Only the direction
        /// of the gradient is used: near the medial axis (the centre line of a tube) the interpolated
        /// gradient is short, and dividing by its length multiplied the depth.
        /// </summary>
        public float Distance(float3 p, float t, NativeArray<float> fields, out float3 normal)
        {
            if (Type != ColliderShapeType.DistanceField)
            {
                return Distance(p, t, out normal);
            }
            float3 a = math.lerp(A0, A1, t);
            quaternion r = math.all(R0.value == R1.value) ? R0 : math.nlerp(R0, R1, t);
            float3 local = math.mul(math.conjugate(r), p - a) / FieldScale;
            float d = AdaptorPhysX.DistanceField.Sample(fields, FieldOffset, FieldDims, FieldOrigin, FieldCellSize, local,
                out float3 gradient);
            float3 scaled = math.normalizesafe(gradient, new float3(0.0f, 1.0f, 0.0f)) / FieldScale;
            float length = math.length(scaled);
            normal = math.mul(r, scaled / length);
            return d / length;
        }

        /// <summary>Signed distance and outward normal at <paramref name="p"/> for the pose at time <paramref name="t"/> in [0,1] (primitive shapes).</summary>
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
                    // Static boxes skip the interpolation; moving ones use nlerp (small per-frame
                    // rotations) and the conjugate as inverse of the unit quaternion.
                    quaternion r = math.all(R0.value == R1.value) ? R0 : math.nlerp(R0, R1, t);
                    float3 local = math.mul(math.conjugate(r), p - a);
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
