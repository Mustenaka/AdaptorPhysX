using APEX.Common.Collider;
using APEX.Common.Collider.Desc;
using Unity.Mathematics;

namespace APEX.Math.Graphics
{
    /// <summary>
    /// Math Library for calculating shape intersections
    /// </summary>
    public class IntersectUtil
    {
        /// <summary>
        /// Calculate the area of a triangle
        /// </summary>
        public static float GetArea(float3 p0, float3 p1, float3 p2)
        {
            var v0 = p1 - p0;
            var v1 = p2 - p0;
            return math.length(math.cross(v0, v1)) * 0.5f;
        }

        /// <summary>
        /// Check if a point is inside a sphere
        /// </summary>
        public static bool PointInside(float3 p, SphereDesc sphere)
        {
            float3 d = sphere.center - p;
            return math.dot(d, d) < sphere.radius * sphere.radius;
        }

        /// <summary>
        /// Get the closest point on the surface of a sphere to a given point
        /// </summary>
        public static bool GetClosestSurfacePoint(float3 p, SphereDesc sphere, out ContactInfo concatInfo)
        {
            concatInfo = default(ContactInfo);
            // Calculate the vector from the sphere's center to the point
            float3 c2p = p - sphere.center;
            float d2 = math.dot(c2p, c2p);
            float r2 = sphere.radius * sphere.radius;
            if (d2 < r2)
            {
                // Point is inside the sphere, return the point on the sphere's surface
                concatInfo.normal = math.normalize(c2p);
                concatInfo.position = sphere.center + concatInfo.normal * sphere.radius;
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Check if a point is inside a box
        /// </summary>
        public static bool PointInside(float3 p, BoxDesc box)
        {
            var o = box.center; //min作为原点
            p = p - o;
            float3 projOnAxis = new float3(
                math.dot(p, box.ax.xyz),
                math.dot(p, box.ay.xyz),
                math.dot(p, box.az.xyz)
            );
            float3 axisLength = new float3(box.ax.w, box.ay.w, box.az.w);
            return math.all(projOnAxis > 0) && math.all(projOnAxis < axisLength);
        }

        /// <summary>
        /// Get the closest point on the surface of a box to a given point
        /// </summary>
        public static bool GetClosestSurfacePoint(float3 p, BoxDesc box, out ContactInfo concatInfo)
        {
            var o = box.center; //min作为原点
            p = p - o;
            float3 projOnAxis = new float3(
                math.dot(p, box.ax.xyz),
                math.dot(p, box.ay.xyz),
                math.dot(p, box.az.xyz)
            );
            float3 axisLength = new float3(box.ax.w, box.ay.w, box.az.w);
            float3 side = math.step(axisLength * 0.5f, projOnAxis); // >0.5 => 1 | <0.5 => 0
            float3 signedDist = (1 - side * 2) * (projOnAxis - side * axisLength);
            bool inside = math.all(signedDist > 0);
            if (inside)
            {
                var dst = signedDist.x;
                var axis = box.ax;
                var sideFlag = side.x;
                var axisIndex = 0;
                if (signedDist.y < dst)
                {
                    dst = signedDist.y;
                    axisIndex = 1;
                    sideFlag = side.y;
                    axis = box.ay;
                }

                if (signedDist.z < dst)
                {
                    dst = signedDist.z;
                    sideFlag = side.z;
                    axisIndex = 2;
                    axis = box.az;
                }

                concatInfo = new ContactInfo();
                concatInfo.normal = sideFlag == 1 ? axis.xyz : -axis.xyz;
                var offset = (projOnAxis[axisIndex] - sideFlag * axis.w);
                concatInfo.position = o + p - axis.xyz * offset;
                return true;
            }
            else
            {
                concatInfo = default;
                return false;
            }
        }

        /// <summary>
        /// Check if a point is inside a capsule
        /// </summary>
        public static bool PointInside(float3 p, CapsuleDesc capsule)
        {
            var o = capsule.center; //c0作为原点
            var radius = capsule.radius;
            var axis = capsule.axis;
            p = p - o;
            var proj = math.dot(p, axis.xyz); //p点在轴上的投影
            if (proj < -radius || proj > axis.w + radius)
            {
                return false;
            }

            var r2 = radius * radius;
            if (proj >= 0 && proj <= axis.w)
            {
                //轴上投影在圆柱体之间
                var dist2 = math.dot(p, p) - proj * proj; //计算p到轴的垂直距离平方
                return dist2 < r2;
            }

            if (proj >= -radius && proj < 0)
            {
                //轴上投影处于原点附近
                return math.dot(p, p) < r2;
            }

            if (proj <= axis.w + radius)
            {
                //轴上投影处于另一头附近
                var v = p - (axis.xyz * axis.w);
                return math.dot(v, v) < r2;
            }

            return false;
        }

        /// <summary>
        /// Get the closest point on the surface of a capsule to a given point
        /// </summary>
        public static bool GetClosestSurfacePoint(float3 p, CapsuleDesc capsule, out ContactInfo concatInfo)
        {
            concatInfo = default(ContactInfo);
            var o = capsule.center; //c0作为原点
            var radius = capsule.radius;
            var axis = capsule.axis;
            p = p - o;
            var proj = math.dot(p, axis.xyz); //p点在轴上的投影
            if (proj < -radius || proj > axis.w + radius)
            {
                return false;
            }

            var r2 = radius * radius;
            if (proj >= 0 && proj <= axis.w)
            {
                //轴上投影在圆柱体之间
                var dist2 = math.dot(p, p) - proj * proj; //计算p到轴的垂直距离平方
                if (dist2 < r2)
                {
                    var q = axis.xyz * proj;
                    concatInfo.normal = math.normalize(p - q);
                    concatInfo.position = o + q + concatInfo.normal * radius;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            if (proj >= -radius && proj < 0)
            {
                //轴上投影处于原点附近
                var c2p = p;
                if (math.dot(c2p, c2p) < r2)
                {
                    concatInfo.normal = math.normalize(c2p);
                    concatInfo.position = o + radius * concatInfo.normal;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            if (proj <= axis.w + radius)
            {
                //轴上投影处于另一头附近
                var c = (axis.xyz * axis.w);
                var c2p = p - c;
                if (math.dot(c2p, c2p) < r2)
                {
                    concatInfo.normal = math.normalize(c2p);
                    concatInfo.position = o + c + radius * concatInfo.normal;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            return false;
        }

        /// <summary>
        /// Check if a point is inside a mesh
        /// </summary>
        public static bool PointInside(float3 p, MeshDesc mesh)
        {
            // Iterate over all triangles in the mesh
            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                // Get the vertices of the current triangle
                float3 v0 = mesh.vertices[mesh.triangles[i]];
                float3 v1 = mesh.vertices[mesh.triangles[i + 1]];
                float3 v2 = mesh.vertices[mesh.triangles[i + 2]];

                // Calculate the normal of the triangle
                // float3 normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
                float3 normal = math.normalize(math.cross(v1 - v0, v2 - v0));

                // Calculate the distance from the point to the plane of the triangle
                float distanceToPlane = math.dot(normal, p - v0);

                // If the point is on the opposite side of the plane compared to the normal,
                // it is outside the mesh
                if (distanceToPlane > 0)
                {
                    return false;
                }
            }

            // If the point is on the same side of the plane for all triangles,
            // it is inside the mesh
            return true;
        }

        /// <summary>
        /// Check if a point is on the surface of a mesh
        /// </summary>
        public static bool GetClosestSurfacePoint(float3 p, MeshDesc mesh, out ContactInfo concatInfo)
        {
            concatInfo = default(ContactInfo);
            
            // Iterate over all triangles in the mesh
            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                // Get the vertices of the current triangle
                float3 v0 = mesh.vertices[mesh.triangles[i]];
                float3 v1 = mesh.vertices[mesh.triangles[i + 1]];
                float3 v2 = mesh.vertices[mesh.triangles[i + 2]];

                // Calculate the normal of the triangle
                float3 normal = math.normalize(math.cross(v1 - v0, v2 - v0));

                // Calculate the distance from the point to the plane of the triangle
                float distanceToPlane = math.dot(normal, p - v0);

                // If the point is very close to the plane (within epsilon),
                // it is considered to be on the surface of the mesh
                if (math.abs(distanceToPlane) < math.EPSILON)
                {
                    // Check if the point is inside the triangle
                    if (PointInsideTriangle(p, v0, v1, v2))
                    {
                        concatInfo.position = p;
                        concatInfo.normal = normal;
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Check if a point is inside a triangle
        /// </summary>
        private static bool PointInsideTriangle(float3 p, float3 v0, float3 v1, float3 v2)
        {
            // Calculate the barycentric coordinates of the point with respect to the triangle
            float3 e0 = v1 - v0;
            float3 e1 = v2 - v0;
            float3 e2 = p - v0;
            float d00 = math.dot(e0, e0);
            float d01 = math.dot(e0, e1);
            float d11 = math.dot(e1, e1);
            float d20 = math.dot(e2, e0);
            float d21 = math.dot(e2, e1);
            float denom = d00 * d11 - d01 * d01;
            float alpha = (d11 * d20 - d01 * d21) / denom;
            float beta = (d00 * d21 - d01 * d20) / denom;
            float gamma = 1.0f - alpha - beta;

            // Check if the barycentric coordinates are within the range [0, 1]
            return alpha >= 0 && beta >= 0 && gamma >= 0;
        }
    }
}