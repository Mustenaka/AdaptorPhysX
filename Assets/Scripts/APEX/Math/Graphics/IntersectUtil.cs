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
        /// Checks if a line segment intersects with a sphere and returns the closest intersection point and normal.
        /// </summary>
        public static bool GetLineIntersectPoint(float3 p1, float3 p2, SphereDesc sphere, out ContactInfo contactInfo)
        {
            contactInfo = default(ContactInfo);

            // Direction of the line segment
            float3 dir = p2 - p1;

            // Calculate the vector from the sphere's center to one of the line segment's endpoints
            float3 sphereToLineStart = p1 - sphere.center;

            // Calculate the quadratic coefficients of the equation for the line
            float a = math.dot(dir, dir);
            float b = 2 * math.dot(sphereToLineStart, dir);
            float c = math.dot(sphereToLineStart, sphereToLineStart) - (sphere.radius * sphere.radius);

            // Calculate the discriminant
            float discriminant = b * b - 4 * a * c;

            // If the discriminant is negative, there are no real roots, so the line does not intersect with the sphere
            if (discriminant < 0)
            {
                return false;
            }

            // Calculate the two possible t values (the intersection points) using the quadratic formula
            float sqrtDiscriminant = math.sqrt(discriminant);
            float t1 = (-b + sqrtDiscriminant) / (2 * a);
            float t2 = (-b - sqrtDiscriminant) / (2 * a);

            // If either t value is between 0 and 1, the intersection point lies on the line segment
            if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1))
            {
                // Calculate the intersection point
                float3 intersectionPoint = p1 + math.clamp(t1, 0, 1) * dir;

                // Calculate the normal at the intersection point
                contactInfo.normal = math.normalize(intersectionPoint - sphere.center);
                contactInfo.position = intersectionPoint;

                return true;
            }

            return false;
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
        /// Checks if a line segment intersects with a box and returns the closest intersection point and normal.
        /// </summary>
        public static bool GetLineIntersectPoint(float3 p1, float3 p2, BoxDesc box, out ContactInfo contactInfo)
        {
            contactInfo = default(ContactInfo);

            // Direction of the line segment
            float3 dir = p2 - p1;

            // Calculate the inverse direction components
            float3 invDir = 1.0f / dir;

            // Calculate the distances to the two farthest planes along each axis
            float3 tMin = (box.center - math.abs(box.ax.w * 0.5f) - p1) * invDir;
            float3 tMax = (box.center + math.abs(box.ax.w * 0.5f) - p1) * invDir;

            // Get the minimum and maximum of the intersection times
            float3 tEnter = math.min(tMin, tMax);
            float3 tExit = math.max(tMin, tMax);

            // Get the largest of the minimum intersection times
            float tEnterMax = math.max(math.max(tEnter.x, tEnter.y), tEnter.z);

            // Get the smallest of the maximum intersection times
            float tExitMin = math.min(math.min(tExit.x, tExit.y), tExit.z);

            // If the largest minimum intersection time is less than or equal to the smallest maximum intersection time, there is an intersection
            if (tEnterMax <= tExitMin)
            {
                // Calculate the intersection point
                float3 intersectionPoint = p1 + tEnterMax * dir;

                // Find the normal of the intersected face
                float3 faceNormal = math.abs(tExitMin - tEnterMax) < math.EPSILON
                    ? math.abs(tEnter.x - tExit.x) < math.EPSILON ? new float3(-1, 0, 0) :
                    math.abs(tEnter.y - tExit.y) < math.EPSILON ? new float3(0, -1, 0) : new float3(0, 0, -1)
                    : math.abs(tEnterMax - tEnter.x) < math.EPSILON
                        ? new float3(1, 0, 0)
                        : math.abs(tEnterMax - tEnter.y) < math.EPSILON
                            ? new float3(0, 1, 0)
                            : new float3(0, 0, 1);

                // Set the contact information
                contactInfo.position = intersectionPoint;
                contactInfo.normal = faceNormal;

                return true;
            }

            // No intersection
            return false;
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
        /// Checks if a line segment intersects with a capsule and returns the closest intersection point and normal.
        /// </summary>
        public static bool GetLineIntersectPoint(float3 p1, float3 p2, CapsuleDesc capsule, out ContactInfo contactInfo)
        {
            contactInfo = default(ContactInfo);

            // Calculate the direction of the capsule axis
            float3 capsuleAxis = math.normalize(capsule.axis.xyz);

            // Calculate the vector from the start point of the capsule axis to p1
            float3 v1 = p1 - capsule.center;

            // Calculate the vector from the start point of the capsule axis to p2
            float3 v2 = p2 - capsule.center;

            // Calculate the projection of v1 and v2 onto the capsule axis
            float t1 = math.dot(v1, capsuleAxis);
            float t2 = math.dot(v2, capsuleAxis);

            // If both points are on the same side of the capsule axis, there's no intersection
            if ((t1 < 0 && t2 < 0) || (t1 > capsule.axis.w && t2 > capsule.axis.w))
                return false;

            // If p1 is closer to the capsule center than p2, swap them
            if (t1 > t2)
            {
                (p1, p2) = (p2, p1);
                (t1, t2) = (t2, t1);
            }

            // If both points are outside the capsule's hemispheres, intersect with the cylinder part
            if (t1 < 0 && t2 > capsule.axis.w)
            {
                // Calculate the direction of the line segment
                float3 dir = p2 - p1;

                // Calculate the inverse direction components
                float3 invDir = 1.0f / dir;

                // Calculate the distance to the nearest and farthest planes along the capsule axis
                float tEnter = (capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                               math.dot(dir, capsuleAxis);
                float tExit = (capsule.axis.w - capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                              math.dot(dir, capsuleAxis);

                // Ensure tEnter is less than tExit
                if (tEnter > tExit)
                {
                    (tEnter, tExit) = (tExit, tEnter);
                }

                // Ensure tEnter is within the valid range
                tEnter = math.max(0, tEnter);

                // Calculate the intersection point
                float3 intersectionPoint = p1 + tEnter * dir;

                // Calculate the normal at the intersection point
                float3 normal = math.normalize(intersectionPoint - (capsule.center + capsuleAxis *
                    math.clamp(math.dot(intersectionPoint - capsule.center, capsuleAxis), 0, capsule.axis.w)));

                // Set the contact information
                contactInfo.position = intersectionPoint;
                contactInfo.normal = normal;

                return true;
            }

            // If p1 is inside the capsule's hemispheres, intersect with the hemispheres
            if (t1 < 0 || t2 > capsule.axis.w)
            {
                // Calculate the direction of the line segment
                float3 dir = p2 - p1;

                // Calculate the inverse direction components
                float3 invDir = 1.0f / dir;

                // Calculate the distance to the nearest and farthest planes along the capsule axis
                float tEnter = (capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                               math.dot(dir, capsuleAxis);
                float tExit = (capsule.axis.w - capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                              math.dot(dir, capsuleAxis);

                // Ensure tEnter is less than tExit
                if (tEnter > tExit)
                {
                    (tEnter, tExit) = (tExit, tEnter);
                }

                // Ensure tEnter is within the valid range
                tEnter = math.max(0, tEnter);

                // Calculate the intersection point
                float3 intersectionPoint = p1 + tEnter * dir;

                // Calculate the normal at the intersection point
                float3 normal = math.normalize(intersectionPoint - (capsule.center + capsuleAxis *
                    math.clamp(math.dot(intersectionPoint - capsule.center, capsuleAxis), 0, capsule.axis.w)));

                // Set the contact information
                contactInfo.position = intersectionPoint;
                contactInfo.normal = normal;

                return true;
            }

            // If p1 and p2 are both inside the capsule's cylinder part, intersect with the cylinder
            {
                // Calculate the direction of the line segment
                float3 dir = p2 - p1;

                // Calculate the inverse direction components
                float3 invDir = 1.0f / dir;

                // Calculate the distance to the nearest and farthest planes along the capsule axis
                float tEnter = (capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                               math.dot(dir, capsuleAxis);
                float tExit = (capsule.axis.w - capsule.radius - math.dot(p1 - capsule.center, capsuleAxis)) /
                              math.dot(dir, capsuleAxis);

                // Ensure tEnter is less than tExit
                if (tEnter > tExit)
                {
                    (tEnter, tExit) = (tExit, tEnter);
                }

                // Ensure tEnter is within the valid range
                tEnter = math.max(0, tEnter);

                // Calculate the intersection point
                float3 intersectionPoint = p1 + tEnter * dir;

                // Calculate the normal at the intersection point
                float3 normal = math.normalize(intersectionPoint - (capsule.center + capsuleAxis *
                    math.clamp(math.dot(intersectionPoint - capsule.center, capsuleAxis), 0, capsule.axis.w)));

                // Set the contact information
                contactInfo.position = intersectionPoint;
                contactInfo.normal = normal;

                return true;
            }
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
        /// Checks if a line segment intersects with a mesh and returns the closest intersection point and normal.
        /// </summary>
        public static bool GetLineIntersectPoint(float3 p1, float3 p2, MeshDesc mesh, out ContactInfo contactInfo)
        {
            contactInfo = default(ContactInfo);

            // Initialize variables to hold the closest intersection point and normal
            float3 closestIntersectionPoint = float3.zero;
            float3 closestIntersectionNormal = float3.zero;

            // Initialize variable to hold the closest squared distance to the line segment
            float closestSqDistance = float.MaxValue;

            // Iterate over all triangles in the mesh
            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                // Get the vertices of the current triangle
                float3 v0 = mesh.vertices[mesh.triangles[i]];
                float3 v1 = mesh.vertices[mesh.triangles[i + 1]];
                float3 v2 = mesh.vertices[mesh.triangles[i + 2]];

                // Calculate the normal of the triangle
                float3 normal = math.normalize(math.cross(v1 - v0, v2 - v0));

                // Calculate the distance from p1 to the plane of the triangle
                float distanceToPlaneP1 = math.dot(normal, v0 - p1);

                // Calculate the distance from p2 to the plane of the triangle
                float distanceToPlaneP2 = math.dot(normal, v0 - p2);

                // If p1 and p2 are on the same side of the plane, skip this triangle
                if (distanceToPlaneP1 * distanceToPlaneP2 > 0)
                    continue;

                // Calculate the intersection point of the line segment with the plane of the triangle
                float t = distanceToPlaneP1 / (distanceToPlaneP1 - distanceToPlaneP2);
                float3 intersectionPoint = p1 + t * (p2 - p1);

                // Check if the intersection point is inside the triangle
                if (PointInsideTriangle(intersectionPoint, v0, v1, v2))
                {
                    // Calculate the squared distance from the intersection point to the line segment
                    float sqDistance = math.lengthsq(intersectionPoint - p1);

                    // If this intersection point is closer to the line segment, update the closest values
                    if (sqDistance < closestSqDistance)
                    {
                        closestIntersectionPoint = intersectionPoint;
                        closestIntersectionNormal = normal;
                        closestSqDistance = sqDistance;
                    }
                }
            }

            // If a valid intersection point was found, set the contact information and return true
            if (closestSqDistance < float.MaxValue)
            {
                contactInfo.position = closestIntersectionPoint;
                contactInfo.normal = closestIntersectionNormal;
                return true;
            }

            // No intersection point found
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