using System;
using UnityEngine;
using System.Collections.Generic;

namespace CollisionTest
{
    public class CloestPointCapsuleCollision : MonoBehaviour
    {
        public Transform capsuleTransform;
        public MeshFilter meshFilter;

        public List<Vector3> collisionPoints;
        public List<GameObject> points;
        public int pointCreateCount = 10;

        private void Start()
        {
            points = new List<GameObject>();
            for (int i = 0; i < pointCreateCount; i++)
            {
                points.Add(GeneratePoint(i));
            }
        }

        private GameObject GeneratePoint(int index)
        {
            // create gameobject for point
            var tmp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            tmp.transform.position = new Vector3(-100f, -100f, -100f);
            tmp.transform.localScale = new(0.1f, 0.1f, 0.1f);
            tmp.name = index.ToString();

            // remove collider
            var collider = tmp.GetComponent<Collider>();
            Destroy(collider);

            return tmp;
        }

        void Update()
        {
            Vector3 capsuleStart =
                capsuleTransform.position - capsuleTransform.up * capsuleTransform.localScale.y * 0.5f;
            Vector3 capsuleEnd = capsuleTransform.position + capsuleTransform.up * capsuleTransform.localScale.y * 0.5f;
            float capsuleRadius = capsuleTransform.localScale.x * 0.5f;

            Matrix4x4 meshTransform = meshFilter.transform.localToWorldMatrix;

            collisionPoints.Clear();

            for (int i = 0; i < meshFilter.mesh.triangles.Length; i += 3)
            {
                Vector3 vertex1 = meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i]]);
                Vector3 vertex2 =
                    meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i + 1]]);
                Vector3 vertex3 =
                    meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i + 2]]);

                List<Vector3> intersectionPoints = CapsuleTriangleIntersection(capsuleStart, capsuleEnd, capsuleRadius,
                    vertex1, vertex2, vertex3);

                collisionPoints.AddRange(intersectionPoints);
            }

            for (int i = 0; i < collisionPoints.Count; i++)
            {
                Debug.Log("Collision Point: " + i + " " + collisionPoints[i]);
                points[i].transform.position = collisionPoints[i];
            }
        }

        List<Vector3> CapsuleTriangleIntersection(Vector3 capsuleStart, Vector3 capsuleEnd, float capsuleRadius,
            Vector3 vertex1, Vector3 vertex2, Vector3 vertex3)
        {
            List<Vector3> intersectionPoints = new List<Vector3>();

            // 计算胶囊体和三角形之间的最近点对
            Vector3 closestPointOnCapsule = ClosestPointOnSegment(capsuleStart, capsuleEnd, vertex1);
            Vector3 closestPointOnTriangle = ClosestPointOnTriangle(capsuleStart, vertex1, vertex2, vertex3);

            // 检查最近点对之间的距离是否小于胶囊体半径
            float distance = Vector3.Distance(closestPointOnCapsule, closestPointOnTriangle);

            if (distance < capsuleRadius)
            {
                intersectionPoints.Add(closestPointOnCapsule);
            }

            return intersectionPoints;
        }

        Vector3 ClosestPointOnSegment(Vector3 start, Vector3 end, Vector3 point)
        {
            Vector3 direction = end - start;
            float length = direction.magnitude;
            direction.Normalize();

            float t = Mathf.Clamp01(Vector3.Dot(point - start, direction) / length);
            return start + t * direction * length;
        }

        Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            // 计算三角形法线
            Vector3 normal = Vector3.Cross(b - a, c - a).normalized;

            // 计算点到平面的投影向量
            Vector3 projection = Vector3.ProjectOnPlane(p - a, normal);

            // 通过投影向量计算最近点
            return p - projection;
        }
    }
}