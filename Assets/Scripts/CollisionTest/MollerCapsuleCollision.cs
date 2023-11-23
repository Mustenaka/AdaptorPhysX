using System.Collections.Generic;
using UnityEngine;

namespace CollisionTest
{
    public class MollerCapsuleCollision : MonoBehaviour
    {
        public Transform capsuleTransform;
        public MeshFilter meshFilter;

        void Update()
        {
            Vector3 capsuleStart =
                capsuleTransform.position - capsuleTransform.up * capsuleTransform.localScale.y * 0.5f;
            Vector3 capsuleEnd = capsuleTransform.position + capsuleTransform.up * capsuleTransform.localScale.y * 0.5f;

            Matrix4x4 meshTransform = meshFilter.transform.localToWorldMatrix;

            List<Vector3> collisionPoints = new List<Vector3>();

            for (int i = 0; i < meshFilter.mesh.triangles.Length; i += 3)
            {
                Vector3 vertex1 = meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i]]);
                Vector3 vertex2 =
                    meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i + 1]]);
                Vector3 vertex3 =
                    meshTransform.MultiplyPoint(meshFilter.mesh.vertices[meshFilter.mesh.triangles[i + 2]]);

                List<Vector3> intersectionPoints =
                    CapsuleTriangleIntersection(capsuleStart, capsuleEnd, vertex1, vertex2, vertex3);

                collisionPoints.AddRange(intersectionPoints);
            }

            foreach (Vector3 point in collisionPoints)
            {
                Debug.Log("Collision Point: " + point);
            }
        }

        List<Vector3> CapsuleTriangleIntersection(Vector3 capsuleStart, Vector3 capsuleEnd, Vector3 vertex1,
            Vector3 vertex2, Vector3 vertex3)
        {
            List<Vector3> intersectionPoints = new List<Vector3>();

            // Möller–Trumbore算法
            Vector3 e1, e2, h, s, q;
            float a, f, u, v;

            e1 = vertex2 - vertex1;
            e2 = vertex3 - vertex1;
            h = Vector3.Cross(capsuleEnd - capsuleStart, e2);
            a = Vector3.Dot(e1, h);

            if (a > -float.Epsilon && a < float.Epsilon)
                return intersectionPoints;

            f = 1.0f / a;
            s = capsuleStart - vertex1;
            u = f * Vector3.Dot(s, h);

            if (u < 0.0f || u > 1.0f)
                return intersectionPoints;

            q = Vector3.Cross(s, e1);
            v = f * Vector3.Dot(capsuleEnd - capsuleStart, q);

            if (v < 0.0f || u + v > 1.0f)
                return intersectionPoints;

            float t = f * Vector3.Dot(e2, q);

            if (t > float.Epsilon)
            {
                Vector3 intersectionPoint = capsuleStart + t * (capsuleEnd - capsuleStart);
                intersectionPoints.Add(intersectionPoint);
            }

            return intersectionPoints;
        }
    }
}