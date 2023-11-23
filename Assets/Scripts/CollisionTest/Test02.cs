using UnityEngine;

namespace CollisionTest
{
    public class ClosestPointOnCapsule : MonoBehaviour
    {
        public GameObject capsuleObject; // 你的Capsule对象
        public GameObject meshObject; // 你的Mesh对象

        private GameObject GeneratePoint(Vector3 p)
        {
            // create gameobject for point
            var tmp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            tmp.transform.position = p;
            tmp.transform.localScale = new(0.1f, 0.1f, 0.1f);

            // remove collider
            var collider = tmp.GetComponent<Collider>();
            Destroy(collider);

            return tmp;
        }
        
        void Start()
        {
            if (capsuleObject != null && meshObject != null)
            {
                // 获取CapsuleCollider和MeshFilter组件
                CapsuleCollider capsuleCollider = capsuleObject.GetComponent<CapsuleCollider>();
                MeshFilter meshFilter = meshObject.GetComponent<MeshFilter>();

                if (capsuleCollider != null && meshFilter != null)
                {
                    // 获取Capsule上的最近点
                    Vector3 closestPointOnCapsule = capsuleCollider.ClosestPoint(meshObject.transform.position);

                    // 获取Mesh上的最近点
                    Vector3[] meshVertices = meshFilter.mesh.vertices;
                    Vector3 closestPointOnMesh =
                        GetClosestPointOnMesh(meshObject.transform, meshVertices, closestPointOnCapsule);

                    // 输出结果
                    Debug.Log("Closest Point on Capsule: " + closestPointOnCapsule);
                    Debug.Log("Closest Point on Mesh: " + closestPointOnMesh);

                    var pc = GeneratePoint(closestPointOnCapsule);
                    var pm = GeneratePoint(closestPointOnMesh);
                }
                else
                {
                    Debug.LogError("CapsuleCollider or MeshFilter not found on specified objects.");
                }
            }
            else
            {
                Debug.LogError("Capsule or Mesh objects not specified.");
            }
        }

        // 获取Mesh上的最近点
        Vector3 GetClosestPointOnMesh(Transform meshTransform, Vector3[] meshVertices, Vector3 point)
        {
            float minDistance = float.MaxValue;
            Vector3 closestPoint = Vector3.zero;

            foreach (Vector3 vertex in meshVertices)
            {
                // 将Mesh顶点从局部坐标系转换为世界坐标系
                Vector3 worldVertex = meshTransform.TransformPoint(vertex);

                // 计算顶点和Capsule上点之间的距离
                float distance = Vector3.Distance(point, worldVertex);

                // 如果距离更短，则更新最近点
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestPoint = worldVertex;
                }
            }

            return closestPoint;
        }
    }
}