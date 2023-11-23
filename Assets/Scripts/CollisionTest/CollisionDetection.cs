using UnityEngine;

namespace CollisionTest
{
    public class CollisionDetection : MonoBehaviour
    {
        public Vector3[] points;

        public GameObject target;
        public CapsuleCollider capsule;

        public Mesh bakedMesh; // use to after bake mesh

        private void Start()
        {
            points = new Vector3[] { };
            capsule = this.GetComponent<CapsuleCollider>();
            BakeMeshFromSkinRenderer();
        }

        private void Update()
        {
            points = GetCollisionPoints(capsule, bakedMesh);
        }

        // 烘焙Mesh
        private void BakeMeshFromSkinRenderer()
        {
            var skinRenderer = target.GetComponent<SkinnedMeshRenderer>();
            if (skinRenderer != null)
            {
                bakedMesh = new Mesh();
                skinRenderer.BakeMesh(bakedMesh);
            }
            else
            {
                Debug.LogError("Target does not have a SkinnedMeshRenderer component.");
            }
        }

        // 示例函数，返回胶囊体和Mesh的碰撞点数组
        public Vector3[] GetCollisionPoints(CapsuleCollider capsule, Mesh mesh)
        {
            // 获取胶囊体的位置和方向
            Vector3 capsulePosition = capsule.transform.position;
            Vector3 capsuleDirection = capsule.transform.up;

            // 获取胶囊体的高度和半径
            float capsuleHeight = capsule.height;
            float capsuleRadius = capsule.radius;

            // 执行胶囊体和Mesh的碰撞检测
            RaycastHit[] hits = Physics.CapsuleCastAll(
                capsulePosition,
                capsulePosition + capsuleDirection * capsuleHeight,
                capsuleRadius,
                capsuleDirection,
                Mathf.Infinity
                // mesh
            );

            // 提取碰撞点数组
            Vector3[] collisionPoints = new Vector3[hits.Length];
            for (int i = 0; i < hits.Length; i++)
            {
                collisionPoints[i] = hits[i].point;
            }

            return collisionPoints;
        }
    }
}