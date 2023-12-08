using UnityEngine;

namespace APEX.Tools
{
    public static class ExtendMath
    {
        public static Vector3 Projection(Vector3 vectorToProject, Vector3 projectionVector)
        {
            // 计算点积
            float dotProduct = Vector3.Dot(vectorToProject, projectionVector);

            // 计算投影长度的平方
            float projectionLengthSquared = projectionVector.sqrMagnitude;

            // 计算投影
            Vector3 projection = (dotProduct / projectionLengthSquared) * projectionVector;

            return projection;
        }
    }
}