using Unity.Mathematics;
using UnityEngine;

namespace APEX.Tools.MathematicsTools
{
    public static class ExtendConvert
    {
        /// <summary>
        /// convert UnityEngine.Vector3 to Unity.Mathematics.float3
        /// </summary>
        /// <param name="v3">from vector3</param>
        /// <returns>to float3</returns>
        public static float3 ToFloat3(this Vector3 v3)
        {
            return (float3)v3;
            // return new float3(v3.x, v3.y, v3.z);
        }

        /// <summary>
        /// convert Unity.Mathematics.float3 to UnityEngine.Vector3
        /// </summary>
        /// <param name="f3">from float3</param>
        /// <returns>to Vector3</returns>
        public static Vector3 ToVector3(this float3 f3)
        {
            return (Vector3)f3;
            // return new Vector3(f3.x, f3.y, f3.z);
        }
    }
}