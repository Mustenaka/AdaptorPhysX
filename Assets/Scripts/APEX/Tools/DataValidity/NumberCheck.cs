using UnityEngine;

namespace APEX.Tools
{
    public static class NumberCheck
    {
        /// <summary>
        /// Check the vector2 is NaN
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static bool IsVector2NaN(Vector2 vector)
        {
            return float.IsNaN(vector.x) || float.IsNaN(vector.y);
        }
        
        /// <summary>
        /// Check the vector3 is NaN
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        public static bool IsVector3NaN(Vector3 vector)
        {
            return float.IsNaN(vector.x) || float.IsNaN(vector.y) || float.IsNaN(vector.z);
        }
    }
}