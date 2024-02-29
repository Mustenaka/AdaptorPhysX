using Unity.Mathematics;

namespace APEX.Common.Collider.Desc
{
    /// <summary>
    /// Sphere information
    /// </summary>
    public struct SphereDesc : IColliderDesc
    {
        public float3 center;
        public float radius;
    }


    /// <summary>
    /// Box information
    /// </summary>
    public struct BoxDesc : IColliderDesc
    {
        public float3 min;
        
        // 3个边轴,xyz为normalzied的朝向，w为长度
        public float4 ax;
        public float4 ay;
        public float4 az;
    }


    /// <summary>
    /// Capsule information
    /// </summary>
    public struct CapsuleDesc : IColliderDesc
    {
        public float3 c0;
        public float4 axis; // xyz为单位化的方向，w为长度
        public float radius;
    }
}