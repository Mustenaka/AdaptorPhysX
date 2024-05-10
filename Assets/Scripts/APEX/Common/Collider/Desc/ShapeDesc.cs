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
        public float3 center;

        // 3 side axes,xyz is the orientation of normalized
        // and w is the length
        public float4 ax;
        public float4 ay;
        public float4 az;
    }

    /// <summary>
    /// Capsule information
    /// </summary>
    public struct CapsuleDesc : IColliderDesc
    {
        // center point of capsule
        public float3 center;

        // xyz is the unitized direction and w is the length
        public float4 axis;

        // The radius of the upper or lower hemispheres of the capsule
        public float radius;
    }

    /// <summary>
    /// Mesh information
    /// </summary>
    public struct MeshDesc : IColliderDesc
    {
        // center point of the mesh (sometime we call it "mass_point")
        public float3[] vertices;
        public int[] triangles;
        public float3 center;
    }
}