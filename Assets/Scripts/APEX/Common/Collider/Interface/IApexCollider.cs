using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Collider.Interface
{
    public interface IApexCollider
    {
        // collider target entity id
        int entityId { get; }

        // if the other have unity rigid body, attach it
        Rigidbody attachedRigidbody { get; }

        bool GetContactInfo(float3 fromPosition, float3 toPosition, out ContactInfo contactInfo);
        
        // void FillColliderDesc(Collider)
    }
}