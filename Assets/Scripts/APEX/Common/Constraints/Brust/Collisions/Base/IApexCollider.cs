using UnityEngine;

namespace APEX.Common.Collisions
{
    public interface IApexCollider
    {
        public void DetectCollisions(Object target);
        public void ApplyCollisionFeedback();
    }
}