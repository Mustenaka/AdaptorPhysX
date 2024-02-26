using Unity.Mathematics;

namespace APEX.Common.Constraints
{
    public struct ApexPinConstraint
    {
        public float3 position;

        public ApexPinConstraint(float3 position)
        {
            this.position = position;
        }
    }
}