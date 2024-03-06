using Unity.Mathematics;

namespace APEX.Common.Constraints
{
    public struct ApexPinConstraint
    {
        public int index;
        public float3 position;

        public ApexPinConstraint(int index, float3 position)
        {
            this.index = index;
            this.position = position;
        }

        public ApexPinConstraint(float3 position)
        {
            this.index = 0;
            this.position = position;
        }
    }
}