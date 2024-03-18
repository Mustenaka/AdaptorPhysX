using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace APEX.Common.Constraints
{
    public struct FinalConstraintJob : IJobFor
    {
        [NativeDisableUnsafePtrRestriction] public NativeArray<float3> position;

        [ReadOnly] public NativeArray<EApexParticleConstraintType> particleConstraintTypes;

        // Change it, Length of Pin same as Particle Count
        [ReadOnly] public NativeArray<ApexPinConstraint> pin;

        public void Execute(int index)
        {
            var constraintType = particleConstraintTypes[index];
            switch (constraintType)
            {
                case EApexParticleConstraintType.Pin:
                    position[index] = pin[index].position;
                    break;
                case EApexParticleConstraintType.Collision:
                    break;
                case EApexParticleConstraintType.Free:
                case EApexParticleConstraintType.Resist:
                default: break;
            }
        }
    }
}