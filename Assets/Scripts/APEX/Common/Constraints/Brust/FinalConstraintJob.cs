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

        [ReadOnly] public NativeArray<ApexPinConstraint> pin;

        public void Execute(int index)
        {
            var constraintType = particleConstraintTypes[index];
            switch (constraintType)
            {
                case EApexParticleConstraintType.Pin:
                    for (var i = 0; i < pin.Length; i++)
                    {
                        if (pin[i].index != index) continue;
                        this.position[index] = pin[i].position;
                        break;
                    }

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