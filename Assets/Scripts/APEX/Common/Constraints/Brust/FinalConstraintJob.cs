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
        [ReadOnly] public float dt;
        [ReadOnly] public NativeArray<ApexPinConstraint> pin;
        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public float stiffness;
        [ReadOnly] public float restLength;
        public NativeArray<float3> localForce;
        public NativeArray<float3> particleForce;

        public void Execute(int index)
        {
            var constraintType = particleConstraintTypes[index];
            var cnt = particleConstraintTypes.Length;
            switch (constraintType)
            {
                case EApexParticleConstraintType.Pin:
                    position[index] = pin[index].position;
                    break;
                case EApexParticleConstraintType.Collision:
                    break;
                case EApexParticleConstraintType.Free:
                    break;
                case EApexParticleConstraintType.Resist:
                    break;
                case EApexParticleConstraintType.Drag:
                    position[index] = pin[index].position;

                    var totalForce = float3.zero;

                    // 计算并反馈绳子对重物的力
                    if (index + 1 < cnt)
                    {
                        var dir = math.normalize(position[index + 1] - position[index]);
                        var len = math.max(0, math.length(position[index + 1] - position[index]) - restLength);
                    
                        float3 force = (1 - stiffness) * (masses[index] * (dir * len) / (dt * dt));
                        totalForce += force;
                        localForce[index] += force;
                    
                        // 反馈力到重物
                        localForce[index + 1] -= force;
                    }
                    
                    // 计算并反馈重物对绳子的力
                    if (index - 1 >= 0)
                    {
                        var dir = math.normalize(position[index - 1] - position[index]);
                        var len = math.max(0, math.length(position[index - 1] - position[index]) - restLength);
                    
                        float3 force = (1 - stiffness) * (masses[index] * (dir * len) / (dt * dt));
                        totalForce += force;
                        localForce[index] += force;
                    
                        // 反馈力到绳子上的相邻节点
                        localForce[index - 1] -= force;
                    }
                    
                    // 考虑阻尼影响
                    localForce[index] -= 0.98f * totalForce;

                    // if (index + 1 < cnt)
                    // {
                    //     var dir = math.normalize(position[index + 1] - position[index]);
                    //     var len = math.max(0, math.length(position[index + 1] - position[index]) - restLength);
                    //
                    //     localForce[index] += (1 - stiffness) * (masses[index] * (dir * len) / (dt * dt));
                    // }
                    //
                    // if (index - 1 >= 0)
                    // {
                    //     var dir = math.normalize(position[index - 1] - position[index]);
                    //     var len = math.max(0, math.length(position[index - 1] - position[index]) - restLength);
                    //
                    //     localForce[index] += (1 - stiffness) * (masses[index] * (dir * len) / (dt * dt));
                    // }

                    break;
                default: break;
            }
        }
    }
}