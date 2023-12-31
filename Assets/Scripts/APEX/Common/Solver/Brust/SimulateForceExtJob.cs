using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace APEX.Common.Solver
{
    [BurstCompile]
    public struct SimulateForceExtJob : IJobFor
    {
        [ReadOnly] public NativeArray<float3> previousPosition;
        [ReadOnly] public NativeArray<float3> nowPosition;
        [ReadOnly] public NativeArray<bool> isStatic;
        [ReadOnly] public NativeArray<float3> forceExt;
        [ReadOnly] public NativeArray<float> mass;
        
        [WriteOnly] public NativeArray<float3> nextPosition;
        
        [ReadOnly] public float3 gravity;
        [ReadOnly] public float3 globalForce;
        [ReadOnly] public float airDrag;
        [ReadOnly] public float damping;
        [ReadOnly] public float dt;
        [ReadOnly] public int iterator;     // default is 1
        
        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            for (int i = 0; i < nextPosition.Length; i++)
            {
                callbackParticle[i].nextPosition = nextPosition[i];
            }
        }

        /// <summary>
        /// Execute force extend. (Contain air resistance, force apply)
        /// </summary>
        /// <param name="index">the particle index</param>
        public void Execute(int index)
        {
            // iterator adjust
            if (iterator <= 1)
            {
                iterator = 1;
            }
            
            // simplex pin
            if (isStatic[index])
            {
                return;
            }
            
            // calc air resistance
            float3 airResistance = -airDrag * (nowPosition[index] - previousPosition[index]) / dt;

            // calc force apply.
            var forceApply = gravity + globalForce + airResistance + forceExt[index];
            nextPosition[index] = nowPosition[index]
                                    + (1 - damping) * (nowPosition[index] - previousPosition[index])
                                    + forceApply / mass[index] * (dt * dt) * iterator;
        }
    }
}