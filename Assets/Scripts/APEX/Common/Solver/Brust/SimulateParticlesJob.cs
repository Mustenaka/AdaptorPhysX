using APEX.Common.Particle;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Solver
{
    [BurstCompile]
    public struct SimulateParticlesJob: IJobParallelFor
    {            
        public NativeArray<ApexParticleBaseBurst> particles;
        [ReadOnly] public int iterator;
        [ReadOnly] public float3 gravity;
        [ReadOnly] public float3 globalForce;
        [ReadOnly] public float airDrag;
        [ReadOnly] public float damping;
        [ReadOnly] public float dt;
        
        /// <summary>
        /// Execute force extend. (Contain air resistance, force apply)
        /// </summary>
        /// <param name="index">the particle index</param>
        public void Execute(int index)
        {
            var particle = particles[index];

            // simplex pin
            if (particle.isStatic)
                return;

            // batch iteration
            for (int i = 0; i < iterator; i++)
            {
                // calc air resistance
                float3 airResistance = -airDrag * (particle.nowPosition - particle.previousPosition) / dt;

                // calc force apply.
                particle.forceApply = gravity + globalForce + airResistance + particle.forceExt;
                particle.nextPosition = particle.nowPosition
                                        + (1 - damping) * (particle.nowPosition - particle.previousPosition)
                                        + particle.forceApply / particle.mass * (dt * dt);

                particles[index] = particle;
            }
        }
    }
}