using APEX.Common.Particle;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace APEX.Common.Solver
{
    [BurstCompile]
    public struct SimulateParticlesJob: IJobParallelFor
    {            
        public NativeArray<ApexParticleBaseBurst> particles;
        public int iterator;
        public float3 gravity;
        public float3 globalForce;
        public float airDrag;
        public float damping;
        public float dt;
        
        /// <summary>
        /// Execute force extend.
        /// </summary>
        /// <param name="index">the particle index</param>
        public void Execute(int index)
        {
            var particle = particles[index];

            // simplex pin
            if (particle.isStatic)
                return;

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