using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.Collections;
using Unity.Jobs;

namespace APEX.Common.Constraints
{
    public class DistanceConstraintJob : IJobParallelFor
    {
        public NativeArray<ApexParticleBaseBurst> particles;

        [ReadOnly] public NativeParallelHashMap<int, NativeArray<DoubleJob>> constraints;
        [ReadOnly] public float restLength = 1.2f;
        [ReadOnly] public float stiffness = 0.5f;
        
        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].ConvertBaseClass(callbackParticle[i]);
            }
        }

        public void ConstraintConstructor()
        {
            
        }
        
        public void Execute(int index)
        {
            
        }
    }
}