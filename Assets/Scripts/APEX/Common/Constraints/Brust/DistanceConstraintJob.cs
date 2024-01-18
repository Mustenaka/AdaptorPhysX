using System.Collections.Generic;
using System.Linq;
using APEX.Common.Particle;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Distance constraint burst version, a constraint based on the target length
    /// </summary>
    public struct DistanceConstraintJob : IJobFor
    {
        [ReadOnly] public NativeArray<float3> particlesNextPosition;
        [WriteOnly] public NativeArray<float3> particlesAdjustNextPosition;

        [ReadOnly] public NativeArray<int> pinIndex;
        [ReadOnly] public NativeParallelHashMap<int, NativeArray<ApexConstraintParticleDouble>> constraints;
        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            for (int i = 0; i < particlesNextPosition.Length; i++)
            {
                callbackParticle[i].nextPosition = particlesAdjustNextPosition[i];
            }
        }
        
        public void Execute(int index)
        {
            var con = constraints[index];
            foreach (var single in con)
            {
                // position correction through constraint
                CalcParticleConstraint(particlesNextPosition[single.pl],
                    particlesNextPosition[single.pr],
                    pinIndex.Contains(single.pr),
                    pinIndex.Contains(single.pl),
                    out float3 resultL, out float3 resultR);

                // apply the position
                particlesAdjustNextPosition[single.pl] = resultL;
                particlesAdjustNextPosition[single.pr] = resultR;
            }
        }

        private void CalcParticleConstraint(float3 l, float3 r, bool lStatic, bool rStatic,
            out float3 resultL, out float3 resultR)
        {
            var delta = l - r;
            float currentDistance = math.length(delta);
            float error = currentDistance - restLength;

            if (currentDistance > Mathf.Epsilon)
            {
                float3 correction = math.normalize(delta) * (error * stiffness);

                // if one side Static, than static one sid, the other side double offset
                if (!lStatic && !rStatic)
                {
                    l -= correction;
                    r += correction;
                }
                else if (lStatic && !rStatic)
                {
                    r += (correction + correction);
                }
                else if (!lStatic && rStatic)
                {
                    l -= (correction + correction);
                }
            }

            resultL = l;
            resultR = r;
        }
    }
}