using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Distance constraint burst version, a constraint based on the target length
    /// </summary>
    public struct DistanceConstraintJob : IJobFor
    {
        [ReadOnly] public NativeArray<float3> nextPosition;
        public NativeArray<float3> adjustNextPosition;

        [ReadOnly] public NativeArray<int> pinIndex;
        [ReadOnly] public NativeArray<ApexConstraintParticleDouble> constraints;
        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            Debug.Log(callbackParticle[1].nextPosition + " ----- " + adjustNextPosition[1]);
            
            for (int i = 0; i < adjustNextPosition.Length; i++)
            {
                callbackParticle[i].nextPosition = adjustNextPosition[i];
            }
        }

        public void Execute(int index)
        {
            var con = constraints[index];
            var l = con.pl;
            var r = con.pr;

            // position correction through constraint
            CalcParticleConstraint(nextPosition[l],
                nextPosition[r],
                pinIndex.Contains(l),
                pinIndex.Contains(r),
                out float3 resultL, out float3 resultR);

            // apply the position
            adjustNextPosition[l] = resultL;
            adjustNextPosition[r] = resultR;

            if (pinIndex.Contains(l))
            {
                Debug.Log(nextPosition[r] + " " + adjustNextPosition[r] + " " + resultL + " " +
                          resultR);
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

                // if (lStatic)
                // {
                //     Debug.Log(correction + " ----- " + r);
                // }
            }

            resultL = l;
            resultR = r;
        }
    }
}