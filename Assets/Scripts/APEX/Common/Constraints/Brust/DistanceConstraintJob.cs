using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Distance constraint burst version, a constraint based on the target length
    /// </summary>
    [BurstCompile]
    public struct DistanceConstraintJob : IJobFor
    {
        public NativeArray<float3> nextPosition;
        // public NativeArray<float3> adjustNextPosition;

        [ReadOnly] public NativeArray<int> pinIndex;
        [ReadOnly] public NativeArray<ApexConstraintParticleDouble> constraints;
        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;
        [WriteOnly] public float3 way;

        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            // Debug.Log("call back: " + callbackParticle[1].nextPosition + " " + nextPosition[1]);
            for (int i = 0; i < nextPosition.Length; i++)
            {
                callbackParticle[i].nextPosition = nextPosition[i];
            }
        }

        public void Execute(int index)
        {
            var con = constraints[index];

            var delta = nextPosition[con.pl] - nextPosition[con.pr];
            float currentDistance = math.length(delta);
            float error = currentDistance - restLength;

            var lStatic = pinIndex.Contains(con.pl);
            var rStatic = pinIndex.Contains(con.pr);

            if (currentDistance > Mathf.Epsilon)
            {
                float3 correction = math.normalize(delta) * (error * stiffness);

                // if one side Static, than static one sid, the other side double offset
                if (!lStatic && !rStatic)
                {
                    nextPosition[con.pl] = nextPosition[con.pl] - correction;
                    nextPosition[con.pr] = nextPosition[con.pr] + correction;
                }
                else if (lStatic && !rStatic)
                {
                    nextPosition[con.pr] = nextPosition[con.pr] + correction * 2;
                }
                else if (!lStatic && rStatic)
                {
                    nextPosition[con.pl] = nextPosition[con.pl] - correction * 2;
                }
            }

            // position correction through constraint
            // CalcParticleConstraint(nextPosition[l],
            //     nextPosition[r],
            //     pinIndex.Contains(l),
            //     pinIndex.Contains(r),
            //     out float3 resultL, out float3 resultR);

            // apply the position
            // adjustNextPosition[l] = resultL;
            // adjustNextPosition[r] = resultR;
            //

            if (pinIndex.Contains(con.pl))
            {
                way = nextPosition[con.pr];
                // Debug.Log("Execute: " + nextPosition[con.pr] + " " + nextPosition[con.pr] + " way: " + way);
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