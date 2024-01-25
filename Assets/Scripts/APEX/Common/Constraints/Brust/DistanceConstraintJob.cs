using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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
        [NativeDisableUnsafePtrRestriction] public NativeArray<float3> nextPosition;
        [ReadOnly] public NativeArray<int> pinIndex;
        [ReadOnly] public NativeArray<ApexConstraintParticleDouble> constraints;
        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            // Debug.Log("call back: " + callbackParticle[1].nextPosition + " " + adjustNextPosition[1]);
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
                    nextPosition[con.pl] -= correction;
                    nextPosition[con.pr] += correction;
                }
                else if (lStatic && !rStatic)
                {
                    nextPosition[con.pr] += correction * 2;
                }
                else if (!lStatic && rStatic)
                {
                    nextPosition[con.pl] -= correction * 2;
                }
            }
        }
    }
}