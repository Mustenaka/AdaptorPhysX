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
        [ReadOnly] public NativeArray<ApexConstraintParticleDouble> constraints;

        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public float d;

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

            if (currentDistance > Mathf.Epsilon)
            {
                float3 correction = math.normalize(delta) * (error * stiffness);

                var ml = masses[con.pl];
                var mr = masses[con.pr];
                var totalM = ml + mr;

                nextPosition[con.pl] -= correction * d * ml / totalM;
                nextPosition[con.pr] += correction * d * mr / totalM;
            }
        }
    }
}