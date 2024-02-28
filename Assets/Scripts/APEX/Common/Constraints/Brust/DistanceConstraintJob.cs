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

        public void Execute(int index)
        {
            // IDK why Execute will lock the array length, so must be do some trick
            if (index >= constraints.Length)
            {
                return;
            }

            var con = constraints[index];
            var delta = nextPosition[con.pl] - nextPosition[con.pr];

            float currentDistance = math.length(delta);
            float error = currentDistance - restLength;

            if (currentDistance > Mathf.Epsilon)
            {
                float3 correction = (error * stiffness) * math.normalize(delta);

                var ml = masses[con.pl];
                var mr = masses[con.pr];
                var totalM = ml + mr;

                nextPosition[con.pl] -= (d * ml / totalM) * correction;
                nextPosition[con.pr] += (d * mr / totalM) * correction;
            }
        }
    }
}