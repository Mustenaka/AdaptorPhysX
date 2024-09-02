using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    ///  XPBD Bend constraint burst version, a constraint based on the desired angle between segments
    /// </summary>
    [BurstCompile]
    public struct XBendConstraintJob : IJobFor
    {
        [NativeDisableUnsafePtrRestriction] public NativeArray<float3> nextPosition;

        [ReadOnly] public NativeArray<ApexConstraintParticleThree> bendConstraints;
        [ReadOnly] public NativeArray<float> masses;

        [ReadOnly] public float restAngle; // Desired angle between segments
        [ReadOnly] public float compliance;
        [ReadOnly] public float dt;

        [NativeDisableUnsafePtrRestriction] public NativeArray<float> lagrangeMultipliers; // Store lambda values

        public void Execute(int index)
        {
            if (index >= bendConstraints.Length || index == 0)
            {
                return;
            }

            // if (index % 3 != 1)
            // {
            //     return;
            // }

            var constraint = bendConstraints[index];
            int p1 = constraint.pl;
            int p2 = constraint.pmid;
            int p3 = constraint.pr;

            // Calculate vectors for the two segments
            var v1 = nextPosition[p2] - nextPosition[p1];
            var v2 = nextPosition[p3] - nextPosition[p2];

            // Compute the current angle
            float cosAngle = math.dot(math.normalize(v1), math.normalize(v2));
            float currentAngle = math.acos(math.clamp(cosAngle, -1f, 1f));
            // float currentAngle = math.acos(cosAngle);
            float angleError = currentAngle - restAngle;

            // Calculate corrections
            float alpha = compliance / (dt * dt);
            float invM1 = 1.0f / masses[p1];
            float invM2 = 1.0f / masses[p2];
            float invM3 = 1.0f / masses[p3];

            // Effective mass
            float denom = invM1 + invM2 + invM3 + alpha;
            float lambdaDelta = (-angleError - alpha * lagrangeMultipliers[index]) / denom;

            lagrangeMultipliers[index] += lambdaDelta;

            // Apply corrections
            var correction = lambdaDelta * (math.cross(v1, v2) / math.length(math.cross(v1, v2)));
            
            nextPosition[p1] -= correction * invM1 * 0.001f;
            nextPosition[p2] += correction * invM2 * 0.001f;
            nextPosition[p3] -= correction * invM3 * 0.001f;
        }
    }
}