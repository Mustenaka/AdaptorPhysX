using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// XPBD Distance constraint burst version, a constraint based on the target length
    /// </summary>
    public class XDistanceConstraintJob : IJobFor
    {
        [NativeDisableUnsafePtrRestriction] public NativeArray<float3> nextPosition;

        [ReadOnly] public NativeArray<ApexConstraintParticleDouble> constraints;

        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public float d;
        
        // XPBD specific parameters
        public NativeArray<float> lagrangeMultipliers;  // Store lambda values
        public float compliance;  // Compliance parameter (alpha)
        public float deltaTime;  // Time step (dt)
        
        public void Execute(int index)
        {
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
                float3 normalizedDelta = math.normalize(delta);

                // Calculate inverse masses
                var ml = masses[con.pl];
                var mr = masses[con.pr];
                var totalM = ml + mr;

                float invMassL = ml > 0f ? 1.0f / ml : 0f;
                float invMassR = mr > 0f ? 1.0f / mr : 0f;

                // Calculate compliance
                float complianceTerm = compliance / (deltaTime * deltaTime);

                // Update Lagrange multiplier (lambda)
                float effectiveMass = 1.0f / (invMassL + invMassR + complianceTerm);
                float deltaLambda = (-error - lagrangeMultipliers[index] * complianceTerm) * effectiveMass;
                lagrangeMultipliers[index] += deltaLambda;

                // Apply correction based on lambda
                float3 correction = deltaLambda * normalizedDelta;

                nextPosition[con.pl] -= (d * invMassL) * correction;
                nextPosition[con.pr] += (d * invMassR) * correction;
            }
        }
    }
}