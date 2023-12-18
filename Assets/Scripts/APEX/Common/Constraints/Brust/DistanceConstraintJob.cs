using System.Collections.Generic;
using System.Linq;
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
    public struct DistanceConstraintJob : IJobParallelFor
    {
        public NativeArray<ApexParticleBaseBurst> particles;

        [ReadOnly] public NativeParallelHashMap<int, NativeArray<ApexConstraintParticleDouble>> constraints;
        [ReadOnly] public float restLength;
        [ReadOnly] public float stiffness;

        public void ParticleCallback(List<ApexParticleBase> callbackParticle)
        {
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].ConvertBaseClass(callbackParticle[i]);
            }
        }

        /// <summary>
        /// Line Constructor:
        ///     particle one by one connect.
        /// </summary>
        /// <param name="doubleConnect">Reverse connection (2x)</param>
        public void LineConstructor(bool doubleConnect = true)
        {
            constraints = new NativeParallelHashMap<int, NativeArray<ApexConstraintParticleDouble>>();
            for (int i = 0; i < particles.Length - 1; i++)
            {
                var lToR = new ApexConstraintParticleDouble(this.particles[i].index, this.particles[i + 1].index);
                var rToL = new ApexConstraintParticleDouble(this.particles[i + 1].index, this.particles[i].index);

                // Do not use ??= expression in Unity
                if (!constraints.ContainsKey(i))
                {
                    // In general, the length constraint to which a particle is connected is at most 8 (surface body).
                    constraints.Add(i, new NativeArray<ApexConstraintParticleDouble>(8, Allocator.Persistent));
                }

                if (!constraints.ContainsKey(i + 1))
                {
                    constraints.Add(i + 1, new NativeArray<ApexConstraintParticleDouble>(8, Allocator.Persistent));
                }

                // Tail-in data
                var apexConstraintParticleDoubles = constraints[i];
                apexConstraintParticleDoubles[apexConstraintParticleDoubles.Length] = lToR;

                if (doubleConnect)
                {
                    var constraintParticleDoubles = constraints[i];
                    constraintParticleDoubles[constraintParticleDoubles.Length] = rToL;
                }
            }
        }

        public void Execute(int index)
        {
            foreach (var constraint in constraints)
            {
                foreach (var single in constraint.Value)
                {
                    CalcParticleConstraint(particles[single.pl].nextPosition,
                        particles[single.pr].nextPosition,
                        particles[single.pl].isStatic,
                        particles[single.pr].isStatic);
                }
            }
        }

        private void CalcParticleConstraint(float3 l, float3 r, bool lStatic, bool rStatic)
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
        }
    }
}