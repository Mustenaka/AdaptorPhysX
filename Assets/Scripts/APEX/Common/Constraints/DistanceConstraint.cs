using System.Collections.Generic;
using APEX.Common.Constraints.Base;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Distance constraint, a constraint based on the target length
    /// </summary>
    /// <typeparam name="T">particle</typeparam>
    public class DistanceConstraint<T> : ApexConstraintBatchDouble where T : ApexParticleBase
    {
        // the rest length in its natural state
        public float restLength = 1.2f;

        // Relaxation Parameter Range(0, 1)
        public float stiffness = 0.5f;

        // particle group
        public List<T> particles;

        // Create Constraint By particles
        public DistanceConstraint(ref List<T> particles, bool doubleConnect = true)
        {
            constraintBatchType = EApexConstraintBatchType.DistanceConstraint;
            this.particles = particles;

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<Constraints.ApexConstraintParticleDouble>>();
            for (int i = 0; i < particles.Count - 1; i++)
            {
                var lToR = new Constraints.ApexConstraintParticleDouble(this.particles[i].index, this.particles[i + 1].index);
                var rToL = new Constraints.ApexConstraintParticleDouble(this.particles[i + 1].index, this.particles[i].index);

                // Do not use ??= expression in Unity
                if (!constraints.ContainsKey(i))
                {
                    constraints.Add(i, new List<Constraints.ApexConstraintParticleDouble>());
                }

                if (!constraints.ContainsKey(i + 1))
                {
                    constraints.Add(i + 1, new List<Constraints.ApexConstraintParticleDouble>());
                }

                constraints[i].Add(lToR);
                
                if (doubleConnect)
                {
                    constraints[i + 1].Add(rToL);
                }
            }
        }

        public override void Do()
        {
            foreach (var constraint in constraints)
            {
                foreach (var single in constraint.Value)
                {
                    CalcParticleConstraint(ref particles[single.pl].nextPosition,
                        ref particles[single.pr].nextPosition, 
                        particles[single.pl].isStatic,
                        particles[single.pr].isStatic);
                }
            }
        }

        public void CalcParticleConstraint(ref Vector3 l, ref Vector3 r, bool lStatic, bool rStatic)
        {
            var delta = l - r;
            float currentDistance = delta.magnitude;
            float error = currentDistance - restLength;

            if (currentDistance > Mathf.Epsilon)
            {
                Vector3 correction = delta.normalized * (error * stiffness);

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
                else if(!lStatic && rStatic)
                {
                    l -= (correction + correction);
                }
            }
        }
    }
}