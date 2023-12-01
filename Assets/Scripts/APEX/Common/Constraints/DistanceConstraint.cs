using System.Collections.Generic;
using APEX.Common.Constraints.Base;
using APEX.Common.Particle;
using Unity.VisualScripting;
using UnityEngine;

namespace APEX.Common.Constraints
{
    public class DistanceConstraint<T> : ApexConstraintBatchBase where T : ApexParticleBase
    {
        // the rest length in its natural state
        public float restLength = 1.2f;

        // Relaxation Parameter
        public float stiffness = 0.5f;

        // particle group
        public List<T> particles;

        // Create Constraint By particles
        public DistanceConstraint(ref List<T> particles, bool doubleConnect = true)
        {
            constraintBatchType = EApexConstraintBatchType.DistanceConstraint;
            this.particles = particles;

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<ApexConstraintPair>>();
            for (int i = 0; i < particles.Count - 1; i++)
            {
                var lToR = new ApexConstraintPair(this.particles[i].Index, this.particles[i + 1].Index);
                var rToL = new ApexConstraintPair(this.particles[i + 1].Index, this.particles[i].Index);
                
                constraints[i] ??= new List<ApexConstraintPair>();
                constraints[i + 1] ??= new List<ApexConstraintPair>();

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
                    CalcParticleConstraint(ref particles[single.pl].NextPosition,
                        ref particles[single.pr].NextPosition, 
                        particles[single.pl].IsStatic,
                        particles[single.pr].IsStatic);
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