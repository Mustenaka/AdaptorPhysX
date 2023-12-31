using System.Collections.Generic;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Distance constraint, a constraint based on the target length
    /// </summary>
    public class DistanceConstraint : ApexConstraintBatchDouble
    {
        // the rest length in its natural state
        public float restLength = 1.2f;

        // Relaxation Parameter Range(0, 1)
        public float stiffness = 0.5f;

        // particle group
        private List<ApexParticleBase> _particles;

        // Create Constraint By particles
        public DistanceConstraint(ref List<ApexParticleBase> particles, bool doubleConnect = true)
        {
            constraintBatchType = EApexConstraintBatchType.DistanceConstraint;
            this._particles = particles;

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<Constraints.ApexConstraintParticleDouble>>();
            for (int i = 0; i < particles.Count - 1; i++)
            {
                var lToR = new Constraints.ApexConstraintParticleDouble(this._particles[i].index, this._particles[i + 1].index);
                var rToL = new Constraints.ApexConstraintParticleDouble(this._particles[i + 1].index, this._particles[i].index);

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
                    CalcParticleConstraint(ref _particles[single.pl].nextPosition,
                        ref _particles[single.pr].nextPosition, 
                        _particles[single.pl].isStatic,
                        _particles[single.pr].isStatic);
                }
            }
        }

        private void CalcParticleConstraint(ref Vector3 l, ref Vector3 r, bool lStatic, bool rStatic)
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