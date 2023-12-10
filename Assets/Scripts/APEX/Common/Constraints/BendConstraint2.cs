using System.Collections.Generic;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    public class BendConstraint2 : ApexConstraintBatchDouble
    {
        [SerializeField] public float restLength = 2.4f;
        [SerializeField] [Range(0, 1)] public float stiffness = 0.1f;

        private List<ApexParticleBase> _particles;

        public BendConstraint2(ref List<ApexParticleBase> particles, bool doubleConnect = false)
        {
            constraintBatchType = EApexConstraintBatchType.BendConstraint2;
            this._particles = particles;

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<Constraints.ApexConstraintParticleDouble>>();
            for (int i = 0; i < particles.Count - 2; i++)
            {
                var lToR = new Constraints.ApexConstraintParticleDouble(this._particles[i].index, this._particles[i + 2].index);
                var rToL = new Constraints.ApexConstraintParticleDouble(this._particles[i + 2].index, this._particles[i].index);

                // Do not use ??= expression in Unity
                if (!constraints.ContainsKey(i))
                {
                    constraints.Add(i, new List<Constraints.ApexConstraintParticleDouble>());
                }

                if (!constraints.ContainsKey(i + 2))
                {
                    constraints.Add(i + 2, new List<Constraints.ApexConstraintParticleDouble>());
                }

                constraints[i].Add(lToR);
                
                if (doubleConnect)
                {
                    constraints[i + 2].Add(rToL);
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