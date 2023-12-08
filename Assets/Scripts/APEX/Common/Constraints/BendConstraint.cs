using System.Collections.Generic;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// 
    /// </summary>
    [SerializeField]
    public class BendConstraint : ApexConstraintBatchThree
    {
        [SerializeField] [Range(0, 1)] public float stiffness = 0.1f;

        private List<ApexParticleBase> _particles;

        public BendConstraint(ref List<ApexParticleBase> particles, bool doubleConnect = false)
        {
            constraintBatchType = EApexConstraintBatchType.BendConstraint;
            this._particles = particles;

            int cnt = particles.Count;

            // quick return, angle constraint must have more than 3 particle
            if (cnt < 3)
            {
                return;
            }

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<ApexConstraintParticleThree>>();
            for (int i = 1; i < cnt - 1; i++)
            {
                var lToR = new ApexConstraintParticleThree(this._particles[i - 1].index, this._particles[i].index,
                    this._particles[i + 1].index);

                if (!constraints.ContainsKey(i))
                {
                    constraints.Add(i, new List<ApexConstraintParticleThree>());
                }

                constraints[i].Add(lToR);

                // Bend constraints often do not require a reverse connection
                if (doubleConnect)
                {
                    var rToL = new ApexConstraintParticleThree(this._particles[i + 1].index, this._particles[i].index,
                        this._particles[i - 1].index);

                    constraints[i].Add(rToL);
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
                        ref _particles[single.pmid].nextPosition,
                        ref _particles[single.pr].nextPosition,
                        _particles[single.pl].isStatic,
                        _particles[single.pmid].isStatic,
                        _particles[single.pr].isStatic);
                }
            }
        }

        public void CalcParticleConstraint(ref Vector3 l, ref Vector3 mid, ref Vector3 r,
            bool lStatic, bool midStatic, bool rStatic)
        {
            if (lStatic || midStatic || rStatic)
            {
                return;
            }

            // Get mid position of constraint
            Vector3 center = (l + r) * 0.5f;

            // calc bend direction
            Vector3 bendDirection = (center - mid).normalized;

            mid += bendDirection * stiffness;
        }
    }
}