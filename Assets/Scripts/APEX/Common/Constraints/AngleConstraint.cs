using System.Collections.Generic;
using APEX.Common.Constraints.Base;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    /// <summary>
    /// Angle constraint, a constraint based on the target three particle of the mid particle
    /// </summary>
    /// <typeparam name="T">particle</typeparam>
    public class AngleConstraint<T> : ApexConstraintBatchThree where T : ApexParticleBase
    {
        // 
        public float desiredAngle = Mathf.PI / 2f;
        [Range(0, 1)] public float stiffness = 0.9f;

        public List<T> particles;

        public AngleConstraint(ref List<T> particles, bool doubleConnect = false)
        {
            constraintBatchType = EApexConstraintBatchType.AngleConstraint;
            this.particles = particles;

            
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
                var lToR = new ApexConstraintParticleThree(this.particles[i - 1].index, this.particles[i].index,
                    this.particles[i + 1].index);

                if (!constraints.ContainsKey(i))
                {
                    constraints.Add(i, new List<ApexConstraintParticleThree>());
                }

                constraints[i].Add(lToR);

                // Angle constraints often do not require a reverse connection
                if (doubleConnect)
                {
                    var rToL = new ApexConstraintParticleThree(this.particles[i + 1].index, this.particles[i].index,
                        this.particles[i - 1].index);

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
                    CalcParticleConstraint(ref particles[single.pl].nextPosition,
                        ref particles[single.pmid].nextPosition,
                        ref particles[single.pr].nextPosition,
                        particles[single.pl].isStatic,
                        particles[single.pmid].isStatic,
                        particles[single.pr].isStatic);
                }
            }
        }

        public void CalcParticleConstraint(ref Vector3 l, ref Vector3 mid, ref Vector3 r, bool lStatic, bool midStatic,
            bool rStatic)
        {
            // calc now angle
            Vector3 dirLMid = (mid - l).normalized;
            Vector3 dirRMid = (mid - r).normalized;
            // Avoiding potential NaN issues with Vector3.Dot and Mathf.Acos
            float dotProduct = Vector3.Dot(dirLMid, dirRMid);
            dotProduct = Mathf.Clamp(dotProduct, -1f, 1f); // Ensure dot product is within valid range [-1, 1]
            float currentAngle = Mathf.Acos(dotProduct);
            // float currentAngle = Mathf.Acos(Vector3.Dot(dirLMid, dirRMid));

            // calc error if angle
            float angleError = currentAngle - desiredAngle;

            // position calibration
            Vector3 correction = stiffness * angleError * (dirLMid + dirRMid);

            if (!lStatic)
            {
                l -= correction;
            }

            if (!midStatic)
            {
                // mid += correction;
            }

            if (!rStatic)
            {
                r += correction;
            }
        }
    }
}