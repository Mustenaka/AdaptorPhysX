using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using APEX.Common.Constraints.Base;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Constraints
{
    public class DistanceConstraint<T> : ApexConstraintBatchBase<ApexParticleConstraintBase> where T : ApexParticleBase
    {
        // the min\max distance between one particle and the other.
        public float minDistance;
        public float maxDistance;
        
        // Relaxation Parameter
        public float stiffness = 1.0f;
        
        // particle group
        public List<T> particles;
        
        public DistanceConstraint(ref List<T> particles)
        {
            constraintBatchType = EApexConstraintBatchType.DistanceConstraint;
            this.particles = particles;
        }
        
        public override void Do()
        {
            foreach (var constraint in constraints)
            {
                foreach (var single in constraint.Value)
                {
                    var posL = particles[single.pl].nextPosition;
                    var posR = particles[single.pr].nextPosition;
                    CalcParticleConstraint(ref posL, ref posR);
                }
            }
        }

        public void CalcParticleConstraint(ref Vector3 l, ref Vector3 r)
        {
            var delta = l - r;
            float currentDistance = delta.magnitude;
            float error = Mathf.Clamp(currentDistance, minDistance, maxDistance) - currentDistance;
            
            if (currentDistance > Mathf.Epsilon)
            {
                Vector3 correction = (error / currentDistance) * delta * 0.5f * stiffness;
                l -= correction;
                r += correction;
            }
        }
    }
}