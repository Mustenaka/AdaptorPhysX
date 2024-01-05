using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace APEX.Common.Solver
{
    /// <summary>
    /// Use it for ApexSolver
    /// </summary>
    public class ApexSimJob
    {
        private JobHandle _jobHandle;

        private NativeArray<ApexParticleBaseBurst> _particles;

        /* distance constraint */
        private NativeArray<DistanceConstraintJob> _distanceConstraint;
        
        /* bend constraint */
        
        /* pin constraint */
        
        public ApexSimJob(List<ApexParticleBase> particles, List<IApexConstraintBatch> constraintBatch)
        {
            /* generate jobs persistence particle : from input particles */
            _particles = new NativeList<ApexParticleBaseBurst>(particles.Count, Allocator.Persistent);
            for (var i = 0; i < _particles.Length; i++)
            {
                _particles[i] = new ApexParticleBaseBurst(particles[i]);
            }
            
            /* generate job persistence constraint: from input constraint */
            foreach (var con in constraintBatch)
            {
                Debug.Log(con.GetConstraintType());
            }
        }
    }
}