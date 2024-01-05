using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using Unity.Collections;
using Unity.Jobs;

namespace APEX.Common.Solver
{
    /// <summary>
    /// Use it for ApexSolver
    /// </summary>
    public class ApexSimJob
    {
        private JobHandle _jobHandle;

        private NativeArray<SimulateForceExtJob> _simulateForce;

        /* distance constraint */
        private NativeArray<DistanceConstraintJob> _distanceConstraint;
        
        /* bend constraint */
        // private NativeArray<>
        
        /* pin constraint */
        
        public ApexSimJob(List<ApexParticleBase> particles, List<IApexConstraintBatch> constraintBatch)
        {
            
        }
    }
}