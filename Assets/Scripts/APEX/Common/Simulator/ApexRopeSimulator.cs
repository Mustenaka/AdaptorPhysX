using APEX.Common.Constraints;
using APEX.Common.Solver;
using Unity.Jobs;
using Unity.Mathematics;

namespace APEX.Common.Simulator
{
    public class ApexRopeSimulator : ApexSimulatorBase
    {
        // physics param
        public float3 gravity = new float3(0, -9.81f, 0);
        public float3 globalForce = new float3(0, 0, 0);
        public float airDrag = 0.2f;
        public float stiffness = 0.5f;
        public float damping = 0.5f;

        // simulator param
        public int iterator = 10;

        private JobHandle _jobHandle;

        private JobHandle DoDistanceConstraintJobs(JobHandle depend, float dt)
        {
            var distanceConstraintJob = new DistanceConstraintJob()
            {
            };
            return distanceConstraintJob.Schedule(distanceConstraintJob.constraints.Length, depend);
        }

        // private JobHandle DoColliderJobs(JobHandle depend, float dt)
        // {
        //     
        // }
        
        private JobHandle DoConstraintJobs(JobHandle depend, float dt)
        {
            JobHandle jobDepend = depend;
            for (var i = 0; i < iterator; i++)
            {
                jobDepend = DoDistanceConstraintJobs(jobDepend, dt);
            }

            return jobDepend;
        }

        private JobHandle DoForceJobs(float dt)
        {
            var job = new SimulateForceExtJob
            {
                dt = dt
            };

            var depend = new JobHandle();
            return job.ScheduleParallel(job.nowPosition.Length, 64, depend);
        }

        /// <summary>
        /// Do Step:
        ///     1. predict next position, by Varlet integral
        ///     2. revise next position, by distance &
        ///     3. Collider
        /// </summary>
        /// <param name="dt"></param>
        public override void Step(float dt)
        {
            var handle = DoForceJobs(dt);           // predict next position
            handle = DoConstraintJobs(handle, dt);  // revise next position
            _jobHandle = handle;
        }

        /// <summary>
        /// Complete all jobs
        /// </summary>
        public override void Complete()
        {
            _jobHandle.Complete();
        }
    }
}