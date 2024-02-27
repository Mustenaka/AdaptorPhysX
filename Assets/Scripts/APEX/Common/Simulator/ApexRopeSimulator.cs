using System;
using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Common.Solver;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Simulator
{
    /// <summary>
    /// Rope Simulator actor
    /// </summary>
    [Serializable]
    public class ApexRopeSimulator : IApexSimulatorBase
    {
        // particle param
        public NativeArray<float3> originPosition = new NativeArray<float3>();
        public NativeArray<float3> previousPosition = new NativeArray<float3>();
        public NativeArray<float3> nowPosition = new NativeArray<float3>();
        public NativeArray<float3> nextPosition = new NativeArray<float3>();

        // particle physics param
        public NativeArray<float> mass = new NativeArray<float>();
        public NativeArray<float3> forceExt = new NativeArray<float3>();

        // particle constraint type
        public NativeArray<EApexParticleConstraintType>
            constraintTypes = new NativeArray<EApexParticleConstraintType>();

        // constraint connect param： rope is double connect
        public NativeArray<ApexConstraintParticleDouble>
            doubleConnect = new NativeArray<ApexConstraintParticleDouble>();

        // particle pin(Attachment)
        public NativeArray<ApexPinConstraint> pin = new NativeArray<ApexPinConstraint>();

        // switch constraint
        public bool useForce = true;
        public bool useDistanceConstraint = true;
        public bool useColliderConstraint = true;
        
        // Distance Constraint Param
        public float restLength = 1.2f;
        public float stiffness = 0.5f;
        
        // Collider Constraint Param

        // physics param - force
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public Vector3 globalForce = new Vector3(0, 0, 0);
        public float airDrag = 0.2f;
        public float damping = 0.005f;

        // simulator param
        public int iterator = 1;

        private JobHandle _jobHandle;

        /// <summary>
        /// consider pin, iterator, collider... do the final constraint
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoFinalConstraintJobs(JobHandle depend)
        {
            // float d = 1.0f / (iterator - iterIndex);
            var finalConstraintJob = new FinalConstraintJob()
            {
                position = nextPosition,

                particleConstraintTypes = constraintTypes,

                pin = pin,
            };
            return finalConstraintJob.Schedule(nowPosition.Length, depend);
        }

        /// <summary>
        /// do distance constraint jobs
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <param name="iterIndex">the iterator index</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoDistanceConstraintJobs(JobHandle depend, int iterIndex)
        {
            var d = 1.0f / (iterator - iterIndex);
            var distanceConstraintJob = new DistanceConstraintJob()
            {
                nextPosition = nextPosition,
                constraints = doubleConnect,

                restLength = restLength,
                stiffness = stiffness,

                masses = mass,
                d = d,
            };
            return distanceConstraintJob.Schedule(distanceConstraintJob.constraints.Length, depend);
        }

        /// <summary>
        /// do all constraint jobs
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoConstraintJobs(JobHandle depend)
        {
            JobHandle jobDepend = depend;
            for (var i = 0; i < iterator; i++)
            {
                jobDepend = DoDistanceConstraintJobs(jobDepend, i);
                jobDepend = DoFinalConstraintJobs(jobDepend);
            }

            return jobDepend;
        }

        /// <summary>
        /// do force effect and predict particle next position.
        /// </summary>
        /// <param name="dt">delta time</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoForceJobs(float dt)
        {
            var job = new ForceJob
            {
                previousPosition = previousPosition,
                nowPosition = nowPosition,
                nextPosition = nextPosition,

                mass = mass,

                forceExt = forceExt,
                gravity = gravity,
                globalForce = globalForce,

                airDrag = airDrag,
                damping = damping,

                dt = dt,
            };

            var depend = new JobHandle();
            return job.ScheduleParallel(job.nowPosition.Length, 64, depend);
        }

        /// <summary>
        /// Do Step:
        ///     1. predict next position, by Varlet integral
        ///     2. Collider TODO: finish it
        ///     3. revise next position, by distance &
        /// </summary>
        /// <param name="dt">delta time</param>
        public void Step(float dt)
        {
            var handle = DoForceJobs(dt); // 1. predict next position
            // TODO: 2. collider constraint.. 
            handle = DoConstraintJobs(handle); // 3. revise next position
            _jobHandle = handle;
        }

        /// <summary>
        /// Complete all jobs
        /// </summary>
        public void Complete()
        {
            _jobHandle.Complete();
        }

        /// <summary>
        /// sync position from solve
        /// </summary>
        /// <param name="particles">all solver particle</param>
        /// <param name="down">particle lower bound</param>
        public void SyncParticleFromSolve(List<ApexParticleBase> particles, int down)
        {
            var cnt = down;
            for (var i = 0; i < nowPosition.Length; i++)
            {
                previousPosition[i] = particles[cnt].previousPosition;
                nowPosition[i] = particles[cnt].nowPosition;
                nextPosition[i] = particles[cnt].nextPosition;
                cnt++;
            }
        }

        /// <summary>
        /// sync particle to solve
        /// </summary>
        /// <param name="particles">all solver particle</param>
        /// <param name="down">particle lower bound</param>
        public void SyncParticleToSolver(List<ApexParticleBase> particles, int down)
        {
            var cnt = down;
            for (var i = 0; i < nowPosition.Length; i++)
            {
                particles[cnt].previousPosition = previousPosition[i];
                particles[cnt].nowPosition = nowPosition[i];
                particles[cnt].nextPosition = nextPosition[i];
                cnt++;
            }
        }

        /// <summary>
        /// how many particle this simulator actor have
        /// </summary>
        /// <returns></returns>
        public int GetParticleCount()
        {
            return this.nowPosition.Length;
        }
    }
}