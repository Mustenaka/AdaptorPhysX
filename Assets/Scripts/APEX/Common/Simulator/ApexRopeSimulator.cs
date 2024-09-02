using System;
using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Force;
using APEX.Common.Particle;
using APEX.Tools.Extend;
using APEX.Usage;
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
        public NativeArray<float3> originPosition;
        public NativeArray<float3> previousPosition;
        public NativeArray<float3> nowPosition;
        public NativeArray<float3> nextPosition;

        // particle physics param
        public NativeArray<float> mass;
        public NativeArray<float3> forceExt;
        public NativeArray<float3> forceFrameExt;

        // particle constraint type
        public NativeArray<EApexParticleConstraintType> constraintTypes;

        // constraint connect param： rope is double connect
        public NativeArray<ApexConstraintParticleDouble> doubleConnect;
        public NativeArray<float> distancelambdas; // Add this to store the Lagrange multipliers: default is 0

        // constraint connect param: bend constraint
        public NativeArray<ApexConstraintParticleThree> bendConnect;
        public NativeArray<float> bendLambdas;

        // particle pin(Attachment)
        public NativeArray<ApexPinConstraint> pin;

        // switch constraint
        public bool useForce = true;
        public bool useDistanceConstraint = true;
        public bool useBendConstraint = true;
        public bool useColliderConstraint = true;
        public bool usePinConstraint = true;

        // Distance Constraint Param
        public float restLength = 1.2f;
        public float stiffness = 0.5f;
        public float compliance = 0.0001f; // a compliance parameter

        // physics param - force
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public Vector3 globalForce = new Vector3(0, 0, 0);
        public float airDrag = 0.2f;
        public float damping = 0.005f;

        // simulator param
        public int iterator = 1;
        public float dt;

        // action param
        public Action<int> beforeStep = delegate { };
        public Action<int> afterComplete = delegate { };

        private JobHandle _jobHandle;

        /// <summary>
        /// consider pin, iterator, collider... do the final constraint
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoFinalConstraintJobs(JobHandle depend)
        {
            var finalConstraintJob = new FinalConstraintJob()
            {
                position = nextPosition,
                particleConstraintTypes = constraintTypes,

                pin = pin,
                dt = dt,
                localForce = forceFrameExt,
                masses = mass,
                stiffness = stiffness,
                restLength = restLength,
                particleForce = forceExt,
            };
            return usePinConstraint ? finalConstraintJob.Schedule(nowPosition.Length, depend) : depend;
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
            // PBD
            // var distanceConstraintJob = new DistanceConstraintJob()
            // {
            //     nextPosition = nextPosition,
            //     constraints = doubleConnect,
            //
            //     restLength = restLength,
            //     stiffness = 0.98f,
            //
            //     masses = mass,
            //     d = d,
            // };

            // XPBD
            var distanceConstraintJob = new XDistanceConstraintJob()
            {
                nextPosition = nextPosition,
                constraints = doubleConnect,

                restLength = restLength,
                stiffness = stiffness,

                masses = mass,
                d = d,

                lagrangeMultipliers = distancelambdas,
                compliance = compliance,
                deltaTime = dt,
            };
            return useDistanceConstraint ? distanceConstraintJob.Schedule(nextPosition.Length, depend) : depend;
        }

        /// <summary>
        /// do bend constraint jobs
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <param name="iterIndex">the iterator index</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoBendConstraintJobs(JobHandle depend, int iterIndex)
        {
            var bendConstraintJob = new XBendConstraintJob()
            {
                nextPosition = nextPosition,
                bendConstraints = bendConnect,
                masses = mass,

                restAngle = math.PI,
                compliance = compliance,
                dt = dt,

                lagrangeMultipliers = bendLambdas,
            };
            // return depend;
            return useBendConstraint ? bendConstraintJob.Schedule(nextPosition.Length, depend) : depend;
        }

        /// <summary>
        /// do force effect and predict particle next position.
        /// </summary>
        /// <returns>job handle depend</returns>
        private JobHandle DoForceJobs()
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
            return useForce ? job.ScheduleParallel(job.nowPosition.Length, 64, depend) : depend;
        }

        /// <summary>
        /// Do all collider jobs
        /// </summary>
        /// <param name="depend">job handle depend</param>
        /// <returns>job handle depend</returns>
        private JobHandle DoColliderJobs(JobHandle depend)
        {
            return depend;
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
                jobDepend = DoBendConstraintJobs(jobDepend, i);
                jobDepend = DoFinalConstraintJobs(jobDepend);
            }

            return jobDepend;
        }

        /// <summary>
        /// Do Step:
        ///     1. predict next position, by Varlet integral
        ///     2. Collider TODO: finish it
        ///     3. revise next position, by distance &
        /// </summary>
        /// <param name="_dt">delta time</param>
        public void Step(float _dt)
        {
            this.dt = _dt;

            var handle = DoForceJobs(); // 1. predict next position
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
        /// Sync pin constraint
        /// </summary>
        /// <param name="pins"></param>
        public void SyncPinConstraint(List<ApexPin> pins)
        {
            for (var i = 0; i < pins.Count; i++)
            {
                pin[pins[i].particleIndex] = new ApexPinConstraint(pins[i].pinPosition);
            }
        }

        /// <summary>
        /// Sync drag contsraints
        /// </summary>
        /// <param name="drags"></param>
        public void SyncDragConstraint(List<ApexPin> drags)
        {
            for (int i = 0; i < drags.Count; i++)
            {
                // var point = pin[drags[i].particleIndex];
                drags[i].executeForce = forceFrameExt[drags[i].particleIndex];
                drags[i].duration = dt;
            }

            distancelambdas.Clean();
            bendLambdas.Clean();
            forceFrameExt.Clean();
        }

        /// <summary>
        /// Do something before Step() | run at ApexSolver
        /// </summary>
        public void DoBeforeStepAction(int cnt)
        {
            beforeStep?.Invoke(cnt);
        }

        /// <summary>
        /// Do something after Complete() | run at ApexSolver
        /// </summary>
        public void DoAfterCompleteAction(int cnt)
        {
            afterComplete?.Invoke(cnt);
        }

        /// <summary>
        /// sync position from solve
        /// </summary>
        /// <param name="particles">all solver particle</param>
        /// <param name="down">particle lower bound</param>
        public void SyncParticleFromSolve(List<ApexParticleBase> particles, int down)
        {
            var cnt = down;
            for (var i = 0; i < nowPosition.Length; i++, cnt++)
            {
                previousPosition[i] = particles[cnt].previousPosition;
                nowPosition[i] = particles[cnt].nowPosition;
                nextPosition[i] = particles[cnt].nextPosition;
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
            for (var i = 0; i < nowPosition.Length; i++, cnt++)
            {
                particles[cnt].previousPosition = previousPosition[i];
                particles[cnt].nowPosition = nowPosition[i];
                particles[cnt].nextPosition = nextPosition[i];
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

        /// <summary>
        /// Dispose resource
        /// </summary>
        public void Dispose()
        {
            originPosition.Dispose();
            previousPosition.Dispose();
            nowPosition.Dispose();
            nextPosition.Dispose();

            mass.Dispose();
            forceExt.Dispose();
            forceFrameExt.Dispose();

            constraintTypes.Dispose();

            doubleConnect.Dispose();
            distancelambdas.Dispose();

            bendConnect.Dispose();
            bendLambdas.Dispose();

            pin.Dispose();
        }
    }
}