using System;
using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Tools;
using APEX.Tools.MathematicsTools;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Solver
{
    public class ApexSolver : MonoBehaviour
    {
        // runtime type
        public EApexSolverBackend backend;

        // particle and constraint param
        [SerializeReference] public List<IApexConstraintBatch> constraintBatch = new List<IApexConstraintBatch>();
        public List<ApexParticleBase> particles = new List<ApexParticleBase>(); // particle container

        // physics param
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public Vector3 globalForce = new Vector3(0, 0, 0);
        public float airDrag = 0.2f;
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.5f;

        // simulator param
        public float dt = 0.001f;
        public float accTime = 0.0f;
        public int iterator = 10;

        private void Update()
        {
            accTime += Time.deltaTime;
            int cnt = (int)(accTime / dt);

            for (int i = 0; i < cnt; i++)
            {
                switch (backend)
                {
                    case EApexSolverBackend.SingleThread:
                        Simulator();    // no burst simulator
                        break;
                    case EApexSolverBackend.JobsMultithreading:
                        TestSimulator();// burst simulator
                        break;
                    default:
                        Simulator();    // default: no burst simulator
                        break;
                }
            }

            accTime %= dt;
        }

        private void TestSimulator()
        {
            var simulateForceExtJob = new SimulateForceExtJob()
            {
                previousPosition = new NativeArray<float3>(
                    particles.Select(p => p.previousPosition.ToFloat3()).ToArray(),
                    Allocator.TempJob),
                nowPosition = new NativeArray<float3>(
                    particles.Select(p => p.nowPosition.ToFloat3()).ToArray(),
                    Allocator.TempJob),
                nextPosition = new NativeArray<float3>(particles.Count, Allocator.TempJob),

                isStatic = new NativeArray<bool>(
                    particles.Select(p => p.isStatic).ToArray(),
                    Allocator.TempJob),
                forceExt = new NativeArray<float3>(
                    particles.Select(p => p.forceExt.ToFloat3()).ToArray(),
                    Allocator.TempJob),
                mass = new NativeArray<float>(
                    particles.Select(p => p.mass).ToArray(),
                    Allocator.TempJob),

                gravity = gravity,
                globalForce = globalForce,
                airDrag = airDrag,
                damping = damping,
                dt = dt,
                iterator = iterator,
            };
            JobHandle sheduleJobDependency = new JobHandle();

            // Do Force(in: Gravity, Local force, Global Force)
            // innerLoopBatchCount recommend multiples of 32 - i use 64
            var handle =
                simulateForceExtJob.ScheduleParallel(simulateForceExtJob.nowPosition.Length, 5, sheduleJobDependency);
            handle.Complete();
            simulateForceExtJob.ParticleCallback(particles);

            // Do Constraint
            SimulateConstraint();

            // Update
            SimulateUpdate();
        }

        private void Simulator()
        {
            for (int i = 0; i < iterator; i++)
            {
                // Do Force(in: Gravity, Local force, Global Force)
                SimulateForceExt();

                // Do Constraint
                SimulateConstraint();

                // Update
                SimulateUpdate();
            }
        }

        private void SimulateForceExt()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // simplex pin
                if (particles[i].isStatic)
                {
                    continue;
                }

                // calc air resistance
                Vector3 airResistance = -airDrag * (particles[i].nowPosition - particles[i].previousPosition) / dt;

                // calc force apply.
                particles[i].forceApply = gravity + globalForce + airResistance + particles[i].forceExt;
                particles[i].nextPosition = particles[i].nowPosition
                                            + (1 - damping) * (particles[i].nowPosition - particles[i].previousPosition)
                                            + particles[i].forceApply / particles[i].mass * (dt * dt);
            }
        }

        private void SimulateConstraint()
        {
            foreach (var constraint in constraintBatch)
            {
                constraint.Do();
            }
        }

        private void SimulateUpdate()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // if particle is static, not apply
                if (particles[i].isStatic)
                {
                    continue;
                }

                // if the nextPosition is NaN, will not apply
                if (NumberCheck.IsVector3NaN(particles[i].nextPosition))
                {
                    particles[i].nextPosition = particles[i].nowPosition;
                }

                particles[i].previousPosition = particles[i].nowPosition;
                particles[i].nowPosition = particles[i].nextPosition;
            }
        }
    }
}