using System;
using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Tools;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace APEX.Common.Solver
{
    public class ApexSolver : MonoBehaviour
    {
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
        public float dt = 0.002f;
        public float accTime = 0.0f;
        public int iterator = 10;
        
        // run it by Jobs
        public bool runJobs = true;

        private void Update()
        {
            accTime += Time.deltaTime;
            int cnt = (int)(accTime / dt);

            for (int i = 0; i < cnt; i++)
            {
                if (runJobs)
                {
                    TestSimulator(); // burst
                }
                else
                {
                    Simulator(); // no burst
                }
            }

            accTime %= dt;
        }

        private void TestSimulator()
        {
            var simulateForceExtJob = new SimulateForceExtJob()
            {
                particles = new NativeArray<ApexParticleBaseBurst>(
                    particles.Select(p => p.ConvertBurst()).ToArray(),
                    Allocator.Persistent),
                gravity = gravity,
                globalForce = globalForce,
                airDrag = airDrag,
                damping = damping,
                dt = dt
            };
            
            for (int it = 0; it < iterator; it++)
            {
                // Do Force(in: Gravity, Local force, Global Force)
                // innerLoopBatchCount recommend multiples of 32 - i use 64
                var handle = simulateForceExtJob.Schedule(particles.Count, 128);
                handle.Complete();
                simulateForceExtJob.ParticleCallback(particles);

                // Do Constraint
                SimulateConstraint();

                // Update
                SimulateUpdate();
            }

            // finish simulator (by one dt )
            simulateForceExtJob.particles.Dispose();
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