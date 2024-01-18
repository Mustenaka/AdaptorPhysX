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

        // pin(attach) point
        public NativeArray<int> pinIndex;

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
                        SingleSimulator(); // no burst simulator
                        break;
                    case EApexSolverBackend.JobsMultithreading:
                        BurstSimulator(); // burst simulator
                        break;
                    default:
                        SingleSimulator(); // default: no burst simulator
                        break;
                }
            }

            accTime %= dt;
        }

        private void BurstSimulator()
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
                pinIndex = pinIndex,
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
            JobsSimulateConstraint();

            // Update
            SimulateUpdate();
        }

        // TEMP: test code for jobs constraint test.
        public NativeParallelHashMap<int, NativeArray<ApexConstraintParticleDouble>> cons;

        /// <summary>
        /// Line Constructor:
        ///     particle one by one connect.
        /// </summary>
        /// <param name="doubleConnect">Reverse connection (2x)</param>
        public void LineConstructor(bool doubleConnect = true)
        {
            var length = particles.Count;
            Debug.Log(length);
            /*
             * NativeArray<ApexConstraintParticleDouble> 因为没有创建所以，不是固定内存，
             * unity job认为不占据内存就没有创建空间，因此产生报错
             * ArgumentException: Unity.Collections.NativeArray`1[APEX.Common.Constraints.ApexConstraintParticleDouble]
             * used in native collection is not blittable, not primitive, or contains a type tagged as NativeContainer
             * 解决方案想法：
             *      1. 看看能不能像c/c++ setmem的方式人工分配内存空间
             *      2. 需要修改数据结构，从Hash方法降级为Line方法，这样会增加约束定位的时间
             */
            cons =
                new NativeParallelHashMap<int, NativeArray<ApexConstraintParticleDouble>>(length, Allocator.Persistent);
            for (int i = 0; i < length - 1; i++)
            {
                var lToR = new ApexConstraintParticleDouble(particles[i].index, particles[i + 1].index);
                var rToL = new ApexConstraintParticleDouble(particles[i + 1].index, particles[i].index);

                Debug.Log("asdasdasd");

                // // Do not use ??= expression in Unity
                // if (!cons.ContainsKey(i))
                // {
                //     // In general, the length constraint to which a particle is connected is at most 8 (surface body).
                //     cons.Add(i, new NativeArray<ApexConstraintParticleDouble>(8, Allocator.Persistent));
                // }
                //
                // if (!cons.ContainsKey(i + 1))
                // {
                //     cons.Add(i + 1, new NativeArray<ApexConstraintParticleDouble>(8, Allocator.Persistent));
                // }
                //
                // // Tail-in data
                // var apexConstraintParticleDoubles = cons[i];
                // apexConstraintParticleDoubles[apexConstraintParticleDoubles.Length] = lToR;
                //
                // if (doubleConnect)
                // {
                //     var constraintParticleDoubles = cons[i];
                //     constraintParticleDoubles[constraintParticleDoubles.Length] = rToL;
                // }
            }

            foreach (var con in cons)
            {
                string one = con.Key + " : ";
                foreach (var single in con.Value)
                {
                    one += "(" + single.pl + ", " + single.pr + ")";
                }

                Debug.Log(one);
            }
        }

        private void JobsSimulateConstraint()
        {
            JobHandle sheduleJobDependency = new JobHandle();

            foreach (var constraint in constraintBatch)
            {
                var typeOfConstraint = constraint.GetConstraintType();

                switch (typeOfConstraint)
                {
                    case EApexConstraintBatchType.DistanceConstraint:
                        var distanceConstraint = constraint as DistanceConstraint;

                        var distanceConstraintJob = new DistanceConstraintJob()
                        {
                            restLength = distanceConstraint.restLength,
                            stiffness = distanceConstraint.stiffness,
                            pinIndex = pinIndex,
                            constraints = cons,

                            particlesNextPosition = new NativeArray<float3>(
                                particles.Select(p => p.nowPosition.ToFloat3()).ToArray(),
                                Allocator.TempJob),
                            particlesAdjustNextPosition = new NativeArray<float3>(particles.Count, Allocator.TempJob),
                        };

                        var handle = distanceConstraintJob.ScheduleParallel(
                            distanceConstraintJob.particlesNextPosition.Length, 5, sheduleJobDependency);
                        handle.Complete();
                        distanceConstraintJob.ParticleCallback(particles);

                        break;
                }
            }
        }

        private void SingleSimulator()
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