using System;
using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Simulator;
using APEX.Tools;
using Temp.JobTest;
using UnityEngine;

namespace APEX.Common.Solver
{
    /// <summary>
    /// All Solver
    /// </summary>
    public class ApexSolver : MonoBehaviour
    {
        // particle and constraint param
        public List<ApexParticleBase> particles = new List<ApexParticleBase>(); // particle container

        // simulator param
        public float dt = 0.001f;
        public float accTime;

        // simulator actors
        [SerializeReference] public List<IApexSimulatorBase> actors = new List<IApexSimulatorBase>();

        // delegate param
        public Action particleSend;

        private void Update()
        {
            // time consequence control
            accTime += Time.deltaTime;
            var cnt = (int)(accTime / dt);

            for (var i = 0; i < cnt; i++)
            {
                int div = 0;
                foreach (var actor in actors)
                {
                    particleSend?.Invoke(); // from render change
                    actor.SyncParticleFromSolve(particles, div); // from solve calc
                    actor.Step(dt); // PBD step

                    if (i < cnt - 1)
                    {
                        actor.Complete(); // finish one step calc
                        actor.SyncParticleToSolver(particles, div); // get particle position change
                        ParticleApply(); // apply particle calc
                    }

                    div += actor.GetParticleCount(); // maybe there not only one actor
                }
            }

            accTime %= dt;
        }

        private void LateUpdate()
        {
            foreach (var actor in actors)
            {
                actor.Complete(); // last complete per frame
                ParticleApply();
            }
        }

        /// <summary>
        /// Apply particle from PBD Simulator
        /// </summary>
        private void ParticleApply()
        {
            for (var i = 0; i < particles.Count; i++)
            {
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