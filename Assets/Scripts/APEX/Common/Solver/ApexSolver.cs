using System;
using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Simulator;
using APEX.Tools;
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

        // delegate param action
        public Action<int> actorStepBefore;
        public Action<int> actorStepFinished;

        private void Update()
        {
            // time consequence control
            accTime += Time.deltaTime;
            var cnt = (int)(accTime / dt);

            // make sure time sequence is right
            for (var i = 0; i < cnt; i++)
            {
                var div = 0;

                foreach (var actor in actors)
                {
                    // Not repeat invoke. change function for interface
                    actorStepBefore.Invoke(div); // from render change

                    actor.SyncParticleFromSolve(particles, div); // send solver particle to simulator
                    actor.Step(dt); // PBD step
                    actor.Complete(); // finish one step calc
                    actor.SyncParticleToSolver(particles, div); // get particle change from simulator
                    ParticleApply(); // apply particle calc

                    actorStepFinished.Invoke(div);
                    div += actor.GetParticleCount(); // maybe there not only one actor
                }
            }

            accTime %= dt;
        }

        private void OnDestroy()
        {
            for (var i = 0; i < actors.Count; i++)
            {
                if (actors[i] != null)
                {
                    actors[i].Dispose();
                }
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