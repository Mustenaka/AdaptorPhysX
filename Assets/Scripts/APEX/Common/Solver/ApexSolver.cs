using System;
using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Simulator;
using APEX.Tools;
using UnityEngine;

namespace APEX.Common.Solver
{
    /// <summary>
    /// Core Component: Apex Solver
    ///     A Time sequence controller that connects Unity to the execution of each physics simulator actor
    /// </summary>
    public class ApexSolver : MonoBehaviour
    {
        // simulator param
        public float dt = 0.001f;
        public float accTime;

        // particle container
        public List<ApexParticleBase> particles = new List<ApexParticleBase>();

        // simulator actors
        [SerializeReference] public List<IApexSimulatorBase> actors = new List<IApexSimulatorBase>();

        // delegate param action, use it to extend
        public Action actorStepBefore;
        public Action actorStepFinished;

        private void Update()
        {
            // time consequence control
            accTime += Time.deltaTime;
            var cnt = (int)(accTime / dt);

            // make sure time sequence is right
            actorStepBefore?.Invoke();
            for (var i = 0; i < cnt; i++)
            {
                var div = 0; // assign particles to difference simulate actor

                foreach (var actor in actors)
                {
                    actor.DoBeforeStepAction(div); // send particles status to solver (Delete it when you package)
                    actor.SyncParticleFromSolve(particles, div); // send solver particle to simulator
                    actor.Step(dt); // PBD step
                    actor.Complete(); // finish one step calc
                    actor.SyncParticleToSolver(particles, div); // get particle change from simulator
                    ParticleApply(); // apply particle calc
                    actor.DoAfterCompleteAction(div); // render solver particle to unity

                    div += actor.GetParticleCount(); // maybe there not only one actor
                }
            }

            actorStepFinished?.Invoke();

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
                    continue;
                }

                particles[i].previousPosition = particles[i].nowPosition;
                particles[i].nowPosition = particles[i].nextPosition;
            }
        }
    }
}