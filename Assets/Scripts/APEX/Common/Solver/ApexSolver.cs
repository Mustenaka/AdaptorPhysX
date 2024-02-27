using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Common.Simulator;
using APEX.Tools;
using APEX.Tools.MathematicsTools;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Solver
{
    /// <summary>
    /// All Solver
    /// </summary>
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
        public float dt = 0.001f;
        public float accTime;
        public int iterator = 10;

        // simulator actors
        public List<ApexSimulatorBase> actors;

        private void Update()
        {
            // time consequence control
            accTime += Time.deltaTime;
            var cnt = (int)(accTime / dt);

            for (var i = 0; i < cnt; i++)
            {
                foreach (var actor in actors)
                {
                    Invoke();
                    actor.Step(dt);
                    if (i < cnt - 1)
                    {
                        actor.Complete();
                    }
                }
            }

            accTime %= dt;
        }

        private void LateUpdate()
        {
            foreach (var actor in actors)
            {
                actor.Complete(); // last complete per frame
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