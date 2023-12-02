using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Solver
{
    public class ApexSolver: MonoBehaviour 
    {
        public List<IApexConstraintBatch> constraintBatch = new List<IApexConstraintBatch>();
        public List<ApexParticleBase> particles = new List<ApexParticleBase>();
            
        // physics param
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.5f;
        
        // simulator param
        public float dt = 0.002f;
        public float accTime;
        public int iterator = 10;

        private void Update()
        {
            accTime += Time.deltaTime;
            int cnt = (int)(accTime / dt);

            for (int i = 0; i < cnt; i++)
            {
                Simulator();
            }

            accTime %= dt;
        }

        private void Simulator()
        {
            for (int i = 0; i < iterator; i++)
            {
                // Do Gravite
                SimulateGravity();
                
                // Do Global ｜ Local Force
            
                // Do Constraint
                SimulateConstraint();
                
                // Update
                SimulateUpdate();
            }
        }
        
        private void SimulateGravity()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // simplex pin
                if (particles[i].isStatic)
                {
                    continue;
                }
                
                particles[i].nextPosition = particles[i].nowPosition 
                                            + (1 - damping) * (particles[i].nowPosition - particles[i].previousPosition)
                                            + gravity / particles[i].mass * (dt * dt);
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
                if (particles[i].isStatic)
                {
                    continue;
                }
                
                particles[i].previousPosition = particles[i].nowPosition;
                particles[i].nowPosition = particles[i].nextPosition;
            }
        }
    }
}