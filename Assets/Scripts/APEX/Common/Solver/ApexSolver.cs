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
        public int iterator;

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
            // Do Gravite
            SimulateGravity();
            // Do Global ｜ Local Force
            
            // Do Constraint
            SimulateConstraint();
            // Update
            SimulateUpdate();
        }
        
        private void SimulateGravity()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // simplex pin
                if (particles[i].IsStatic)
                {
                    continue;
                }
                
                particles[i].NextPosition = particles[i].NowPosition 
                                            + (1 - damping) * (particles[i].NowPosition - particles[i].PreviousPosition)
                                            + gravity / particles[i].Mass * (dt * dt);
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
                if (particles[i].IsStatic)
                {
                    continue;
                }
                
                particles[i].PreviousPosition = particles[i].NowPosition;
                particles[i].NowPosition = particles[i].NextPosition;
            }
        }
    }
}