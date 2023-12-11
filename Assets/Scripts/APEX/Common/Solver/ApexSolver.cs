using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Tools;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace APEX.Common.Solver
{
    public class ApexSolver: MonoBehaviour 
    {
        [SerializeReference] public List<IApexConstraintBatch> constraintBatch = new List<IApexConstraintBatch>();
        public List<ApexParticleBase> particles = new List<ApexParticleBase>();     // particle container
            
        // physics param
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        public float airDrag = 0.2f;
        public Vector3 airVelocity = new Vector3(0, 0, 0);
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
                // Do Force
                SimulateGravity();
                
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

                particles[i].forceExt += gravity;
                particles[i].nextPosition = particles[i].nowPosition 
                                            + (1 - damping) * (particles[i].nowPosition - particles[i].previousPosition)
                                            + particles[i].forceExt / particles[i].mass * (dt * dt);
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