using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Solver
{
    public class ApexSolver<T1> : MonoBehaviour 
        where T1: ApexParticleBase
    {
        public List<IApexConstraintBatch> constraintBatch;
        public List<T1> particles;

        // physics param
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.5f;
        
        // simulator param
        public float dt = 0.002f;
        public float accTime;
        public int iterator;

        // private void Awake()
        // {
        //     constraintBatch = new List<IApexConstraintBatch>();
        //     particles = new List<T1>();
        // }

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
            SimulateGravite();
            // Do Constraint
            SimulateConstraint();
            // Update
            SimulateUpdate();
        }
        
        private void SimulateGravite()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                particles[i].nextPosition = particles[i].nowPosition 
                                            + (1 - damping) * (particles[i].nowPosition - particles[i].previousPosition)
                                            + gravity * (dt * dt);
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
                particles[i].previousPosition = particles[i].nowPosition;
                particles[i].nowPosition = particles[i].nextPosition;
            }
        }
    }
}