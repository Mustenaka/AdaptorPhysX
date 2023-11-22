using System;
using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Solver;
using UnityEngine;

namespace APEX.Rope
{
    public class ApexRope : MonoBehaviour
    {
        public List<GameObject> elements;  // TO-DO: Use Material or something else to replace it.
        public List<ApexLineParticle> particles;

        private int ParticlesCount => particles.Count;

        public List<Vector3> originStatus;
        public List<Vector3> naturalPotentialEnergyRestingPosition;
        
        public ApexSolver<ApexLineParticle> solver;
        
        private void Update()
        {
            for (int i = 0; i < ParticlesCount; i++)
            {
                elements[i].transform.position = particles[i].nowPosition;
            }
        }
    }
}