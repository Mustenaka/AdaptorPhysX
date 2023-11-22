using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Solver;
using UnityEngine;

namespace APEX.Rope
{
    public class ApexRope : MonoBehaviour
    {
        public List<GameObject> elements = new List<GameObject>();  // TO-DO: Use Material or something else to replace it.
        public List<ApexParticleBase> particles = new List<ApexParticleBase>();

        private int ParticlesCount => particles.Count;
        
        public ApexSolver solver;
        
        private void Update()
        {
            for (int i = 0; i < ParticlesCount; i++)
            {
                elements[i].transform.position = particles[i].nowPosition;
            }
        }
    }
}