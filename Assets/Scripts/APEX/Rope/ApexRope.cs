using System.Collections.Generic;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Rope
{
    public class ApexRope : MonoBehaviour
    {
        public GameObject element;  // TO-DO: Use Material replace it.
        
        public int particlesCount;

        public List<Vector3> originStatus;
        public List<Vector3> naturalPotentialEnergyRestingPosition;

        public List<ApexLineParticle> particles;
        // public List<List<ApexParticleConstraintBase>> constraints;
        
        // physics param
        [Range(0, 1f)] public float stiffness;
        [Range(0, 1f)] public float damping;
    }
}