using System.Collections.Generic;
using APEX.Common.Particle;
using Unity.VisualScripting;
using UnityEngine;

namespace APEX.Rope
{
    public class ApexRopeCreate : MonoBehaviour
    {
        public bool useSelfPosition;
        public Vector3 firstParticlePosition = Vector3.zero;
        
        public int particleCount = 10;
        
        public float stepSize = 1.2f;
        public Vector3 stepDirect = Vector3.left;
        
        public GameObject obj;      // TO-DO: Use Material replace it.
        
        // physics param
        public float mass = 1.0f;      // If you want the centroid offset, please change this generation method
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.5f;
        
        private void Start()
        {
            // if the obj is empty, create sphere to fill it
            if (obj == null)
            {
                obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            }
            
            // is use self position, the firstParticlePosition is self
            if (useSelfPosition)
            {
                firstParticlePosition = this.transform.position;
            }
            
            InitRope();
        }

        /// <summary>
        /// Init particle of rope
        /// </summary>
        private void InitRope()
        {
            var rope = this.AddComponent<ApexRope>();
            rope.element = obj;
            rope.particlesCount = particleCount;
            rope.particles = new List<ApexLineParticle>();
            rope.stiffness = stiffness;
            rope.damping = damping;
            
            for (int i = 0; i < particleCount; i++)
            {
                Vector3 particlePosition = firstParticlePosition + i * (stepSize * stepDirect);
                
                var particle = GameObject.Instantiate(obj, transform, true);
                particle.name = i.ToString();
                particle.transform.position = particlePosition;
                
                ApexLineParticle p = new ApexLineParticle
                {
                    index = i,
                    mass = mass,
                    previousPosition = particlePosition,
                    position = particlePosition,
                    rotation = Quaternion.Euler(0,0,0),
                    scale = this.transform.localScale
                };

                rope.particles.Add(p);
            }
        }
    }
}