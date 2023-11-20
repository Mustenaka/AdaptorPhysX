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

        public GameObject obj;
        
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

        private void InitRope()
        {
            for (int i = 0; i < particleCount; i++)
            {
                Vector3 particlePosition = firstParticlePosition + i * (stepSize * stepDirect);
                
                var particle = GameObject.Instantiate(obj, transform, true);
                particle.name = i.ToString();
                particle.transform.position = particlePosition;
            }
        }
    }
}