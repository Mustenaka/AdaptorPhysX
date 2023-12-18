using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Common.Solver;
using Unity.VisualScripting;
using UnityEngine;

namespace APEX.Rope
{
    /// <summary>
    /// rope create
    /// </summary>
    public class ApexRopeCreate : MonoBehaviour
    {
        public bool useSelfPosition;
        public Vector3 firstParticlePosition = Vector3.zero;

        public int particleCount = 10;
        public float stepSize = 1.2f;
        public Vector3 stepDirect = Vector3.left;

        public GameObject obj; // TO-DO: Use Material replace it.

        // default physics param(only use for create)
        public float mass = 1.0f; // If you want the centroid offset, please change this generation method
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.005f;
        [Range(1, 20)] public int iterator = 10;

        private void Start()
        {
            // if the obj is empty, create sphere to fill it
            if (obj == null)
            {
                obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                Destroy(obj);
            }

            // is use self position, the firstParticlePosition is self
            if (useSelfPosition)
            {
                firstParticlePosition = this.transform.position;
            }

            // Init Rope
            InitRope();
        }

        /// <summary>
        /// Init particle of rope
        /// </summary>
        private void InitRope()
        {
            var rope = this.AddComponent<ApexRope>();
            var solver = this.AddComponent<ApexSolver>();

            rope.solver = solver;

            rope.particles = new List<ApexLineParticle>();
            rope.elements = new List<GameObject>();

            for (int i = 0; i < particleCount; i++)
            {
                Vector3 particlePosition = firstParticlePosition + i * (stepSize * stepDirect);

                var element = GameObject.Instantiate(obj, transform, true);
                element.name = i.ToString();
                element.transform.position = particlePosition;

                ApexLineParticle p = new ApexLineParticle
                {
                    index = i,
                    mass = mass,
                    previousPosition = particlePosition,
                    nowPosition = particlePosition,
                    forceExt = Vector3.zero,
                    
                    scale = this.transform.localScale
                };

                // static the first particle
                if (i == 0)
                {
                    p.isStatic = true;
                }

                rope.elements.Add(element);
                rope.particles.Add(p);
            }

            rope.solver.particles = new List<ApexParticleBase>(rope.particles);
            rope.solver.stiffness = stiffness;
            rope.solver.damping = damping;
            rope.solver.iterator = iterator;

            var distanceConstraint = new DistanceConstraint(ref rope.solver.particles);
            // var angleConstraint = new AngleConstraint(ref rope.solver.particles);
            // var bendConstraint = new BendConstraint(ref rope.solver.particles);
            solver.constraintBatch.Add(distanceConstraint);
            // solver.constraintBatch.Add(angleConstraint);
            // solver.constraintBatch.Add(bendConstraint);
        }
    }
}