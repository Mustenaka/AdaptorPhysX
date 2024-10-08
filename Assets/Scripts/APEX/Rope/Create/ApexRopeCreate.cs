using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Common.Simulator;
using APEX.Common.Solver;
using APEX.Usage;
using Unity.Collections;
using Unity.Mathematics;
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

        public int particleCount = 20;
        public float stepSize = 1.2f;
        public Vector3 stepDirect = Vector3.left;

        public GameObject obj; // TO-DO: Use Material replace it.

        // default physics param(only use for create)
        public float mass = 1.0f; // If you want the centroid offset, please change this generation method
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.005f;
        [Range(1, 32)] public int iterator = 10;

        // Pin GameObject
        public List<ApexPin> pins;

        // Solver
        public ApexSolver solver;

        private void Start()
        {
            // Get Solver
            solver = FindObjectOfType<ApexSolver>();
            if (solver == null)
            {
                Debug.LogError("Solver not found!");
                return;
            }

            // if the obj is empty, create sphere to fill it
            // TODO: use render function to 
            if (obj == null)
            {
                obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);

                // this object better have no collider
                Destroy(obj.GetComponent<Collider>());
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
            // rope
            var rope = this.AddComponent<ApexRope>();

            rope.solver = solver;
            rope.particles = new List<ApexLineParticle>();
            rope.elements = new List<GameObject>();

            // rope simulator actor
            var ropeSimulatorActor = new ApexRopeSimulator
            {
                originPosition = new NativeArray<float3>(particleCount, Allocator.Persistent),
                previousPosition = new NativeArray<float3>(particleCount, Allocator.Persistent),
                nowPosition = new NativeArray<float3>(particleCount, Allocator.Persistent),
                nextPosition = new NativeArray<float3>(particleCount, Allocator.Persistent),

                mass = new NativeArray<float>(particleCount, Allocator.Persistent),
                forceExt = new NativeArray<float3>(particleCount, Allocator.Persistent),
                forceFrameExt = new NativeArray<float3>(particleCount, Allocator.Persistent),
                constraintTypes = new NativeArray<EApexParticleConstraintType>(particleCount, Allocator.Persistent),

                // distance constraint
                doubleConnect = new NativeArray<ApexConstraintParticleDouble>(particleCount - 1, Allocator.Persistent),
                distancelambdas = new NativeArray<float>(particleCount - 1, Allocator.Persistent),

                // bend constraint
                bendConnect = new NativeArray<ApexConstraintParticleThree>(particleCount - 1, Allocator.Persistent),
                bendLambdas = new NativeArray<float>(particleCount - 1, Allocator.Persistent),

                pin = new NativeArray<ApexPinConstraint>(particleCount, Allocator.Persistent),

                stiffness = stiffness,
                damping = damping,
                iterator = iterator,
            };

            // calc detail particle
            for (var i = 0; i < particleCount; i++)
            {
                var particlePosition = firstParticlePosition + i * (stepSize * stepDirect);

                var element = GameObject.Instantiate(obj, transform, true);
                element.name = i.ToString();
                element.transform.localPosition = particlePosition;

                var p = new ApexLineParticle
                {
                    index = i,
                    mass = mass,
                    previousPosition = particlePosition,
                    nowPosition = particlePosition,
                    forceExt = Vector3.zero,

                    scale = transform.localScale
                };

                ropeSimulatorActor.originPosition[i] = particlePosition;
                ropeSimulatorActor.previousPosition[i] = particlePosition;
                ropeSimulatorActor.nowPosition[i] = particlePosition;
                ropeSimulatorActor.nextPosition[i] = particlePosition;

                ropeSimulatorActor.mass[i] = mass;
                ropeSimulatorActor.constraintTypes[i] = EApexParticleConstraintType.Free;

                rope.elements.Add(element);
                rope.particles.Add(p);
            }

            // generate rope connect relation
            for (var i = 0; i < particleCount - 1; i++)
            {
                ropeSimulatorActor.doubleConnect[i] = new ApexConstraintParticleDouble(i, i + 1);
            }

            // generate rope bend relation(three relation)
            for (var i = 1; i < particleCount - 1; i++)
            {
                ropeSimulatorActor.bendConnect[i] = new ApexConstraintParticleThree(i - 1, i, i + 1);
            }

            // mark first particle is pin
            for (var i = 0; i < pins.Count; i++)
            {
                ropeSimulatorActor.pin[pins[i].particleIndex] = new ApexPinConstraint(pins[i].pinPosition);
                // ropeSimulatorActor.constraintTypes[pins[i].particleIndex] = EApexParticleConstraintType.Pin;
                ropeSimulatorActor.constraintTypes[pins[i].particleIndex] = pins[i].type;
            }

            // send it to solver, This have 1 question:
            // solver.particles is all particles
            // rope.particles is not all particles, is only own particles
            // so, this block will create a error
            // need fix it. than i can create multiple objects
            rope.solver.particles.AddRange(new List<ApexParticleBase>(rope.particles));
            rope.solver.actors.Add(ropeSimulatorActor);

            // add rope compare SimulatorActor
            rope.ropeSimulator = ropeSimulatorActor;
            rope.pins = pins;
        }
    }
}