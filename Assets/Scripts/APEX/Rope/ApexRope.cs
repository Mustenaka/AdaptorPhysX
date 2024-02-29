using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Render;
using APEX.Common.Simulator;
using APEX.Common.Solver;
using UnityEngine;

namespace APEX.Rope
{
    /// <summary>
    /// Rope actor
    /// </summary>
    public class ApexRope : MonoBehaviour, IApexRender
    {
        public List<GameObject>
            elements = new List<GameObject>(); // TODO: Use Material or something else to replace it.

        public List<ApexLineParticle> particles = new List<ApexLineParticle>();

        private int ParticlesCount => particles.Count;

        public ApexSolver solver;
        public ApexRopeSimulator ropeActor;

        private void Start()
        {
            // TODO: actor step before apply still Bug at first run app
            solver.actorStepBefore += SendParticle;
            solver.actorStepFinished += RendParticle;
        }

        private void OnDestroy()
        {
            // TODO: actor step before apply still Bug at first run app
            solver.actorStepBefore -= SendParticle;
            solver.actorStepFinished -= RendParticle;
        }

        // private void Update()
        // {
        //     for (var i = 0; i < ParticlesCount; i++)
        //     {
        //         elements[i].transform.localPosition = particles[i].nowPosition;
        //     }
        // }
        //
        // private void LateUpdate()
        // {
        //     for (var i = 0; i < ParticlesCount; i++)
        //     {
        //         solver.particles[i].nowPosition = elements[i].transform.localPosition;
        //     }
        // }

        private void RendParticle()
        {
            for (var i = 0; i < ParticlesCount; i++)
            {
                elements[i].transform.localPosition = particles[i].nowPosition;
            }
        }

        private void SendParticle()
        {
            for (var i = 0; i < ParticlesCount; i++)
            {
                solver.particles[i].nowPosition = elements[i].transform.localPosition;
            }
        }

        public ApexRenderType GetRenderType()
        {
            return ApexRenderType.Rope;
        }
    }
}