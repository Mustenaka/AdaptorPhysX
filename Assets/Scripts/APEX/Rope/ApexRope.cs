using System;
using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Common.Render;
using APEX.Common.Simulator;
using APEX.Common.Solver;
using APEX.Usage;
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

        public List<ApexPin> pins;

        public ApexSolver solver;
        public ApexRopeSimulator ropeSimulator;

        private void Start()
        {
            solver.actorStepBefore += SendParticle;
            solver.actorStepFinished += RendParticle;
        }

        private void Update()
        {
            for (var i = 0; i < pins.Count; i++)
            {
                ropeSimulator.SyncPinFromSolve(i, pins[i].particleIndex, pins[i].pinPosition);
            }
        }

        private void OnDestroy()
        {
            solver.actorStepBefore -= SendParticle;
            solver.actorStepFinished -= RendParticle;
        }

        private void RendParticle(int div)
        {
            for (var i = 0; i < ParticlesCount; i++)
            {
                elements[i].transform.localPosition = particles[i + div].nowPosition;
            }
        }

        private void SendParticle(int div)
        {
            for (var i = 0; i < ParticlesCount; i++)
            {
                solver.particles[i + div].nowPosition = elements[i].transform.localPosition;
            }
        }

        public ApexRenderType GetRenderType()
        {
            return ApexRenderType.Rope;
        }
    }
}