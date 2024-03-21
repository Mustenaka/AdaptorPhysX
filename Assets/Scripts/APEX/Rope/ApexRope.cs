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
    /// Rope
    /// </summary>
    public class ApexRope : MonoBehaviour, IApexRender
    {
        // TODO: Use Material or something else to replace it.
        public List<GameObject> elements = new List<GameObject>();
        public List<ApexLineParticle> particles = new List<ApexLineParticle>();

        public List<ApexPin> pins;

        public ApexSolver solver;
        public ApexRopeSimulator ropeSimulator;

        private void Start()
        {
            ropeSimulator.beforeStep += SendParticle;
            ropeSimulator.afterComplete += RendParticle;
        }

        private void OnDestroy()
        {
            ropeSimulator.beforeStep -= SendParticle;
            ropeSimulator.afterComplete -= RendParticle;
        }

        private void Update()
        {
            // sync pins
            ropeSimulator.SyncPinConstraint(pins);
        }

        private void RendParticle(int div)
        {
            for (var i = 0; i < elements.Count; i++)
            {
                elements[i].transform.localPosition = solver.particles[i + div].nowPosition;
            }
        }

        private void SendParticle(int div)
        {
            for (var i = 0; i < elements.Count; i++)
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