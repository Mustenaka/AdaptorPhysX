using System.Collections.Generic;
using APEX.Common.Particle;
using UnityEngine;

namespace APEX.Common.Simulator
{
    public interface IApexSimulatorBase
    {
        /// <summary>
        /// every job step
        /// </summary>
        /// <param name="dt">PBD delta time(not unity deltaTime)</param>
        public void Step(float dt);

        /// <summary>
        /// all the step finished
        /// </summary>
        public void Complete();

        /// <summary>
        /// sync pin information solver to simulator
        /// </summary>
        /// <param name="pinIndex">pin index</param>
        /// <param name="pinPosition">pin position</param>
        public void SyncPinFromSolve(List<int> pinIndex, List<Vector3> pinPosition);
        
        /// <summary>
        /// sync particle from solver to simulator
        /// </summary>
        /// <param name="particles">all solver particle</param>
        /// <param name="down">particle lower bound</param>
        public void SyncParticleFromSolve(List<ApexParticleBase> particles, int down);
        
        /// <summary>
        /// sync particle from simulator to solver
        /// </summary>
        /// <param name="particles">all solver particle</param>
        /// <param name="down">particle lower bound</param>
        public void SyncParticleToSolver(List<ApexParticleBase> particles, int down);

        /// <summary>
        /// Gets how many particles the simulator actor has
        /// </summary>
        /// <returns></returns>
        public int GetParticleCount();

        /// <summary>
        /// Dispose resource, when solver is OnDestroy()
        /// </summary>
        public void Dispose();
    }
}