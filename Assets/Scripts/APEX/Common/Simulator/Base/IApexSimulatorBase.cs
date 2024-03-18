using System.Collections.Generic;
using APEX.Common.Particle;
using APEX.Usage;

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
        /// Sync pin constraint
        /// </summary>
        /// <param name="pins">Mono ApexPin</param>
        public void SyncPinConstraint(List<ApexPin> pins);
        
        /// <summary>
        /// Do action before Step()
        /// </summary>
        /// <param name="cnt">a count of div</param>
        public void DoBeforeStepAction(int cnt);
        
        /// <summary>
        /// Do action after then Complete()
        /// </summary>
        /// <param name="cnt">a count of div</param>
        public void DoAfterCompleteAction(int cnt);
        
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