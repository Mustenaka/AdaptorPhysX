namespace APEX.Common.Simulator
{
    public abstract class ApexSimulatorBase
    {
        /// <summary>
        /// every job step
        /// </summary>
        /// <param name="dt"></param>
        public abstract void Step(float dt);
        
        /// <summary>
        /// all the step finished
        /// </summary>
        public abstract void Complete();
    }
}