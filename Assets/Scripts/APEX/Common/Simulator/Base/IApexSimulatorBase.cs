namespace APEX.Common.Simulator
{
    public interface IApexSimulatorBase
    {
        /// <summary>
        /// every job step
        /// </summary>
        /// <param name="dt"></param>
        public void Step(float dt);

        /// <summary>
        /// all the step finished
        /// </summary>
        public void Complete();
    }
}