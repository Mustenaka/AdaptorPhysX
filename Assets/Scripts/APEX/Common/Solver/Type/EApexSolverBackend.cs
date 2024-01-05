namespace APEX.Common.Solver
{
    /// <summary>
    /// Declares which platform the calculation runtime
    /// </summary>
    public enum EApexSolverBackend
    {
        SingleThread, 
        JobsMultithreading,
    }
}