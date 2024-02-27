using System;

namespace APEX.Common.Solver
{
    /// <summary>
    /// Declares which platform the calculation runtime
    /// </summary>
    [Obsolete]
    public enum EApexSolverBackend
    {
        SingleThread, 
        JobsMultithreading,
        GPU,
    }
}