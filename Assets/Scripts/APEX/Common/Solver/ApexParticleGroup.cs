using System.Collections.Generic;
using APEX.Common.Particle;

namespace APEX.Common.Solver
{
    /// <summary>
    /// The particle Group Usage
    /// </summary>
    public class ApexParticleGroup<T> where T : ApexParticleBase
    {
        public List<T> particles = new List<T>();
    }
}