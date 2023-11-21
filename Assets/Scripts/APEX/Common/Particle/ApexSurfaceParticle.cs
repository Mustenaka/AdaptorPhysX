using System.Collections.Generic;

namespace APEX.Common.Particle
{
    /// <summary>
    /// Particle implementation
    /// </summary>
    public class ApexSurfaceParticle : ApexParticleBase
    {
        // particle self param
        public float anisotropy;
        
        // matching param
        public List<int> pairVertex;
        public List<float> pairVertexValue;
    }
}