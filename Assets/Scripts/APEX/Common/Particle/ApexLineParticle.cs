using System.Collections.Generic;
using UnityEngine;

namespace APEX.Common.Particle
{
    /// <summary>
    /// Particle implementation:
    ///     Use this for Rope|Line particle implementation.
    /// </summary>
    [System.Serializable]
    public class ApexLineParticle : ApexParticleBase
    {
        public Vector3 szie;
        
        // use it for vertext generate
        public int vertexCount;
        public List<Vector3> vertex;
    }
}