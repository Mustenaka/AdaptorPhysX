using UnityEngine;

namespace APEX.Common.Particle
{
    /// <summary>
    /// The base type of the particle type,
    /// if you need to extend the build, please inherit this module
    /// </summary>
    public class ApexParticleBase
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 scale;
        
        public int index;
    }
}