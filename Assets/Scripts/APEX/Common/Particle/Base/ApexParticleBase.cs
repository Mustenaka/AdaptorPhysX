using UnityEngine;

namespace APEX.Common.Particle
{
    /// <summary>
    /// The base type of the particle type,
    /// if you need to extend the build, please inherit this module
    /// </summary>
    public class ApexParticleBase
    {
        public Vector3 previousPosition;
        public Vector3 nowPosition;
        public Vector3 nextPosition;

        public Quaternion previousRotation;
        public Quaternion nowRotation;
        public Quaternion nextRoatation;
        
        public Vector3 scale;
        
        public int index;
        public bool isStatic;
        
        // physic param
        public float mass;
    }
}