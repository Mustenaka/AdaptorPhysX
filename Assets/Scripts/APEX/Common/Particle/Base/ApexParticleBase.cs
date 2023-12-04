using System.Collections.Generic;
using UnityEngine;

namespace APEX.Common.Particle
{
    /// <summary>
    /// The base type of the particle type,
    /// if you need to extend the build, please inherit this module
    /// </summary>
    public class ApexParticleBase
    {
        /* position, serialize nowPosition */
        public Vector3 previousPosition;
        public Vector3 nowPosition;
        public Vector3 nextPosition;

        /* rotation, serialize nowRotation, Rotation need adaptive game engine*/
        public Quaternion previousRotation;
        public Quaternion nowRotation;
        public Quaternion nextRotation;
        
        /* scale equal transform.TransLocalScale */
        public Vector3 scale;
        
        /* Index of the particle array */
        public int index;
        
        /* True - this particle nextPosition will not apply nowPosition */
        public bool isStatic;
        
        /* physic param */
        public float mass;
    }
}