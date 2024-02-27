using UnityEngine;

namespace APEX.Common.Particle
{
    /// <summary>
    /// The base type of the particle type,
    /// if you need to extend the build, please inherit this module
    /// </summary>
    [System.Serializable]
    public class ApexParticleBase
    {
        /* position, serialize nowPosition */
        public Vector3 originPosition;
        public Vector3 previousPosition;
        public Vector3 nowPosition;
        public Vector3 nextPosition;

        /* scale equal transform.TransLocalScale */
        public Vector3 scale;

        /* Index of the particle array */
        public int index;

        /* physic param */
        public Vector3 forceExt;
        public float mass;
    }
}