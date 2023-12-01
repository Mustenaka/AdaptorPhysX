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
        public Vector3 PreviousPosition { get; set; }
        public Vector3 NowPosition{ get; set; }
        public Vector3 NextPosition{ get; set; }

        /* rotation, serialize nowRotation, Rotation need adaptive game engine*/
        public Quaternion PreviousRotation{ get; set; }
        public Quaternion NowRotation{ get; set; }
        public Quaternion NextRoatation{ get; set; }
        
        /* scale equal transform.TransLocalScale */
        public Vector3 Scale{ get; set; }
        
        /* Index of the particle array */
        public int Index{ get; set; }
        
        /* True - this particle nextPosition will not apply nowPosition */
        public bool IsStatic{ get; set; }
        
        /* physic param */
        public float Mass{ get; set; }
    }
}