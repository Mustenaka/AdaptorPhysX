using Unity.Mathematics;
using APEX.Tools.MathematicsTools;

namespace APEX.Common.Particle
{
    /// <summary>
    /// particle base, struct use for Burst
    /// </summary>
    public struct ApexParticleBaseBurst
    {
        /* position, serialize nowPosition */
        public float3 previousPosition;
        public float3 nowPosition;
        public float3 nextPosition;
        
        /* scale equal transform.TransLocalScale */
        public float3 scale;
        
        /* Index of the particle array */
        public int index;
        
        /* True - this particle nextPosition will not apply nowPosition */
        public bool isStatic;
        
        /* physic param */
        public float3 forceExt;        // calc physic
        public float3 forceApply;      // apply physic
        public float mass;

        public ApexParticleBaseBurst(ApexParticleBase ori)
        {
            previousPosition = ori.previousPosition.ToFloat3();
            nowPosition = ori.nowPosition.ToFloat3();
            nextPosition = ori.nextPosition.ToFloat3();

            scale = ori.scale.ToFloat3();

            index = ori.index;

            isStatic = ori.isStatic;

            forceExt = ori.forceExt.ToFloat3();
            forceApply = ori.forceApply.ToFloat3();
            mass = ori.mass;
        }

        /// <summary>
        /// Convert Burst Type to ApexParticleBase
        /// </summary>
        /// <returns>ApexParticleBase</returns>
        public void ConvertBaseClass(ApexParticleBase particle)
        {
            particle.previousPosition = this.previousPosition.ToVector3();
            particle.nowPosition = this.nowPosition.ToVector3();
            particle.nextPosition = this.nextPosition.ToVector3();
            
            particle.scale = this.scale.ToVector3();
            particle.index = this.index;
            
            particle.forceExt = this.forceExt.ToVector3();
            particle.forceApply = this.forceApply.ToVector3();
            particle.mass = this.mass;
        }
    }
}