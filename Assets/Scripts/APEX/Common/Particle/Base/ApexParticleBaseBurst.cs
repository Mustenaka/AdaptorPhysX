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
        /// Convert Burst Type to BaseUnityType
        /// </summary>
        /// <returns>ApexParticleBase</returns>
        public ApexParticleBase Convert()
        {
            return new ApexLineParticle()
            {
                previousPosition = this.previousPosition.ToVector3(),
                nowPosition = this.nowPosition.ToVector3(),
                nextPosition = this.nextPosition.ToVector3(),
                
                scale = this.scale.ToVector3(),
                
                index = this.index,
                
                forceExt = this.forceExt.ToVector3(),
                forceApply = this.forceApply.ToVector3(),
                
                mass = this.mass,
            };
        }
    }
}