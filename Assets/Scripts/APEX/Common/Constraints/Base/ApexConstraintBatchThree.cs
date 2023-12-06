using System;
using System.Collections.Generic;
using System.Linq;

namespace APEX.Common.Constraints
{
    [Serializable]
    public abstract class ApexConstraintBatchThree : IApexConstraintBatch
    {
        public EApexConstraintBatchType constraintBatchType;                               // This ConstraintType 
        
        protected Dictionary<int, List<ApexConstraintParticleThree>> constraints;         // use hash table for quick search

        /// <summary>
        /// Create ApexConstraintBatchDouble
        /// </summary>
        protected ApexConstraintBatchThree()
        {
            constraints = new Dictionary<int, List<ApexConstraintParticleThree>>();
        }

        /// <summary>
        /// Create ApexConstraintBatchDouble and appoint batchType
        /// </summary>
        /// <param name="batchType"></param>
        protected ApexConstraintBatchThree(EApexConstraintBatchType batchType)
        {
            constraintBatchType = batchType;
            constraints = new Dictionary<int, List<ApexConstraintParticleThree>>();
        }
        
        /// <summary>
        /// Add Constraint
        /// </summary>
        /// <param name="particles">the constraint particle(must have 3)</param>
        /// <exception cref="SystemException">if the particles length is not 3, exception</exception>
        public void AddConstraint(params int[] particles)
        {
            int len = particles.Length;
            if (len != 3)
            {
                throw new SystemException("AddConstraint must get 3 particle");
            }
            
            var t = new ApexConstraintParticleThree
            {
                pl = particles[0],
                pmid = particles[1],
                pr = particles[2],
            };
            
            // if constraint.key is alive, add the constraint
            if (constraints.TryGetValue(particles[1], out var particleConstraints))
            {
                particleConstraints.Add(t);
            }
            else
            {
                // the constraint.key is empty than create the key and value
                constraints.Add(particles[1], new List<ApexConstraintParticleThree>()
                {
                    t
                });
            }
        }

        /// <summary>
        /// Remove Constraint between l and r
        /// </summary>
        /// <param name="particles">the constraint particle(must have 2)</param>
        /// <exception cref="SystemException">if the particles length is not 2, exception</exception>
        public void RemoveConstraint(params int[] particles)
        {
            int len = particles.Length;
            if (len != 3)
            {
                throw new SystemException("AddConstraint must get 3 particle");
            }
            
            if (!constraints.TryGetValue(particles[0], out var particleConstraints))
            { 
                throw new SystemException("mid particle" + particles[1] + " not have constraint to " + particles[0] +
                                          " and " + particles[2]);
            }

            particleConstraints.RemoveAll(c => c.pl == particles[0] && c.pmid == particles[1] && c.pr == particles[2]);
        }

        /// <summary>
        /// Clear constraint for one index, it use for one particle destroy
        /// </summary>
        /// <param name="index">constraint index</param>
        /// <exception cref="SystemException"></exception>
        public void ClearConstraint(int index)
        {
            if (!constraints.TryGetValue(index, out var particleConstraints))
            { 
                throw new SystemException("particle" + index + " not have any constraint");
            }
            
            particleConstraints.Clear();
        }

        /// <summary>
        /// Remove all constraint
        /// </summary>
        public void ClearAllConstraint()
        {
            constraints.Clear();
        }

        /// <summary>
        /// Get the particle index collection which is linked one particle.
        /// </summary>
        /// <param name="index">the one particle index</param>
        /// <returns>another particle collection</returns>
        /// <exception cref="SystemException">not have any constraint</exception>
        public List<int> GetTargetParticle(int index)
        {
            if (!constraints.TryGetValue(index, out var particleConstraints))
            { 
                throw new SystemException("particle" + index + " not have any constraint to ");
            }

            return particleConstraints.Select(p => p.pr).ToList();
        }

        /// <summary>
        /// Get Constraint by type
        /// </summary>
        /// <param name="type">constraint type</param>
        /// <returns>constraint</returns>
        public virtual IApexConstraintBatch GetConstraintsByType(EApexConstraintBatchType type)
        {
            return this;
        }

        /// <summary>
        /// Do the constraint, must override it
        /// </summary>
        public virtual void Do()
        {
            // Do Constraint
        }
    }
}