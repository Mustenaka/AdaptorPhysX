using System;
using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints.Base;

namespace APEX.Common.Constraints
{
    public abstract class ApexConstraintBatchDouble : IApexConstraintBatch
    {
        public EApexConstraintBatchType constraintBatchType;                               // This ConstraintType 
        
        protected Dictionary<int, List<ApexConstraintParticleDouble>> constraints;         // use hash table for quick search

        /// <summary>
        /// Create ApexConstraintBatchDouble
        /// </summary>
        protected ApexConstraintBatchDouble()
        {
            constraints = new Dictionary<int, List<ApexConstraintParticleDouble>>();
        }

        /// <summary>
        /// Create ApexConstraintBatchDouble and appoint batchType
        /// </summary>
        /// <param name="batchType"></param>
        protected ApexConstraintBatchDouble(EApexConstraintBatchType batchType)
        {
            constraintBatchType = batchType;
            constraints = new Dictionary<int, List<ApexConstraintParticleDouble>>();
        }
        
        /// <summary>
        /// Add Constraint
        /// </summary>
        /// <param name="particles">the constraint particle(must have 2)</param>
        /// <exception cref="SystemException">if the particles length is not 2, exception</exception>
        public void AddConstraint(params int[] particles)
        {
            int len = particles.Length;
            if (len != 2)
            {
                throw new SystemException("AddConstraint must get 2 particle");
            }
            
            var t = new ApexConstraintParticleDouble
            {
                pl = particles[0],
                pr = particles[1]
            };
            
            // if constraint.key is alive, add the constraint
            if (constraints.TryGetValue(particles[0], out var particleConstraints))
            {
                particleConstraints.Add(t);
            }
            else
            {
                // the constraint.key is empty than create the key and value
                constraints.Add(particles[0], new List<ApexConstraintParticleDouble>()
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
            if (len != 2)
            {
                throw new SystemException("AddConstraint must get 2 particle");
            }
            
            if (!constraints.TryGetValue(particles[0], out var particleConstraints))
            { 
                throw new SystemException("particle" + particles[0] + " not have constraint to " + particles[1]);
            }
            
            particleConstraints.RemoveAll(c => c.pl == particles[0] && c.pr == particles[1]);
        }

        /// <summary>
        /// Clear constraint for one index, it use for one particle destroy
        /// </summary>
        /// <param name="l">constraint index</param>
        /// <exception cref="SystemException"></exception>
        public void ClearConstraint(int l)
        {
            if (!constraints.TryGetValue(l, out var particleConstraints))
            { 
                throw new SystemException("particle" + l + " not have any constraint");
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