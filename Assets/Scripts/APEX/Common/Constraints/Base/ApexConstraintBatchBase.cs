using System;
using System.Collections.Generic;
using System.Linq;
using APEX.Common.Constraints.Base;

namespace APEX.Common.Constraints
{
    public abstract class ApexConstraintBatchBase<T> : IApexConstraintBatch where T : ApexParticleConstraintBase, new()
    {
        public EApexConstraintBatchType constraintBatchType; // This ConstraintType 
        public Dictionary<int, List<T>> constraints;         // use hash table for quick search

        private ApexConstraintBatchBase()
        {
            constraints = new Dictionary<int, List<T>>();
        }

        private ApexConstraintBatchBase(EApexConstraintBatchType batchType)
        {
            constraintBatchType = batchType;
            constraints = new Dictionary<int, List<T>>();
        }

        /// <summary>
        /// Add Constraint
        /// </summary>
        /// <param name="pl"></param>
        /// <param name="pr"></param>
        public void AddConstraint(int pl, int pr)
        {
            var t = new T
            {
                pl = pl,
                pr = pr
            };

            // if constraint.key is alive, add the constraint
            if (constraints.TryGetValue(pl, out var particleConstraints))
            {
                particleConstraints.Add(t);
            }
            else
            {
                // the constraint.key is empty than create the key and value
                constraints.Add(pl, new List<T>()
                {
                    t
                });
            }
        }

        /// <summary>
        /// Remove Constraint between l and r
        /// </summary>
        /// <param name="l">constraint l index</param>
        /// <param name="r">constraint r index</param>
        /// <exception cref="SystemException">l not have constraint to r, remove fail</exception>
        public void RemoveConstraint(int l, int r)
        {
            if (!constraints.TryGetValue(l, out var particleConstraints))
            { 
                throw new SystemException("particle" + l + " not have constraint to " + r);
            }
                
            var p = new ApexParticleConstraintBase()
            {
                pl = l,
                pr = r
            };
            particleConstraints.Remove(p as T);
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
        /// Do the constraint
        /// </summary>
        public void Do()
        {
            // Do Constraint
        }
    }
}