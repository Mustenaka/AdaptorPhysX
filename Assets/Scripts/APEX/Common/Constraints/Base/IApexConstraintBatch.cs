using System.Collections.Generic;

namespace APEX.Common.Constraints
{
    public interface IApexConstraintBatch
    {
        void AddConstraint(params int[] particles);
        void RemoveConstraint(params int[] particles);
        void ClearConstraint(int index);
        void ClearAllConstraint();
        List<int> GetTargetParticle(int index);
        IApexConstraintBatch GetConstraintsByType(EApexConstraintBatchType type);
        void Do();
    }
}