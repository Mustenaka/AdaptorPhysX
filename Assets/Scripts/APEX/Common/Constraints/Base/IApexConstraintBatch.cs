using System.Collections.Generic;
using APEX.Common.Constraints.Base;

namespace APEX.Common.Constraints
{
    public interface IApexConstraintBatch
    {
        void AddConstraint(int pl, int pr);
        void RemoveConstraint(int l, int r);
        void ClearConstraint(int l);
        void ClearAllConstraint();
        List<int> GetTargetParticle(int index);
        IApexConstraintBatch GetConstraintsByType(EApexConstraintBatchType type);
        void Do();
    }
}