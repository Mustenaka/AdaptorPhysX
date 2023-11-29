namespace APEX.Common.Constraints.Base
{
    public enum EApexConstraintBatchType
    {
        None = 0,     /* default constraint: None */
        DistanceConstraint,
        BendConstraint,
        AngleConstraint,
        VolumePreservationConstraint,
        CollisionConstraint,
        FixedPointConstraint,
        NormalConstraint,
        StretchingConstraint,
        TearConstraint,
    }
}