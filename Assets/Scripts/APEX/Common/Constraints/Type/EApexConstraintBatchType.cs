namespace APEX.Common.Constraints
{
    public enum EApexConstraintBatchType
    {
        None = 0,     /* default constraint: None */
        DistanceConstraint,
        BendConstraint,
        BendConstraint2,
        AngleConstraint,
        VolumePreservationConstraint,
        CollisionConstraint,
        FixedPointConstraint,
        NormalConstraint,
        StretchingConstraint,
        TearConstraint,
    }
}