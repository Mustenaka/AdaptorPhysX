namespace APEX.Common.Constraints
{
    /// <summary>
    /// Constraint Connect
    ///     Double particle link
    /// </summary>
    public struct ApexConstraintParticleDouble
    {
        // particle left, particle right (both construct constraint)
        public int pl, pr;
        
        public ApexConstraintParticleDouble(int pl, int pr)
        {
            this.pl = pl;
            this.pr = pr;
        }
    }
}