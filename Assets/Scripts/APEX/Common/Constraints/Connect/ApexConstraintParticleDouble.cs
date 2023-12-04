namespace APEX.Common.Constraints
{
    /// <summary>
    /// Constraint Connect
    ///     Double particle link
    /// </summary>
    public class ApexConstraintParticleDouble
    {
        // particle left, particle right (both construct constraint)
        public int pl, pr;

        public ApexConstraintParticleDouble()
        {
            
        }

        public ApexConstraintParticleDouble(int pl, int pr)
        {
            this.pl = pl;
            this.pr = pr;
        }
    }
}