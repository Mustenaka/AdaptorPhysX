namespace APEX.Common.Constraints
{
    /// <summary>
    /// Constraint Connect
    ///     Triple particle link
    /// </summary>
    public class ApexConstraintParticleThree
    {
        // particle left, mid, right
        public int pl, pmid, pr;

        public ApexConstraintParticleThree()
        {
            
        }

        public ApexConstraintParticleThree(int pl, int pmid, int pr)
        {
            this.pl = pl;
            this.pmid = pmid;
            this.pr = pr;
        }
    }
}