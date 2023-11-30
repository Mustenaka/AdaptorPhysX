namespace APEX.Common.Constraints
{
    /// <summary>
    /// Particle linkage
    /// </summary>
    public class ApexConstraintPair
    {
        // particle left, particle right (both construct constraint)
        public int pl, pr;

        public ApexConstraintPair()
        {
            
        }

        public ApexConstraintPair(int l, int r)
        {
            pl = l;
            pr = r;
        }
    }
}