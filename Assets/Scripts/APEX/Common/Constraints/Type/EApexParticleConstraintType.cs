namespace APEX.Common.Constraints
{
    /// <summary>
    /// Particle constraint type, make sure is not confusion
    /// </summary>
    [System.Flags]
    public enum EApexParticleConstraintType
    {
        Free,       // freedom particle
        Pin,        // static particle, pin a position
        Collision,  // collider status
        Resist,     // Pin and resist the other object
        Drag,       // Drag the other object
    }
}