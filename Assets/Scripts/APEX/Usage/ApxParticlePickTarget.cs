using UnityEngine;

namespace APEX.Usage
{
    /// <summary>
    /// Maps a picked Unity collider to one stable native particle ID.
    /// </summary>
    public sealed class ApxParticlePickTarget : MonoBehaviour
    {
        public uint particleId;
    }
}
