using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>Force and torque exerted by tissue on an instrument over one physics step (world space).</summary>
    public struct ApxWrench
    {
        /// <summary>N.</summary>
        public float3 Force;
        /// <summary>N m, about <see cref="Origin"/>.</summary>
        public float3 Torque;
        /// <summary>Torque reference point: the collider's transform position at the end of the step.</summary>
        public float3 Origin;
        /// <summary>Contact point weighted by the size of the node corrections.</summary>
        public float3 ContactCentroid;
        /// <summary>Nodes in contact, averaged over the substeps.</summary>
        public float ContactNodes;

        /// <summary>Torque about another point.</summary>
        public float3 TorqueAbout(float3 point) => Torque + math.cross(Origin - point, Force);

        public static ApxWrench operator +(ApxWrench a, ApxWrench b)
        {
            // Both torques are moved to a's origin.
            float total = math.length(a.Force) + math.length(b.Force);
            return new ApxWrench
            {
                Force = a.Force + b.Force,
                Torque = a.Torque + b.TorqueAbout(a.Origin),
                Origin = a.Origin,
                ContactCentroid = total > 0.0f
                    ? (a.ContactCentroid * math.length(a.Force) + b.ContactCentroid * math.length(b.Force)) / total
                    : a.ContactCentroid,
                ContactNodes = a.ContactNodes + b.ContactNodes,
            };
        }
    }

    public static class ApxHaptics
    {
        /// <summary>Sum of the wrenches all soft bodies exerted on <paramref name="collider"/> in their last step.</summary>
        public static bool TrySumColliderWrench(Collider collider, out ApxWrench sum)
        {
            sum = default;
            bool any = false;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                if (!body.TryGetColliderWrench(collider, out ApxWrench wrench))
                {
                    continue;
                }
                sum = any ? sum + wrench : wrench;
                any = true;
            }
            return any;
        }
    }
}
