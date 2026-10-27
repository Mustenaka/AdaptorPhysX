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
        /// <summary>
        /// Torque reference point at the end of the step: the collider shape's reference point (centre of a
        /// sphere, box or distance field, first end of a capsule), or the origin of a kinematic frame (grab
        /// target, pin collider position).
        /// </summary>
        public float3 Origin;
        /// <summary>Contact point weighted by the size of the node corrections (colliders) or constraint forces (frames).</summary>
        public float3 ContactCentroid;
        /// <summary>Nodes in contact, averaged over the substeps; for a kinematic frame, the nodes it holds.</summary>
        public float ContactNodes;
        /// <summary>
        /// Colliders: sum of n n^T over the contacts (n the collider's surface normal), weighted like the
        /// centroid and scaled to unit trace. Zero for kinematic frames and without contact.
        /// </summary>
        public float3x3 ContactNormals;

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
                ContactNormals = total > 0.0f
                    ? (a.ContactNormals * math.length(a.Force) + b.ContactNormals * math.length(b.Force)) / total
                    : a.ContactNormals,
            };
        }
    }

    /// <summary>
    /// Local linear model of the force on an instrument, published once per physics step (HAP-4), for a
    /// haptic thread to evaluate between steps: F(x) = Force0 + Stiffness (x - Position0).
    /// </summary>
    public struct ApxForceModel
    {
        /// <summary>Simulated time at the end of the step (s).</summary>
        public double Time;
        /// <summary>Instrument reference point averaged over the step's substeps: where Force0 applies.</summary>
        public float3 Position0;
        /// <summary>Force of the tissue on the instrument in the step (N), averaged over the substeps.</summary>
        public float3 Force0;
        /// <summary>dF/dx (N/m): symmetric and negative semi-definite, so the model does no work on a closed path.</summary>
        public float3x3 Stiffness;
        /// <summary>Largest force <see cref="Evaluate"/> returns (N); 0 for no limit.</summary>
        public float MaxForce;

        /// <summary>
        /// Force at <paramref name="position"/>. Zero once it would turn against Force0: the contact only pushes,
        /// so the instrument has left the tissue there.
        /// </summary>
        public float3 Evaluate(float3 position)
        {
            float3 force = Force0 + math.mul(Stiffness, position - Position0);
            if (!(math.dot(force, Force0) > 0.0f))
            {
                return float3.zero;
            }
            float length = math.length(force);
            return MaxForce > 0.0f && length > MaxForce ? force * (MaxForce / length) : force;
        }

        /// <summary>Models of the same instrument from several bodies, about a's position.</summary>
        public static ApxForceModel operator +(ApxForceModel a, ApxForceModel b) => new ApxForceModel
        {
            Time = math.max(a.Time, b.Time),
            Position0 = a.Position0,
            Force0 = a.Force0 + b.Force0 + math.mul(b.Stiffness, a.Position0 - b.Position0),
            Stiffness = a.Stiffness + b.Stiffness,
            MaxForce = a.MaxForce > 0.0f && b.MaxForce > 0.0f ? math.min(a.MaxForce, b.MaxForce) : math.max(a.MaxForce, b.MaxForce),
        };
    }

    /// <summary>
    /// Builds the force model of one instrument from its successive steps (HAP-4). The stiffness is -k N:
    /// N the contact normals of the step (<see cref="ApxWrench.ContactNormals"/>), k the change of the force
    /// along the motion since the last update, -s.y / (s.N s) for motion s and force change y. k is kept while
    /// the instrument moves less than the given distance (slower motion accumulates until it does) or mostly
    /// along the surface, and is 0 out of contact.
    /// </summary>
    public struct ApxForceModelEstimator
    {
        private bool _started;
        private float3 _anchorPosition;
        private float3 _anchorForce;
        private float _stiffness;

        /// <summary>Contact stiffness k (N/m) of the last model.</summary>
        public float NormalStiffness => _stiffness;

        /// <param name="position">Instrument reference point averaged over the step's substeps.</param>
        /// <param name="force">Force of the step; zero without contact.</param>
        /// <param name="normals">Contact normals of the step, unit trace.</param>
        /// <param name="minMotion">Motion (m) before k is updated.</param>
        public ApxForceModel Update(double time, float3 position, float3 force, float3x3 normals, float minMotion)
        {
            if (!(math.lengthsq(force) > 0.0f))
            {
                // A contact starts from zero force at the last position out of contact.
                _stiffness = 0.0f;
                _anchorForce = float3.zero;
                _anchorPosition = position;
                _started = true;
            }
            else if (!_started)
            {
                _anchorForce = force;
                _anchorPosition = position;
                _started = true;
            }
            else
            {
                float3 s = position - _anchorPosition;
                float ss = math.dot(s, s);
                if (ss >= minMotion * minMotion)
                {
                    float sns = math.dot(s, math.mul(normals, s));
                    if (sns > 0.1f * ss)
                    {
                        _stiffness = math.max(0.0f, -math.dot(s, force - _anchorForce) / sns);
                    }
                    _anchorForce = force;
                    _anchorPosition = position;
                }
            }
            return new ApxForceModel
            {
                Time = time,
                Position0 = position,
                Force0 = force,
                Stiffness = -_stiffness * normals,
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

        /// <summary>Sum of the force models all soft bodies published for <paramref name="collider"/> in their last step (HAP-4).</summary>
        public static bool TrySumColliderForceModel(Collider collider, out ApxForceModel sum)
        {
            sum = default;
            bool any = false;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                if (!body.TryGetColliderForceModel(collider, out ApxForceModel model))
                {
                    continue;
                }
                sum = any ? sum + model : model;
                any = true;
            }
            return any;
        }
    }
}
