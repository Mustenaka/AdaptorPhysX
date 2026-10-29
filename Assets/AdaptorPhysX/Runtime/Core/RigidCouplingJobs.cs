using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// A dynamic rigid body touched by the tissue (two-way coupling, TODO ECO-3). The step moves it
    /// along the path the physics engine gives it without the tissue (start pose and velocity, plus
    /// gravity over the step) plus a correction driven by the momentum the tissue nodes receive
    /// from its colliders, substep by substep. The correction at the end of the step is added to
    /// the engine's result.
    /// </summary>
    public struct RigidBodyState
    {
        // Input at the start of the step.
        public float InverseMass;
        /// <summary>World-space inverse inertia at the start of the step.</summary>
        public float3x3 InverseInertia;
        public float3 Center;
        public float3 Velocity;
        public float3 AngularVelocity;
        public float3 Gravity;

        // Correction by the tissue, relative to the free path.
        public float3 Displacement;
        public float3 DeltaVelocity;
        /// <summary>Rotation vector of the corrective rotation about the centre of mass.</summary>
        public float3 Rotation;
        public float3 DeltaAngularVelocity;

        public static RigidBodyState Create(float mass, float3 inertia, quaternion inertiaToWorld, float3 center,
            float3 velocity, float3 angularVelocity, float3 gravity)
        {
            float3x3 r = new float3x3(inertiaToWorld);
            float3 inverse = math.select(1.0f / math.max(inertia, 1e-12f), 0.0f, inertia <= 0.0f);
            return new RigidBodyState
            {
                InverseMass = mass > 0.0f ? 1.0f / mass : 0.0f,
                InverseInertia = math.mul(r, math.mul(float3x3.Scale(inverse), math.transpose(r))),
                Center = center,
                Velocity = velocity,
                AngularVelocity = angularVelocity,
                Gravity = gravity,
            };
        }

        /// <summary>Corrective rotation as a quaternion.</summary>
        public quaternion Turn => Exp(Rotation);

        public static quaternion Exp(float3 rotation)
        {
            float angle = math.length(rotation);
            return angle > 1e-9f ? quaternion.AxisAngle(rotation / angle, angle) : quaternion.identity;
        }
    }

    /// <summary>
    /// After each node job: takes the momentum the nodes received from the colliders of every dynamic
    /// rigid body since the previous call, changes the body's corrective velocity by the opposite
    /// impulse, and moves its colliders to their pose at the end of the next substep.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RigidCouplingJob : IJob
    {
        /// <summary>Blocks * shapes impulse sums of the node jobs (since the start of the step).</summary>
        [ReadOnly] public NativeArray<ColliderImpulse> Partial;
        /// <summary>Shapes as gathered: the pose at the start of the step.</summary>
        [ReadOnly] public NativeArray<ColliderShape> ShapeStart;
        public NativeArray<ColliderShape> Shapes;
        /// <summary>Per shape: impulse sums already applied to the rigid bodies.</summary>
        public NativeArray<ColliderImpulse> Consumed;
        public NativeArray<RigidBodyState> Bodies;
        public float SubstepDt;
        public float StepDt;
        /// <summary>Substeps finished so far; the colliders are moved to the end of the next one when Advance is set.</summary>
        public int Substep;
        public bool Advance;

        public void Execute()
        {
            int shapes = Shapes.Length;
            for (int s = 0; s < shapes; s++)
            {
                ColliderShape shape = Shapes[s];
                if (shape.DynamicBody <= 0)
                {
                    continue;
                }
                var total = new ColliderImpulse();
                for (int b = s; b < Partial.Length; b += shapes)
                {
                    total += Partial[b];
                }
                ColliderImpulse consumed = Consumed[s];
                float3 linear = total.Linear - consumed.Linear;
                float3 angular = total.Angular - consumed.Angular;
                Consumed[s] = total;

                // The nodes gained this momentum (angular about the shape's A1 during the substep):
                // the body loses it, about its current centre of mass.
                RigidBodyState body = Bodies[shape.DynamicBody - 1];
                float3 center = CenterAt(body, Substep * SubstepDt);
                float3 aboutCenter = angular + math.cross(shape.A1 - center, linear);
                body.DeltaVelocity -= linear * body.InverseMass;
                body.DeltaAngularVelocity -= math.mul(body.InverseInertia, aboutCenter);
                Bodies[shape.DynamicBody - 1] = body;
            }
            if (!Advance)
            {
                return;
            }
            for (int i = 0; i < Bodies.Length; i++)
            {
                RigidBodyState body = Bodies[i];
                body.Displacement += body.DeltaVelocity * SubstepDt;
                body.Rotation += body.DeltaAngularVelocity * SubstepDt;
                Bodies[i] = body;
            }
            float t = (Substep + 1) * SubstepDt;
            for (int s = 0; s < shapes; s++)
            {
                ColliderShape shape = Shapes[s];
                if (shape.DynamicBody <= 0)
                {
                    continue;
                }
                RigidBodyState body = Bodies[shape.DynamicBody - 1];
                ColliderShape start = ShapeStart[s];
                float3 center = CenterAt(body, t);
                quaternion turn = math.mul(body.Turn, RigidBodyState.Exp(body.AngularVelocity * t));
                shape.A0 = shape.A1;
                shape.B0 = shape.B1;
                shape.R0 = shape.R1;
                shape.A1 = center + math.mul(turn, start.A1 - body.Center);
                shape.B1 = center + math.mul(turn, start.B1 - body.Center);
                shape.R1 = math.normalize(math.mul(turn, start.R1));
                Shapes[s] = shape;
            }
        }

        /// <summary>
        /// Centre of mass at time t into the step: the engine's path without the tissue (velocity
        /// after the gravity of the whole step, as a semi-implicit Euler step moves it) plus the correction.
        /// </summary>
        private float3 CenterAt(RigidBodyState body, float t) =>
            body.Center + (body.Velocity + body.Gravity * StepDt) * t + body.Displacement;
    }
}
