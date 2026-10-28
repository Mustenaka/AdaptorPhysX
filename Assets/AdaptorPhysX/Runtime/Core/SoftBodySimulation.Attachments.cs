using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>A dynamic attachment (<see cref="SoftBodySimulation.AddAttachment"/>).</summary>
    internal struct AttachmentState
    {
        /// <summary>Target frame (world) at the start and the end of the step.</summary>
        public float4x4 Start;
        public float4x4 End;
        /// <summary>Compliance of the constraint of each point, m/N; 0 is rigid.</summary>
        public float Compliance;
        /// <summary>Force on one point, N, above which the point lets go.</summary>
        public float BreakForce;
        /// <summary>0 once removed.</summary>
        public int Active;
    }

    /// <summary>A material point held by a dynamic attachment.</summary>
    internal struct AttachedPoint
    {
        public int Attachment;
        /// <summary>Material point handle (<see cref="SoftBodySimulation.AddMaterialPoint(float3)"/>).</summary>
        public int Point;
        /// <summary>Where the point was in the target frame when it was attached.</summary>
        public float3 Local;
        /// <summary>1 while held; 0 after it let go or its material point was removed.</summary>
        public int Held;
    }

    /// <summary>
    /// Dynamic attachments (TODO ECO-13): material points held to a moving target frame by a constraint with a compliance,
    /// as Obi's dynamic particle attachment holds particles to a collider. Each point is pulled towards where it was in
    /// the target frame when attached; the constraint is solved once per substep after the cells, its force is
    /// lambda / h^2, and a point whose force exceeds the attachment's break force lets go for good. Unlike the kinematic
    /// frames (pins and grabs), the held material keeps its mass and moves under the constraint, and the attachment can
    /// hold any material point, also one on a cut face or between nodes. Targets are kinematic: the tissue does not push
    /// back on them.
    /// </summary>
    public sealed partial class SoftBodySimulation
    {
        private NativeList<AttachmentState> _attachments;
        private NativeList<AttachedPoint> _attachedPoints;
        /// <summary>Per attachment: target frame at the end of the previous step, the start of the next one.</summary>
        private readonly List<float4x4> _attachmentPrevious = new List<float4x4>();
        /// <summary>Per attachment: momentum it gave the tissue in the last step, as for the kinematic frames (HAP-2).</summary>
        private NativeArray<ColliderImpulse> _attachmentImpulses;
        private bool _attachmentsActive;

        /// <summary>
        /// Adds a dynamic attachment with its target frame at <paramref name="target"/> (world); returns its handle.
        /// <paramref name="compliance"/> is in m/N (0 rigid), <paramref name="breakForce"/> in N per point.
        /// </summary>
        public int AddAttachment(float4x4 target, float compliance = 0.0f, float breakForce = float.PositiveInfinity)
        {
            CompleteStep();
            _attachments.Add(new AttachmentState
            {
                Start = target,
                End = target,
                Compliance = math.max(compliance, 0.0f),
                BreakForce = breakForce,
                Active = 1,
            });
            _attachmentPrevious.Add(target);
            return _attachments.Length - 1;
        }

        /// <summary>Holds material point <paramref name="point"/> at its current place in the attachment's target frame.</summary>
        public void AttachPoint(int attachment, int point)
        {
            CompleteStep();
            if (!IsAttachment(attachment) || !IsMaterialPoint(point))
            {
                throw new System.ArgumentException("No such attachment or material point.");
            }
            _attachedPoints.Add(new AttachedPoint
            {
                Attachment = attachment,
                Point = point,
                Local = math.transform(math.inverse(_attachments[attachment].End), MaterialPointPosition(point)),
                Held = 1,
            });
        }

        /// <summary>Moves the target frame; the points follow during the next step.</summary>
        public void MoveAttachment(int attachment, float4x4 target)
        {
            CompleteStep();
            AttachmentState state = _attachments[attachment];
            state.End = target;
            _attachments[attachment] = state;
        }

        /// <summary>Target frame of an attachment for the next step.</summary>
        public float4x4 AttachmentTarget(int attachment)
        {
            CompleteStep();
            return _attachments[attachment].End;
        }

        /// <summary>Lets go of all points of an attachment; its handle is not used again.</summary>
        public void RemoveAttachment(int attachment)
        {
            CompleteStep();
            if (!IsAttachment(attachment))
            {
                return;
            }
            AttachmentState state = _attachments[attachment];
            state.Active = 0;
            _attachments[attachment] = state;
        }

        public bool IsAttachment(int attachment) =>
            attachment >= 0 && attachment < _attachments.Length && _attachments[attachment].Active != 0;

        /// <summary>Points the attachment still holds.</summary>
        public int HeldPoints(int attachment)
        {
            CompleteStep();
            int held = 0;
            for (int i = 0; i < _attachedPoints.Length; i++)
            {
                AttachedPoint p = _attachedPoints[i];
                held += p.Attachment == attachment && p.Held != 0 && IsAttachment(attachment) ? 1 : 0;
            }
            return held;
        }

        /// <summary>
        /// Force and torque the tissue exerted on an attachment in the last step, averaged over the step: the opposite of
        /// the constraint forces on its points, with the torque about the target frame's origin at the end of the step
        /// and the centroid of the points weighted by their forces. ContactNodes is the number of points held. False
        /// when the attachment held no point in the last step.
        /// </summary>
        public bool TryGetAttachmentWrench(int attachment, out ApxWrench wrench)
        {
            CompleteStep();
            wrench = default;
            if (!IsAttachment(attachment) || !_attachmentImpulses.IsCreated || attachment >= _attachmentImpulses.Length ||
                _impulseDt <= 0.0f)
            {
                return false;
            }
            ColliderImpulse total = _attachmentImpulses[attachment];
            if (total.Contacts <= 0.0f)
            {
                return false;
            }
            float3 origin = _attachments[attachment].End.c3.xyz;
            wrench = new ApxWrench
            {
                Force = -total.Linear / _impulseDt,
                Torque = -total.Angular / _impulseDt,
                Origin = origin,
                ContactCentroid = total.Weight > 0.0f ? total.Centroid / total.Weight : origin,
                ContactNodes = HeldPoints(attachment),
            };
            return true;
        }

        private void CreateAttachments()
        {
            _attachments = new NativeList<AttachmentState>(0, Allocator.Persistent);
            _attachedPoints = new NativeList<AttachedPoint>(0, Allocator.Persistent);
        }

        private void DisposeAttachments()
        {
            if (_attachments.IsCreated) _attachments.Dispose();
            if (_attachedPoints.IsCreated) _attachedPoints.Dispose();
            DisposeIfCreated(ref _attachmentImpulses);
        }

        /// <summary>Before a step: the target frames move from where the last step left them, and the impulses start at zero.</summary>
        private void BeginAttachments()
        {
            _attachmentsActive = false;
            for (int i = 0; i < _attachedPoints.Length && !_attachmentsActive; i++)
            {
                _attachmentsActive = _attachedPoints[i].Held != 0 && _attachments[_attachedPoints[i].Attachment].Active != 0;
            }
            if (!_attachmentImpulses.IsCreated || _attachmentImpulses.Length != _attachments.Length)
            {
                DisposeIfCreated(ref _attachmentImpulses);
                _attachmentImpulses = new NativeArray<ColliderImpulse>(_attachments.Length, Allocator.Persistent);
            }
            else
            {
                for (int a = 0; a < _attachmentImpulses.Length; a++)
                {
                    _attachmentImpulses[a] = default;
                }
            }
            for (int a = 0; a < _attachments.Length; a++)
            {
                AttachmentState state = _attachments[a];
                state.Start = _attachmentPrevious[a];
                _attachmentPrevious[a] = state.End;
                _attachments[a] = state;
            }
        }

        /// <summary>Schedules the attachment constraints of substep <paramref name="s"/> after <paramref name="handle"/>.</summary>
        private JobHandle ScheduleAttachments(JobHandle handle, int s)
        {
            if (!_attachmentsActive)
            {
                return handle;
            }
            float h = _step.H;
            return new AttachmentJob
            {
                Positions = Positions,
                Weights = _weights,
                CopyNodes = CopyNodes.AsArray(),
                MaterialPoints = _materialPoints.AsArray(),
                Attachments = _attachments.AsArray(),
                Points = _attachedPoints.AsArray(),
                Impulses = _attachmentImpulses,
                Time = (s + 1) / (float)_step.Substeps,
                InverseH = 1.0f / h,
                InverseH2 = 1.0f / (h * h),
            }.Schedule(handle);
        }
    }

    /// <summary>
    /// One XPBD pass over the attached points (see <see cref="SoftBodySimulation"/>' attachments) at the end of a
    /// substep, in order. A point x = sum w_k x_k (trilinear weights of its copy's corners) is pulled to its target t:
    /// with lambda starting at zero each substep, dlambda = -(x - t) / (sum w_k^2 m_k^-1 + compliance / h^2) and each
    /// corner moves by w_k m_k^-1 dlambda.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct AttachmentJob : IJob
    {
        public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<MaterialPoint> MaterialPoints;
        [ReadOnly] public NativeArray<AttachmentState> Attachments;
        public NativeArray<AttachedPoint> Points;
        public NativeArray<ColliderImpulse> Impulses;
        /// <summary>Time of the end of the substep within the step, 0 to 1.</summary>
        public float Time;
        public float InverseH;
        public float InverseH2;

        public void Execute()
        {
            for (int i = 0; i < Points.Length; i++)
            {
                AttachedPoint p = Points[i];
                if (p.Held == 0)
                {
                    continue;
                }
                AttachmentState a = Attachments[p.Attachment];
                MaterialPoint m = MaterialPoints[p.Point];
                if (a.Active == 0 || m.Copy < 0)
                {
                    p.Held = 0;
                    Points[i] = p;
                    continue;
                }
                int b = m.Copy * Lattice.CornerCount;
                float3 x = float3.zero;
                float w = 0.0f;
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int n = CopyNodes[b + k];
                    float wk = Geometry.TrilinearWeight(k, m.U);
                    x += wk * Positions[n];
                    w += wk * wk * Weights[n];
                }
                float3 target = math.lerp(math.transform(a.Start, p.Local), math.transform(a.End, p.Local), Time);
                float denominator = w + a.Compliance * InverseH2;
                if (denominator <= 0.0f)
                {
                    continue;
                }
                float3 dlambda = (target - x) / denominator;
                if (math.length(dlambda) * InverseH2 > a.BreakForce)
                {
                    p.Held = 0;
                    Points[i] = p;
                    continue;
                }
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int n = CopyNodes[b + k];
                    Positions[n] += Geometry.TrilinearWeight(k, m.U) * Weights[n] * dlambda;
                }
                // The momentum the corners gained: sum m_k (w_k m_k^-1 dlambda) / h = dlambda / h.
                float3 impulse = dlambda * InverseH;
                ColliderImpulse sum = Impulses[p.Attachment];
                sum.Add(x, a.End.c3.xyz, impulse, math.length(impulse));
                Impulses[p.Attachment] = sum;
            }
        }
    }
}
