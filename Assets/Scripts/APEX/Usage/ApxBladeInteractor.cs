using System;
using APEX.Native;
using UnityEngine;

namespace APEX.Usage
{
    /// <summary>
    /// FixedUpdate-only adapter for mouse/XR/replay cut and grab intent.
    /// Call SetCutHeld/SetGrabHeld from any input frontend; geometry samples
    /// and command commits remain fixed-tick ordered.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class ApxBladeInteractor : MonoBehaviour
    {
        [Header("刀刃")]
        public Transform bladeTip;
        public Transform sideNormalSource;
        public ObjCut renderMeshTarget;
        public float minimumSampleDistance = 0.001F;
        public float cutRadius = 0.005F;
        public int pendingCapacity = 1024;

        [Header("抓取（复用 Drag / Pin）")]
        public ApexDrag dragHandle;
        public ApexPin pinHandle;

        private BladeTrajectoryBuffer _trajectory;
        private GrabCommandBuffer _grabCommands;
        private NativeWorld _nativeWorld;
        private bool _cutHeld;
        private bool _grabHeld;
        private ulong _fixedTick;

        public ulong FixedTick => _fixedTick;

        public void BindNativeWorld(NativeWorld nativeWorld)
        {
            _nativeWorld = nativeWorld;
        }

        public void SetCutHeld(bool held)
        {
            _cutHeld = held;
        }

        public void SetGrabHeld(bool held)
        {
            _grabHeld = held;
        }

        private void Awake()
        {
            _trajectory = new BladeTrajectoryBuffer(
                Mathf.Max(0.0F, minimumSampleDistance),
                Mathf.Max(0.0F, cutRadius),
                Mathf.Max(1, pendingCapacity));
            _grabCommands = new GrabCommandBuffer(Mathf.Max(1, pendingCapacity));
        }

        private void FixedUpdate()
        {
            if (_fixedTick == ulong.MaxValue)
            {
                enabled = false;
                throw new InvalidOperationException("Blade interactor fixed-tick counter is exhausted.");
            }
            ++_fixedTick;

            Transform sampleTransform = bladeTip != null ? bladeTip : transform;
            Transform normalTransform =
                sideNormalSource != null ? sideNormalSource : sampleTransform;
            ApxVec3 position = ToApx(sampleTransform.position);
            ApxVec3 sideNormal = ToApx(normalTransform.forward);
            UpdateCut(position, sideNormal);
            UpdateGrab(position);
        }

        private void UpdateCut(ApxVec3 position, ApxVec3 sideNormal)
        {
            if (_cutHeld)
            {
                if (!_trajectory.IsStrokeActive)
                {
                    _trajectory.BeginStroke(position, sideNormal, _fixedTick);
                }
                else
                {
                    BladeAppendResult append = _trajectory.AppendSample(
                        position,
                        sideNormal,
                        _fixedTick,
                        out _);
                    if (append == BladeAppendResult.CapacityExceeded)
                    {
                        Debug.LogWarning("Blade trajectory capacity reached; pending cuts are committed now.");
                    }
                }
            }
            else if (_trajectory.IsStrokeActive)
            {
                _trajectory.EndStroke(_fixedTick);
            }

            BladeTrajectorySegment[] pending = _trajectory.Drain();
            for (int index = 0; index < pending.Length; ++index)
            {
                ApxCutQuery query = pending[index].Query;
                if (_nativeWorld != null)
                {
                    _nativeWorld.Cut(query);
                }
                if (renderMeshTarget != null)
                {
                    renderMeshTarget.ApplyPreciseWorldCut(query);
                }
            }
        }

        private void UpdateGrab(ApxVec3 position)
        {
            bool hasHandle = dragHandle != null || pinHandle != null;
            if (_grabHeld && hasHandle)
            {
                if (!_grabCommands.IsGrabActive)
                {
                    _grabCommands.Begin(GetGrabParticleId(), position, _fixedTick);
                }
                else
                {
                    _grabCommands.Move(position, _fixedTick);
                }
            }
            else if (_grabCommands.IsGrabActive)
            {
                _grabCommands.End(position, _fixedTick);
            }

            GrabCommand[] pending = _grabCommands.Drain();
            for (int index = 0; index < pending.Length; ++index)
            {
                GrabCommand command = pending[index];
                Vector3 commandPosition = new Vector3(
                    command.Position.X,
                    command.Position.Y,
                    command.Position.Z);
                int particleIndex = checked((int)command.ParticleId);
                if (dragHandle != null)
                {
                    dragHandle.particleIndex = particleIndex;
                    dragHandle.transform.position = commandPosition;
                    dragHandle.dragPosition = commandPosition;
                }
                if (pinHandle != null)
                {
                    pinHandle.particleIndex = particleIndex;
                    pinHandle.transform.position = commandPosition;
                    pinHandle.pinPosition = commandPosition;
                }
            }
        }

        private uint GetGrabParticleId()
        {
            int particleIndex = dragHandle != null
                ? dragHandle.particleIndex
                : pinHandle.particleIndex;
            return checked((uint)particleIndex);
        }

        private static ApxVec3 ToApx(Vector3 value)
        {
            return new ApxVec3(value.x, value.y, value.z);
        }
    }
}
