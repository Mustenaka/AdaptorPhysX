using System;
using APEX.Native;
using UnityEngine;

namespace APEX.Usage
{
    public sealed class ApxParticlePickTarget : MonoBehaviour
    {
        public uint particleId;
    }

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

        [Header("Pointer input")]
        public Camera pointerCamera;
        public LayerMask interactionMask = ~0;
        public float maximumRayDistance = 100.0F;
        public bool sampleLegacyMouseInput;

        [Header("抓取（兼容 Drag / Pin）")]
        public ApexDrag dragHandle;
        public ApexPin pinHandle;
        public uint nativeGrabParticleId;

        private BladeTrajectoryBuffer _trajectory;
        private GrabCommandBuffer _grabCommands;
        private NativeWorld _nativeWorld;
        private InteractionRecorder _recorder;
        private InteractionReplayCursor _replay;
        private readonly ApxKinematicTarget[] _targetBatch = new ApxKinematicTarget[1];
        private bool _cutHeld;
        private bool _grabHeld;
        private bool _hasPointerPosition;
        private Vector3 _pointerPosition;
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

        public void SetPointerSample(Vector3 worldPosition, bool cutHeld, bool grabHeld)
        {
            if (!IsFinite(worldPosition))
            {
                throw new ArgumentOutOfRangeException(nameof(worldPosition));
            }

            _pointerPosition = worldPosition;
            _hasPointerPosition = true;
            _cutHeld = cutHeld;
            _grabHeld = grabHeld;
        }

        public bool TrySubmitPointerRay(Ray ray, bool cutHeld, bool grabHeld)
        {
            if (!IsFinite(ray.origin) ||
                !IsFinite(ray.direction) ||
                ray.direction.sqrMagnitude <= 0.0F ||
                !IsFinite(maximumRayDistance) ||
                maximumRayDistance <= 0.0F)
            {
                throw new ArgumentOutOfRangeException(nameof(ray));
            }

            if (!Physics.Raycast(
                    ray,
                    out RaycastHit hit,
                    maximumRayDistance,
                    interactionMask,
                    QueryTriggerInteraction.Ignore))
            {
                return false;
            }

            ApxParticlePickTarget pickTarget =
                hit.collider.GetComponentInParent<ApxParticlePickTarget>();
            if (pickTarget != null)
            {
                nativeGrabParticleId = pickTarget.particleId;
            }
            SetPointerSample(hit.point, cutHeld, grabHeld);
            return true;
        }

        public void BeginRecording()
        {
            _recorder = new InteractionRecorder(Mathf.Max(1, pendingCapacity));
        }

        public InteractionReplayTimeline EndRecording()
        {
            if (_recorder == null)
            {
                throw new InvalidOperationException("No interaction recording is active.");
            }

            InteractionReplayTimeline timeline = _recorder.BuildTimeline();
            _recorder = null;
            return timeline;
        }

        public void BeginReplay(InteractionReplayTimeline timeline)
        {
            if (timeline == null)
            {
                throw new ArgumentNullException(nameof(timeline));
            }

            _trajectory = new BladeTrajectoryBuffer(
                Mathf.Max(0.0F, minimumSampleDistance),
                Mathf.Max(0.0F, cutRadius),
                Mathf.Max(1, pendingCapacity));
            _grabCommands = new GrabCommandBuffer(Mathf.Max(1, pendingCapacity));
            _fixedTick = 0;
            _cutHeld = false;
            _grabHeld = false;
            _replay = new InteractionReplayCursor(timeline);
        }

        public void StopReplay()
        {
            _replay = null;
        }

        private void Awake()
        {
            _trajectory = new BladeTrajectoryBuffer(
                Mathf.Max(0.0F, minimumSampleDistance),
                Mathf.Max(0.0F, cutRadius),
                Mathf.Max(1, pendingCapacity));
            _grabCommands = new GrabCommandBuffer(Mathf.Max(1, pendingCapacity));
        }

        private void Update()
        {
            if (!sampleLegacyMouseInput)
            {
                return;
            }

            Camera camera = pointerCamera != null ? pointerCamera : Camera.main;
            bool cutHeld = Input.GetMouseButton(0);
            bool grabHeld = Input.GetMouseButton(1);
            if (camera != null && (cutHeld || grabHeld))
            {
                TrySubmitPointerRay(
                    camera.ScreenPointToRay(Input.mousePosition),
                    cutHeld,
                    grabHeld);
            }
            else
            {
                _cutHeld = false;
                _grabHeld = false;
            }
        }

        private void FixedUpdate()
        {
            if (_fixedTick == ulong.MaxValue)
            {
                enabled = false;
                throw new InvalidOperationException("Blade interactor fixed-tick counter is exhausted.");
            }
            ++_fixedTick;

            if (_replay != null)
            {
                while (_replay.TryDequeue(_fixedTick, out RecordedInteractionEvent item))
                {
                    ApplyReplayEvent(item);
                }
                return;
            }

            Transform sampleTransform = bladeTip != null ? bladeTip : transform;
            Transform normalTransform =
                sideNormalSource != null ? sideNormalSource : sampleTransform;
            ApxVec3 position = ToApx(
                _hasPointerPosition ? _pointerPosition : sampleTransform.position);
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

            while (_trajectory.TryDequeue(out BladeTrajectorySegment segment))
            {
                _recorder?.Record(segment);
                ApplyCut(segment.Query);
            }
        }

        private void UpdateGrab(ApxVec3 position)
        {
            bool hasHandle = _nativeWorld != null || dragHandle != null || pinHandle != null;
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

            while (_grabCommands.TryDequeue(out GrabCommand command))
            {
                _recorder?.Record(command);
                ApplyGrab(command);
            }
        }

        private void ApplyReplayEvent(RecordedInteractionEvent item)
        {
            switch (item.Kind)
            {
                case InteractionEventKind.Cut:
                    ApplyCut(item.CutQuery);
                    break;
                case InteractionEventKind.GrabBegin:
                    ApplyGrab(ToGrabCommand(item, GrabCommandPhase.Begin));
                    break;
                case InteractionEventKind.GrabMove:
                    ApplyGrab(ToGrabCommand(item, GrabCommandPhase.Move));
                    break;
                case InteractionEventKind.GrabEnd:
                    ApplyGrab(ToGrabCommand(item, GrabCommandPhase.End));
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(item));
            }
        }

        private void ApplyCut(ApxCutQuery query)
        {
            if (_nativeWorld != null && renderMeshTarget != null)
            {
                renderMeshTarget.ApplyCoupledWorldCut(_nativeWorld, query);
            }
            else if (_nativeWorld != null)
            {
                _nativeWorld.Cut(query);
            }
            else if (renderMeshTarget != null)
            {
                renderMeshTarget.ApplyPreciseWorldCut(query);
            }
        }

        private void ApplyGrab(GrabCommand command)
        {
            bool active = command.Phase != GrabCommandPhase.End;
            if (_nativeWorld != null)
            {
                _targetBatch[0] =
                    new ApxKinematicTarget(command.ParticleId, active, command.Position);
                _nativeWorld.SetKinematicTargets(_targetBatch);
            }

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

        private static GrabCommand ToGrabCommand(
            RecordedInteractionEvent item,
            GrabCommandPhase phase)
        {
            return new GrabCommand(
                item.SequenceId,
                item.FixedTick,
                item.ParticleId,
                phase,
                item.Position);
        }

        private uint GetGrabParticleId()
        {
            if (dragHandle != null)
            {
                return checked((uint)dragHandle.particleIndex);
            }
            if (pinHandle != null)
            {
                return checked((uint)pinHandle.particleIndex);
            }
            return nativeGrabParticleId;
        }

        private static ApxVec3 ToApx(Vector3 value)
        {
            return new ApxVec3(value.x, value.y, value.z);
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
