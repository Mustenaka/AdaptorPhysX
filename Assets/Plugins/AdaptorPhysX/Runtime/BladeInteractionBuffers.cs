using System;
using System.Collections.Generic;

namespace APEX.Native
{
    public enum BladeAppendResult : uint
    {
        Accepted = 0,
        BelowMinimumDistance = 1,
        CapacityExceeded = 2,
    }

    public struct BladeTrajectorySegment
    {
        public uint SegmentId;
        public ulong FixedTick;
        public ApxCutQuery Query;

        public BladeTrajectorySegment(uint segmentId, ulong fixedTick, ApxCutQuery query)
        {
            SegmentId = segmentId;
            FixedTick = fixedTick;
            Query = query;
        }
    }

    /// <summary>
    /// Fixed-tick FIFO for blade samples. Short samples do not move the
    /// accepted anchor, so frame partitioning cannot change emitted geometry.
    /// </summary>
    public sealed class BladeTrajectoryBuffer
    {
        private readonly float _minimumDistanceSquared;
        private readonly float _radius;
        private readonly int _capacity;
        private readonly Queue<BladeTrajectorySegment> _pending;
        private ApxVec3 _anchor;
        private ulong _lastFixedTick;
        private uint _nextSegmentId;
        private bool _strokeActive;

        public BladeTrajectoryBuffer(float minimumSampleDistance, float radius, int capacity)
        {
            float distanceSquared = minimumSampleDistance * minimumSampleDistance;
            float radiusSquared = radius * radius;
            if (!IsFinite(minimumSampleDistance) ||
                minimumSampleDistance < 0.0F ||
                !IsFinite(radius) ||
                radius < 0.0F ||
                !IsFinite(distanceSquared) ||
                !IsFinite(radiusSquared))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(minimumSampleDistance),
                    "Sampling distance and radius must be finite and non-negative.");
            }
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity));
            }

            _minimumDistanceSquared = distanceSquared;
            _radius = radius;
            _capacity = capacity;
            _pending = new Queue<BladeTrajectorySegment>(capacity);
        }

        public bool IsStrokeActive => _strokeActive;

        public int PendingCount => _pending.Count;

        public uint NextSegmentId => _nextSegmentId;

        public void BeginStroke(ApxVec3 position, ApxVec3 sideNormal, ulong fixedTick)
        {
            if (_strokeActive)
            {
                throw new InvalidOperationException("A blade stroke is already active.");
            }
            ValidateSample(position, sideNormal);
            _anchor = position;
            _lastFixedTick = fixedTick;
            _strokeActive = true;
        }

        public BladeAppendResult AppendSample(
            ApxVec3 position,
            ApxVec3 sideNormal,
            ulong fixedTick,
            out BladeTrajectorySegment segment)
        {
            segment = default;
            if (!_strokeActive)
            {
                throw new InvalidOperationException("A blade stroke must begin before sampling.");
            }
            if (fixedTick < _lastFixedTick)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(fixedTick),
                    "Fixed ticks must be monotonic.");
            }
            ValidateSample(position, sideNormal);
            ApxVec3 delta = Subtract(position, _anchor);
            float distanceSquared = Dot(delta, delta);
            if (!IsFinite(distanceSquared))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(position),
                    "Blade sample distance overflowed.");
            }
            if (distanceSquared < _minimumDistanceSquared || distanceSquared == 0.0F)
            {
                _lastFixedTick = fixedTick;
                return BladeAppendResult.BelowMinimumDistance;
            }
            if (_pending.Count >= _capacity || _nextSegmentId == uint.MaxValue)
            {
                return BladeAppendResult.CapacityExceeded;
            }

            segment = new BladeTrajectorySegment(
                _nextSegmentId,
                fixedTick,
                new ApxCutQuery(_anchor, position, sideNormal, _radius));
            _pending.Enqueue(segment);
            _anchor = position;
            _lastFixedTick = fixedTick;
            ++_nextSegmentId;
            return BladeAppendResult.Accepted;
        }

        public void EndStroke(ulong fixedTick)
        {
            if (!_strokeActive)
            {
                throw new InvalidOperationException("No blade stroke is active.");
            }
            if (fixedTick < _lastFixedTick)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(fixedTick),
                    "Fixed ticks must be monotonic.");
            }

            _lastFixedTick = fixedTick;
            _strokeActive = false;
        }

        public BladeTrajectorySegment[] Drain()
        {
            BladeTrajectorySegment[] result = _pending.ToArray();
            _pending.Clear();
            return result;
        }

        public bool TryDequeue(out BladeTrajectorySegment segment)
        {
            if (_pending.Count == 0)
            {
                segment = default;
                return false;
            }

            segment = _pending.Dequeue();
            return true;
        }

        private static void ValidateSample(ApxVec3 position, ApxVec3 sideNormal)
        {
            float normalLengthSquared = Dot(sideNormal, sideNormal);
            if (!IsFinite(position) ||
                !IsFinite(sideNormal) ||
                !IsFinite(normalLengthSquared) ||
                normalLengthSquared <= 0.0F)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(position),
                    "Blade position and non-degenerate side normal must be finite.");
            }
        }

        private static ApxVec3 Subtract(ApxVec3 first, ApxVec3 second)
        {
            return new ApxVec3(
                first.X - second.X,
                first.Y - second.Y,
                first.Z - second.Z);
        }

        private static float Dot(ApxVec3 first, ApxVec3 second)
        {
            return (first.X * second.X + first.Y * second.Y) + first.Z * second.Z;
        }

        private static bool IsFinite(ApxVec3 value)
        {
            return IsFinite(value.X) && IsFinite(value.Y) && IsFinite(value.Z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }

    public enum GrabCommandPhase : uint
    {
        Begin = 0,
        Move = 1,
        End = 2,
    }

    public struct GrabCommand
    {
        public uint SequenceId;
        public ulong FixedTick;
        public uint ParticleId;
        public GrabCommandPhase Phase;
        public ApxVec3 Position;

        public GrabCommand(
            uint sequenceId,
            ulong fixedTick,
            uint particleId,
            GrabCommandPhase phase,
            ApxVec3 position)
        {
            SequenceId = sequenceId;
            FixedTick = fixedTick;
            ParticleId = particleId;
            Phase = phase;
            Position = position;
        }
    }

    public sealed class GrabCommandBuffer
    {
        private readonly int _capacity;
        private readonly Queue<GrabCommand> _pending;
        private uint _particleId;
        private uint _nextSequenceId;
        private ulong _lastFixedTick;
        private bool _grabActive;

        public GrabCommandBuffer(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity));
            }
            _capacity = capacity;
            _pending = new Queue<GrabCommand>(capacity);
        }

        public bool IsGrabActive => _grabActive;

        public int PendingCount => _pending.Count;

        public bool Begin(uint particleId, ApxVec3 position, ulong fixedTick)
        {
            if (_grabActive)
            {
                throw new InvalidOperationException("A grab session is already active.");
            }
            ValidatePosition(position);
            if (!TryAppend(particleId, position, fixedTick, GrabCommandPhase.Begin))
            {
                return false;
            }

            _particleId = particleId;
            _lastFixedTick = fixedTick;
            _grabActive = true;
            return true;
        }

        public bool Move(ApxVec3 position, ulong fixedTick)
        {
            EnsureActiveAndTick(position, fixedTick);
            if (!TryAppend(_particleId, position, fixedTick, GrabCommandPhase.Move))
            {
                return false;
            }

            _lastFixedTick = fixedTick;
            return true;
        }

        public bool End(ApxVec3 position, ulong fixedTick)
        {
            EnsureActiveAndTick(position, fixedTick);
            if (!TryAppend(_particleId, position, fixedTick, GrabCommandPhase.End))
            {
                return false;
            }

            _lastFixedTick = fixedTick;
            _grabActive = false;
            return true;
        }

        public GrabCommand[] Drain()
        {
            GrabCommand[] result = _pending.ToArray();
            _pending.Clear();
            return result;
        }

        public bool TryDequeue(out GrabCommand command)
        {
            if (_pending.Count == 0)
            {
                command = default;
                return false;
            }

            command = _pending.Dequeue();
            return true;
        }

        private bool TryAppend(
            uint particleId,
            ApxVec3 position,
            ulong fixedTick,
            GrabCommandPhase phase)
        {
            if (_pending.Count >= _capacity || _nextSequenceId == uint.MaxValue)
            {
                return false;
            }
            _pending.Enqueue(
                new GrabCommand(_nextSequenceId, fixedTick, particleId, phase, position));
            ++_nextSequenceId;
            return true;
        }

        private void EnsureActiveAndTick(ApxVec3 position, ulong fixedTick)
        {
            if (!_grabActive)
            {
                throw new InvalidOperationException("A grab session must begin first.");
            }
            if (fixedTick < _lastFixedTick)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(fixedTick),
                    "Fixed ticks must be monotonic.");
            }
            ValidatePosition(position);
        }

        private static void ValidatePosition(ApxVec3 position)
        {
            if (float.IsNaN(position.X) ||
                float.IsInfinity(position.X) ||
                float.IsNaN(position.Y) ||
                float.IsInfinity(position.Y) ||
                float.IsNaN(position.Z) ||
                float.IsInfinity(position.Z))
            {
                throw new ArgumentOutOfRangeException(nameof(position));
            }
        }
    }
}
