using System;
using System.Collections.Generic;

namespace APEX.Native
{
    public enum InteractionEventKind : uint
    {
        Cut = 0,
        GrabBegin = 1,
        GrabMove = 2,
        GrabEnd = 3,
    }

    [Serializable]
    public struct RecordedInteractionEvent
    {
        public uint SequenceId;
        public ulong FixedTick;
        public InteractionEventKind Kind;
        public ApxCutQuery CutQuery;
        public uint ParticleId;
        public ApxVec3 Position;
    }

    /// <summary>
    /// Immutable, serializable-by-value fixed-tick input sequence. The core
    /// consumes only this normalized timeline, never wall-clock input state.
    /// </summary>
    public sealed class InteractionReplayTimeline
    {
        private readonly RecordedInteractionEvent[] _events;

        public InteractionReplayTimeline(RecordedInteractionEvent[] events)
        {
            if (events == null)
            {
                throw new ArgumentNullException(nameof(events));
            }

            _events = (RecordedInteractionEvent[])events.Clone();
            Validate(_events);
        }

        public int Count => _events.Length;

        public ulong ContentHash => ComputeHash(_events);

        public RecordedInteractionEvent this[int index] => _events[index];

        public RecordedInteractionEvent[] ToArray()
        {
            return (RecordedInteractionEvent[])_events.Clone();
        }

        private static void Validate(RecordedInteractionEvent[] events)
        {
            ulong previousTick = 0;
            for (int index = 0; index < events.Length; ++index)
            {
                RecordedInteractionEvent item = events[index];
                if (item.SequenceId != (uint)index ||
                    (index != 0 && item.FixedTick < previousTick))
                {
                    throw new ArgumentException(
                        "Replay events require contiguous IDs and monotonic fixed ticks.",
                        nameof(events));
                }

                switch (item.Kind)
                {
                    case InteractionEventKind.Cut:
                        if (!IsFinite(item.CutQuery.Start) ||
                            !IsFinite(item.CutQuery.End) ||
                            !IsFinite(item.CutQuery.SideNormal) ||
                            !IsFinite(item.CutQuery.Radius) ||
                            item.CutQuery.Radius < 0.0F ||
                            LengthSquared(item.CutQuery.SideNormal) <= 0.0F)
                        {
                            throw new ArgumentException(
                                "Recorded cut geometry must be finite and non-degenerate.",
                                nameof(events));
                        }
                        break;
                    case InteractionEventKind.GrabBegin:
                    case InteractionEventKind.GrabMove:
                    case InteractionEventKind.GrabEnd:
                        if (!IsFinite(item.Position))
                        {
                            throw new ArgumentException(
                                "Recorded grab targets must be finite.",
                                nameof(events));
                        }
                        break;
                    default:
                        throw new ArgumentException(
                            "Recorded interaction kind is invalid.",
                            nameof(events));
                }
                previousTick = item.FixedTick;
            }
        }

        private static ulong ComputeHash(RecordedInteractionEvent[] events)
        {
            const ulong offsetBasis = 1469598103934665603UL;
            const ulong prime = 1099511628211UL;
            ulong hash = offsetBasis;
            foreach (RecordedInteractionEvent item in events)
            {
                hash = Mix(hash, item.SequenceId, prime);
                hash = Mix(hash, item.FixedTick, prime);
                hash = Mix(hash, (uint)item.Kind, prime);
                hash = Mix(hash, item.ParticleId, prime);
                hash = Mix(hash, FloatBits(item.Position.X), prime);
                hash = Mix(hash, FloatBits(item.Position.Y), prime);
                hash = Mix(hash, FloatBits(item.Position.Z), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.Start.X), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.Start.Y), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.Start.Z), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.End.X), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.End.Y), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.End.Z), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.SideNormal.X), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.SideNormal.Y), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.SideNormal.Z), prime);
                hash = Mix(hash, FloatBits(item.CutQuery.Radius), prime);
            }
            return hash;
        }

        private static ulong Mix(ulong hash, ulong value, ulong prime)
        {
            return (hash ^ value) * prime;
        }

        private static uint FloatBits(float value)
        {
            return BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
        }

        private static float LengthSquared(ApxVec3 value)
        {
            return (value.X * value.X + value.Y * value.Y) + value.Z * value.Z;
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

    public sealed class InteractionRecorder
    {
        private readonly int _capacity;
        private readonly List<RecordedInteractionEvent> _events;

        public InteractionRecorder(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity));
            }

            _capacity = capacity;
            _events = new List<RecordedInteractionEvent>(capacity);
        }

        public int Count => _events.Count;

        public void Record(BladeTrajectorySegment segment)
        {
            Append(
                new RecordedInteractionEvent
                {
                    FixedTick = segment.FixedTick,
                    Kind = InteractionEventKind.Cut,
                    CutQuery = segment.Query,
                });
        }

        public void Record(GrabCommand command)
        {
            InteractionEventKind kind;
            switch (command.Phase)
            {
                case GrabCommandPhase.Begin:
                    kind = InteractionEventKind.GrabBegin;
                    break;
                case GrabCommandPhase.Move:
                    kind = InteractionEventKind.GrabMove;
                    break;
                case GrabCommandPhase.End:
                    kind = InteractionEventKind.GrabEnd;
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(command));
            }

            Append(
                new RecordedInteractionEvent
                {
                    FixedTick = command.FixedTick,
                    Kind = kind,
                    ParticleId = command.ParticleId,
                    Position = command.Position,
                });
        }

        public InteractionReplayTimeline BuildTimeline()
        {
            return new InteractionReplayTimeline(_events.ToArray());
        }

        private void Append(RecordedInteractionEvent item)
        {
            if (_events.Count >= _capacity)
            {
                throw new InvalidOperationException("Interaction recording capacity is exhausted.");
            }
            if (_events.Count != 0 &&
                item.FixedTick < _events[_events.Count - 1].FixedTick)
            {
                throw new InvalidOperationException(
                    "Interaction recording fixed ticks must be monotonic.");
            }

            item.SequenceId = checked((uint)_events.Count);
            _events.Add(item);
        }
    }

    public sealed class InteractionReplayCursor
    {
        private readonly InteractionReplayTimeline _timeline;
        private int _nextIndex;
        private ulong _lastRequestedTick;

        public InteractionReplayCursor(InteractionReplayTimeline timeline)
        {
            _timeline = timeline ?? throw new ArgumentNullException(nameof(timeline));
        }

        public bool IsComplete => _nextIndex == _timeline.Count;

        public bool TryDequeue(ulong fixedTick, out RecordedInteractionEvent item)
        {
            if (fixedTick < _lastRequestedTick)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(fixedTick),
                    "Replay ticks cannot move backwards.");
            }
            _lastRequestedTick = fixedTick;
            if (_nextIndex == _timeline.Count)
            {
                item = default;
                return false;
            }

            RecordedInteractionEvent next = _timeline[_nextIndex];
            if (next.FixedTick < fixedTick)
            {
                throw new InvalidOperationException(
                    "Replay advanced past an unconsumed interaction event.");
            }
            if (next.FixedTick != fixedTick)
            {
                item = default;
                return false;
            }

            item = next;
            ++_nextIndex;
            return true;
        }
    }
}
