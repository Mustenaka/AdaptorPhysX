using System;
using System.Runtime.InteropServices;

namespace APEX.Native
{
    public readonly struct PhaseOneReplayCommand
    {
        public PhaseOneReplayCommand(uint sequenceId, ulong fixedTick, ApxCutQuery query)
        {
            SequenceId = sequenceId;
            FixedTick = fixedTick;
            Query = query;
        }

        public uint SequenceId { get; }

        public ulong FixedTick { get; }

        public ApxCutQuery Query { get; }
    }

    /// <summary>
    /// Immutable fixed-tick command stream. Stable sequence IDs are contiguous
    /// and ticks are non-decreasing; multiple commands on one tick retain FIFO
    /// order.
    /// </summary>
    public sealed class PhaseOneReplayTimeline
    {
        private readonly PhaseOneReplayCommand[] _commands;

        public PhaseOneReplayTimeline(PhaseOneReplayCommand[] commands)
        {
            if (commands == null)
            {
                throw new ArgumentNullException(nameof(commands));
            }

            PhaseOneReplayCommand[] validated =
                (PhaseOneReplayCommand[])commands.Clone();
            ulong previousTick = 0;
            for (int index = 0; index < validated.Length; ++index)
            {
                PhaseOneReplayCommand command = validated[index];
                if (command.SequenceId != (uint)index ||
                    command.FixedTick == 0 ||
                    (index != 0 && command.FixedTick < previousTick) ||
                    !IsValid(command.Query))
                {
                    throw new ArgumentException(
                        "Replay commands must have contiguous IDs, non-decreasing positive ticks, and valid cut queries.",
                        nameof(commands));
                }
                previousTick = command.FixedTick;
            }

            _commands = validated;
            ulong hash = PhaseOneReplayHash.Begin();
            PhaseOneReplayHash.Append(ref hash, (uint)_commands.Length);
            for (int index = 0; index < _commands.Length; ++index)
            {
                PhaseOneReplayHash.Append(ref hash, _commands[index]);
            }
            ContentHash = hash;
        }

        public int Count => _commands.Length;

        public ulong ContentHash { get; }

        public PhaseOneReplayCommand this[int index] => _commands[index];

        private static bool IsValid(ApxCutQuery query)
        {
            if (!IsFinite(query.Start) ||
                !IsFinite(query.End) ||
                !IsFinite(query.SideNormal) ||
                !IsFinite(query.Radius) ||
                query.Radius < 0.0F)
            {
                return false;
            }

            float segmentX = query.End.X - query.Start.X;
            float segmentY = query.End.Y - query.Start.Y;
            float segmentZ = query.End.Z - query.Start.Z;
            float normalSquared =
                query.SideNormal.X * query.SideNormal.X +
                query.SideNormal.Y * query.SideNormal.Y +
                query.SideNormal.Z * query.SideNormal.Z;
            float segmentSquared =
                segmentX * segmentX + segmentY * segmentY + segmentZ * segmentZ;
            float radiusSquared = query.Radius * query.Radius;
            return IsFinite(normalSquared) &&
                normalSquared > 0.0F &&
                IsFinite(segmentSquared) &&
                segmentSquared > 0.0F &&
                IsFinite(radiusSquared);
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

    public sealed class PhaseOneReplayCursor
    {
        private readonly PhaseOneReplayTimeline _timeline;
        private int _nextCommandIndex;
        private ulong _lastTick;
        private bool _hasAdvanced;

        public PhaseOneReplayCursor(PhaseOneReplayTimeline timeline)
        {
            _timeline = timeline ?? throw new ArgumentNullException(nameof(timeline));
        }

        public int NextCommandIndex => _nextCommandIndex;

        public bool IsComplete => _nextCommandIndex == _timeline.Count;

        public bool TryDequeue(ulong fixedTick, out PhaseOneReplayCommand command)
        {
            if (fixedTick == 0 || (_hasAdvanced && fixedTick < _lastTick))
            {
                throw new ArgumentOutOfRangeException(nameof(fixedTick));
            }
            if (_nextCommandIndex < _timeline.Count &&
                _timeline[_nextCommandIndex].FixedTick < fixedTick)
            {
                throw new InvalidOperationException(
                    "A fixed-tick replay command was skipped.");
            }

            _hasAdvanced = true;
            _lastTick = fixedTick;
            if (_nextCommandIndex < _timeline.Count &&
                _timeline[_nextCommandIndex].FixedTick == fixedTick)
            {
                command = _timeline[_nextCommandIndex++];
                return true;
            }

            command = default;
            return false;
        }
    }

    public static class PhaseOneReplayHash
    {
        private const ulong OffsetBasis = 14695981039346656037UL;
        private const ulong Prime = 1099511628211UL;

        [StructLayout(LayoutKind.Explicit)]
        private struct FloatBits
        {
            [FieldOffset(0)]
            public float Value;

            [FieldOffset(0)]
            public uint Bits;
        }

        public static ulong Begin()
        {
            return OffsetBasis;
        }

        public static void Append(ref ulong hash, PhaseOneReplayCommand command)
        {
            Append(ref hash, command.SequenceId);
            Append(ref hash, command.FixedTick);
            Append(ref hash, command.Query);
        }

        public static void Append(ref ulong hash, ApxCutQuery query)
        {
            Append(ref hash, query.Start);
            Append(ref hash, query.End);
            Append(ref hash, query.SideNormal);
            Append(ref hash, query.Radius);
        }

        public static void Append(ref ulong hash, ApxCutResult result)
        {
            Append(ref hash, result.CutId);
            Append(ref hash, result.CutConstraintCount);
            Append(ref hash, result.SplitParticleCount);
            Append(ref hash, result.FirstSplitParticleId);
        }

        public static void Append(ref ulong hash, ApxVec3[] positions, int count)
        {
            if (positions == null)
            {
                throw new ArgumentNullException(nameof(positions));
            }
            if (count < 0 || count > positions.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(count));
            }

            Append(ref hash, (uint)count);
            for (int index = 0; index < count; ++index)
            {
                Append(ref hash, positions[index]);
            }
        }

        public static void Append(ref ulong hash, uint value)
        {
            for (int shift = 0; shift < 32; shift += 8)
            {
                AppendByte(ref hash, (byte)(value >> shift));
            }
        }

        public static void Append(ref ulong hash, ulong value)
        {
            for (int shift = 0; shift < 64; shift += 8)
            {
                AppendByte(ref hash, (byte)(value >> shift));
            }
        }

        public static void Append(ref ulong hash, float value)
        {
            FloatBits bits = new FloatBits { Value = value };
            Append(ref hash, bits.Bits);
        }

        private static void Append(ref ulong hash, ApxVec3 value)
        {
            Append(ref hash, value.X);
            Append(ref hash, value.Y);
            Append(ref hash, value.Z);
        }

        private static void AppendByte(ref ulong hash, byte value)
        {
            hash ^= value;
            hash *= Prime;
        }
    }
}
