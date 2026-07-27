using System;

namespace APEX.Native
{
    /// <summary>
    /// A zero-allocation, read-only view of mapped native positions.
    /// The view is valid only until its owning world is unmapped or disposed.
    /// </summary>
    public ref struct MappedPositionSnapshot
    {
        private readonly NativeWorld _owner;
        private readonly float[] _positions;
        private readonly ulong _generation;
        private bool _disposed;

        internal MappedPositionSnapshot(
            NativeWorld owner,
            float[] positions,
            int count,
            ulong generation)
        {
            _owner = owner;
            _positions = positions;
            _generation = generation;
            Count = count;
            _disposed = false;
        }

        public int Count { get; }

        public ApxVec3 this[int index]
        {
            get
            {
                ThrowIfInvalid();
                if ((uint)index >= (uint)Count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }

                int scalarIndex = checked(index * 3);
                return new ApxVec3(
                    _positions[scalarIndex],
                    _positions[scalarIndex + 1],
                    _positions[scalarIndex + 2]);
            }
        }

        public void CopyTo(Span<ApxVec3> destination)
        {
            ThrowIfInvalid();
            if (destination.Length < Count)
            {
                throw new ArgumentException(
                    "The destination must have room for every mapped position.",
                    nameof(destination));
            }

            for (int index = 0; index < Count; ++index)
            {
                int scalarIndex = checked(index * 3);
                destination[index] = new ApxVec3(
                    _positions[scalarIndex],
                    _positions[scalarIndex + 1],
                    _positions[scalarIndex + 2]);
            }
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            if (_owner != null)
            {
                _owner.UnmapPositions(_generation);
            }

            _disposed = true;
        }

        private void ThrowIfInvalid()
        {
            if (_disposed ||
                _owner == null ||
                !_owner.IsPositionMapActive(_generation))
            {
                throw new ObjectDisposedException(nameof(MappedPositionSnapshot));
            }
        }
    }
}
