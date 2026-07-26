using System;
using System.Runtime.InteropServices;

namespace APEX.Native
{
    /// <summary>
    /// Owns one native AdaptorPhysX world. This type is not thread-safe.
    /// Phase 1 mapping is a blocking GPU-to-CPU readback fallback; replace it
    /// with graphics-buffer interop when the Phase 4 zero-copy path lands.
    /// </summary>
    public sealed class NativeWorld : IDisposable
    {
        private readonly ApxWorldSafeHandle _handle;
        private bool _disposed;
        private bool _isMapped;
        private ulong _mapGeneration;
        private uint _particleCount;
        private float[] _positionReadback = Array.Empty<float>();

        private NativeWorld(ApxWorldSafeHandle handle)
        {
            _handle = handle;
        }

        public uint ParticleCount => _particleCount;

        internal bool IsPositionMapActive(ulong generation)
        {
            return !_disposed && _isMapped && _mapGeneration == generation;
        }

        public static NativeWorld Create(ApxWorldDesc description)
        {
            if (description.StructSize == 0)
            {
                description.StructSize = ApxWorldDesc.SizeInBytes;
            }

            ApxResult result = NativeMethods.ApxCreateWorld(in description, out IntPtr world);
            ApxException.ThrowIfFailed(result, "apxCreateWorld");
            if (world == IntPtr.Zero)
            {
                throw new ApxException(ApxResult.InternalError, "apxCreateWorld");
            }

            ApxWorldSafeHandle safeHandle = null;
            try
            {
                safeHandle = new ApxWorldSafeHandle(world);
                return new NativeWorld(safeHandle);
            }
            catch
            {
                if (safeHandle != null)
                {
                    safeHandle.Dispose();
                }
                else
                {
                    try
                    {
                        NativeMethods.ApxDestroyWorld(world);
                    }
                    catch
                    {
                        // Preserve the original managed construction failure.
                    }
                }

                throw;
            }
        }

        public static void GetAbiVersion(out uint major, out uint minor)
        {
            ApxResult result = NativeMethods.ApxGetAbiVersion(out major, out minor);
            ApxException.ThrowIfFailed(result, "apxGetAbiVersion");
        }

        public uint AddParticles(ApxParticleDesc[] particles)
        {
            ThrowIfDisposed();
            if (particles == null)
            {
                throw new ArgumentNullException(nameof(particles));
            }

            uint newParticleCount = checked(_particleCount + (uint)particles.Length);
            EnsureReadbackCapacity(newParticleCount);

            ApxResult result = NativeMethods.ApxAddParticles(
                _handle,
                particles.Length == 0 ? null : particles,
                checked((uint)particles.Length),
                out uint firstParticleId);
            ApxException.ThrowIfFailed(result, "apxAddParticles");
            _particleCount = newParticleCount;
            return firstParticleId;
        }

        public uint AddDistanceConstraints(ApxDistanceConstraintDesc[] constraints)
        {
            ThrowIfDisposed();
            if (constraints == null)
            {
                throw new ArgumentNullException(nameof(constraints));
            }

            ApxResult result = NativeMethods.ApxAddDistanceConstraints(
                _handle,
                constraints.Length == 0 ? null : constraints,
                checked((uint)constraints.Length),
                out uint firstConstraintId);
            ApxException.ThrowIfFailed(result, "apxAddDistanceConstraints");
            return firstConstraintId;
        }

        public void SetColliderProxies(ApxColliderProxy[] proxies)
        {
            ThrowIfDisposed();
            if (proxies == null)
            {
                throw new ArgumentNullException(nameof(proxies));
            }

            ApxResult result = NativeMethods.ApxSetColliderProxies(
                _handle,
                proxies.Length == 0 ? null : proxies,
                checked((uint)proxies.Length));
            ApxException.ThrowIfFailed(result, "apxSetColliderProxies");
        }

        public void Step(float frameDeltaTime)
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxStep(_handle, frameDeltaTime);
            ApxException.ThrowIfFailed(result, "apxStep");
        }

        public MappedPositionSnapshot MapPositions()
        {
            ThrowIfDisposed();
            if (_isMapped)
            {
                throw new ApxException(ApxResult.AlreadyMapped, "apxMapParticleBuffer");
            }

            if (_mapGeneration == ulong.MaxValue)
            {
                throw new InvalidOperationException(
                    "The position map generation counter is exhausted.");
            }

            ulong generation = _mapGeneration + 1;
            ApxResult result = NativeMethods.ApxMapParticleBuffer(_handle, out ApxBufferView view);
            ApxException.ThrowIfFailed(result, "apxMapParticleBuffer");

            try
            {
                ValidatePositionView(in view);
                if (view.ElementCount != 0)
                {
                    Marshal.Copy(
                        view.Data,
                        _positionReadback,
                        0,
                        checked((int)view.ElementCount * 3));
                }

                _mapGeneration = generation;
                _isMapped = true;
                return new MappedPositionSnapshot(
                    this,
                    _positionReadback,
                    checked((int)view.ElementCount),
                    generation);
            }
            catch
            {
                ApxResult unmapResult = NativeMethods.ApxUnmapParticleBuffer(_handle);
                ApxException.ThrowIfFailed(unmapResult, "apxUnmapParticleBuffer");
                throw;
            }
        }

        public void UnmapPositions()
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxUnmapParticleBuffer(_handle);
            ApxException.ThrowIfFailed(result, "apxUnmapParticleBuffer");
            _isMapped = false;
        }

        internal void UnmapPositions(ulong generation)
        {
            if (IsPositionMapActive(generation))
            {
                UnmapPositions();
            }
        }

        /// <summary>
        /// Copies positions into caller-owned reusable storage without a
        /// steady-state managed allocation.
        /// </summary>
        public int ReadPositionSnapshot(Span<ApxVec3> destination)
        {
            ThrowIfDisposed();
            if ((ulong)destination.Length < _particleCount)
            {
                throw new ArgumentException(
                    "The destination must have room for every particle.",
                    nameof(destination));
            }

            MappedPositionSnapshot snapshot = MapPositions();
            try
            {
                snapshot.CopyTo(destination);
                return snapshot.Count;
            }
            finally
            {
                snapshot.Dispose();
            }
        }

        public int ReadPositionSnapshot(ApxVec3[] destination)
        {
            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination));
            }

            return ReadPositionSnapshot(destination.AsSpan());
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            if (_isMapped)
            {
                // Destruction also invalidates a map, but an explicit unmap
                // keeps the managed state and native lifecycle aligned.
                NativeMethods.ApxUnmapParticleBuffer(_handle);
                _isMapped = false;
            }

            _handle.Dispose();
            _disposed = true;
        }

        private void ValidatePositionView(in ApxBufferView view)
        {
            const uint expectedStride = 12;
            ulong expectedSize = (ulong)view.ElementCount * expectedStride;

            if (view.ElementStrideBytes != expectedStride ||
                view.SizeBytes != expectedSize ||
                view.ElementCount != _particleCount ||
                view.ElementCount > int.MaxValue / 3 ||
                (view.ElementCount != 0 && view.Data == IntPtr.Zero) ||
                (view.ElementCount == 0 && view.Data != IntPtr.Zero))
            {
                throw new InvalidOperationException(
                    "apxMapParticleBuffer returned an invalid ApxVec3 position view.");
            }
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
            {
                throw new ObjectDisposedException(nameof(NativeWorld));
            }
        }

        private void EnsureReadbackCapacity(uint particleCount)
        {
            int requiredScalars = checked((int)particleCount * 3);
            if (_positionReadback.Length < requiredScalars)
            {
                _positionReadback = new float[requiredScalars];
            }
        }
    }
}
