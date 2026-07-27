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
        private bool _cutCapacityPrepared;
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

        public uint AddClothDistanceConstraints(ApxClothDistanceConstraintDesc[] constraints)
        {
            ThrowIfDisposed();
            if (constraints == null)
            {
                throw new ArgumentNullException(nameof(constraints));
            }

            ApxResult result = NativeMethods.ApxAddClothDistanceConstraints(
                _handle,
                constraints.Length == 0 ? null : constraints,
                checked((uint)constraints.Length),
                out uint firstConstraintId);
            ApxException.ThrowIfFailed(result, "apxAddClothDistanceConstraints");
            return firstConstraintId;
        }

        public uint AddBendConstraints(ApxBendConstraintDesc[] constraints)
        {
            ThrowIfDisposed();
            if (constraints == null)
            {
                throw new ArgumentNullException(nameof(constraints));
            }

            ApxResult result = NativeMethods.ApxAddBendConstraints(
                _handle,
                constraints.Length == 0 ? null : constraints,
                checked((uint)constraints.Length),
                out uint firstConstraintId);
            ApxException.ThrowIfFailed(result, "apxAddBendConstraints");
            return firstConstraintId;
        }

        public uint[] GetBrokenClothDistanceConstraintIds()
        {
            ThrowIfDisposed();
            uint count = GetBrokenClothDistanceConstraintCount();
            if (count == 0)
            {
                return Array.Empty<uint>();
            }

            uint[] ids = new uint[checked((int)count)];
            int written = GetBrokenClothDistanceConstraintIds(ids);
            if ((uint)written != count)
            {
                throw new InvalidOperationException(
                    "The native broken-constraint count changed during a synchronous query.");
            }

            return ids;
        }

        public uint GetBrokenClothDistanceConstraintCount()
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxGetBrokenClothDistanceConstraintIds(
                _handle,
                null,
                0,
                out uint count);
            ApxException.ThrowIfFailed(result, "apxGetBrokenClothDistanceConstraintIds");
            return count;
        }

        public int GetBrokenClothDistanceConstraintIds(uint[] destination)
        {
            ThrowIfDisposed();
            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination));
            }

            ApxResult result = NativeMethods.ApxGetBrokenClothDistanceConstraintIds(
                _handle,
                destination.Length == 0 ? null : destination,
                checked((uint)destination.Length),
                out uint written);
            ApxException.ThrowIfFailed(result, "apxGetBrokenClothDistanceConstraintIds");
            return checked((int)written);
        }

        public void SetRenderVertexBindings(ApxRenderVertexBindingDesc[] bindings)
        {
            ThrowIfDisposed();
            if (bindings == null)
            {
                throw new ArgumentNullException(nameof(bindings));
            }

            ApxResult result = NativeMethods.ApxSetRenderVertexBindings(
                _handle,
                bindings.Length == 0 ? null : bindings,
                checked((uint)bindings.Length));
            ApxException.ThrowIfFailed(result, "apxSetRenderVertexBindings");
        }

        public void BindRenderVertices(
            ApxVec3[] renderRestPositions,
            uint[] simulationTriangleParticleIds)
        {
            ThrowIfDisposed();
            if (renderRestPositions == null)
            {
                throw new ArgumentNullException(nameof(renderRestPositions));
            }
            if (simulationTriangleParticleIds == null)
            {
                throw new ArgumentNullException(nameof(simulationTriangleParticleIds));
            }
            if (simulationTriangleParticleIds.Length % 3 != 0)
            {
                throw new ArgumentException(
                    "Simulation triangle particle IDs must be a flattened triangle array.",
                    nameof(simulationTriangleParticleIds));
            }

            ApxResult result = NativeMethods.ApxBindRenderVertices(
                _handle,
                renderRestPositions.Length == 0 ? null : renderRestPositions,
                checked((uint)renderRestPositions.Length),
                simulationTriangleParticleIds.Length == 0 ? null : simulationTriangleParticleIds,
                checked((uint)(simulationTriangleParticleIds.Length / 3)));
            ApxException.ThrowIfFailed(result, "apxBindRenderVertices");
        }

        public ApxRenderVertexBindingDesc[] GetRenderVertexBindings()
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxGetRenderVertexBindings(
                _handle,
                null,
                0,
                out uint count);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexBindings");
            if (count == 0)
            {
                return Array.Empty<ApxRenderVertexBindingDesc>();
            }

            ApxRenderVertexBindingDesc[] bindings =
                new ApxRenderVertexBindingDesc[checked((int)count)];
            result = NativeMethods.ApxGetRenderVertexBindings(
                _handle,
                bindings,
                count,
                out uint written);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexBindings");
            if (written != count)
            {
                throw new InvalidOperationException(
                    "The native render-binding count changed during a synchronous query.");
            }
            return bindings;
        }

        public void SetRenderTriangles(uint[] triangleVertexIds)
        {
            ThrowIfDisposed();
            if (triangleVertexIds == null)
            {
                throw new ArgumentNullException(nameof(triangleVertexIds));
            }
            if (triangleVertexIds.Length % 3 != 0)
            {
                throw new ArgumentException(
                    "Render triangle vertex IDs must be a flattened triangle array.",
                    nameof(triangleVertexIds));
            }

            ApxResult result = NativeMethods.ApxSetRenderTriangles(
                _handle,
                triangleVertexIds.Length == 0 ? null : triangleVertexIds,
                checked((uint)(triangleVertexIds.Length / 3)));
            ApxException.ThrowIfFailed(result, "apxSetRenderTriangles");
        }

        public ApxVec3[] GetRenderVertexPositions()
        {
            ThrowIfDisposed();
            uint count = GetRenderVertexPositionCount();
            if (count == 0)
            {
                return Array.Empty<ApxVec3>();
            }

            ApxVec3[] positions = new ApxVec3[checked((int)count)];
            int written = GetRenderVertexPositions(positions);
            if ((uint)written != count)
            {
                throw new InvalidOperationException(
                    "The native render-vertex count changed during a synchronous query.");
            }

            return positions;
        }

        public uint GetRenderVertexPositionCount()
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxGetRenderVertexPositions(
                _handle,
                IntPtr.Zero,
                0,
                out uint count);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexPositions");
            return count;
        }

        public int GetRenderVertexPositions(ApxVec3[] destination)
        {
            ThrowIfDisposed();
            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination));
            }

            GCHandle pinned = default;
            try
            {
                IntPtr pointer = IntPtr.Zero;
                if (destination.Length != 0)
                {
                    pinned = GCHandle.Alloc(destination, GCHandleType.Pinned);
                    pointer = pinned.AddrOfPinnedObject();
                }
                return GetRenderVertexPositions(pointer, destination.Length);
            }
            finally
            {
                if (pinned.IsAllocated)
                {
                    pinned.Free();
                }
            }
        }

        public int GetRenderVertexPositions(IntPtr destination, int capacity)
        {
            ThrowIfDisposed();
            ValidateNativeBuffer(destination, capacity, nameof(destination));
            ApxResult result = NativeMethods.ApxGetRenderVertexPositions(
                _handle,
                destination,
                checked((uint)capacity),
                out uint written);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexPositions");
            return checked((int)written);
        }

        public uint GetRenderVertexNormalCount()
        {
            ThrowIfDisposed();
            ApxResult result = NativeMethods.ApxGetRenderVertexNormals(
                _handle,
                IntPtr.Zero,
                0,
                out uint count);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexNormals");
            return count;
        }

        public int GetRenderVertexNormals(ApxVec3[] destination)
        {
            ThrowIfDisposed();
            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination));
            }

            GCHandle pinned = default;
            try
            {
                IntPtr pointer = IntPtr.Zero;
                if (destination.Length != 0)
                {
                    pinned = GCHandle.Alloc(destination, GCHandleType.Pinned);
                    pointer = pinned.AddrOfPinnedObject();
                }
                return GetRenderVertexNormals(pointer, destination.Length);
            }
            finally
            {
                if (pinned.IsAllocated)
                {
                    pinned.Free();
                }
            }
        }

        public int GetRenderVertexNormals(IntPtr destination, int capacity)
        {
            ThrowIfDisposed();
            ValidateNativeBuffer(destination, capacity, nameof(destination));
            ApxResult result = NativeMethods.ApxGetRenderVertexNormals(
                _handle,
                destination,
                checked((uint)capacity),
                out uint written);
            ApxException.ThrowIfFailed(result, "apxGetRenderVertexNormals");
            return checked((int)written);
        }

        public ApxCutResult Cut(ApxCutQuery query)
        {
            ThrowIfDisposed();
            if (!_cutCapacityPrepared)
            {
                EnsureReadbackCapacity(checked(_particleCount * 2U));
            }

            ApxResult result = NativeMethods.ApxCut(_handle, in query, out ApxCutResult cutResult);
            ApxException.ThrowIfFailed(result, "apxCut");
            _particleCount = checked(_particleCount + cutResult.SplitParticleCount);
            _cutCapacityPrepared = true;
            return cutResult;
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

        private static void ValidateNativeBuffer(
            IntPtr destination,
            int capacity,
            string parameterName)
        {
            if (capacity < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity));
            }
            if (capacity != 0 && destination == IntPtr.Zero)
            {
                throw new ArgumentNullException(parameterName);
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
