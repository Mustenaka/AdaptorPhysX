using System;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using NUnit.Framework;

namespace APEX.Native.Tests
{
    public sealed class NativeWorldEditModeTests
    {
        private const float FixedTimeStep = 1.0F / 120.0F;
        private const float InitialHeight = 2.0F;
        private const float ParticleRadius = 0.1F;

        [Test]
        public void AbiVersionMatchesAdditiveVersionZeroPointTwo()
        {
            NativeWorld.GetAbiVersion(out uint major, out uint minor);

            Assert.That(major, Is.EqualTo(0U));
            Assert.That(minor, Is.EqualTo(2U));
        }

        [Test]
        public void ManagedLayoutsMatchCAbiV0()
        {
            Assert.That(IntPtr.Size, Is.EqualTo(8), "P1-0 packages only Windows x64.");
            Assert.That(Marshal.SizeOf<ApxVec3>(), Is.EqualTo(12));
            Assert.That(Marshal.SizeOf<ApxVec4>(), Is.EqualTo(16));
            Assert.That(Marshal.SizeOf<ApxWorldDesc>(), Is.EqualTo(48));
            Assert.That(Marshal.SizeOf<ApxParticleDesc>(), Is.EqualTo(28));
            Assert.That(Marshal.SizeOf<ApxDistanceConstraintDesc>(), Is.EqualTo(16));
            Assert.That(Marshal.SizeOf<ApxClothDistanceConstraintDesc>(), Is.EqualTo(24));
            Assert.That(Marshal.SizeOf<ApxBendConstraintDesc>(), Is.EqualTo(24));
            Assert.That(Marshal.SizeOf<ApxRenderVertexBindingDesc>(), Is.EqualTo(24));
            Assert.That(Marshal.SizeOf<ApxColliderProxy>(), Is.EqualTo(48));
            Assert.That(Marshal.SizeOf<ApxBufferView>(), Is.EqualTo(24));

            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.StructSize), 0);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.Backend), 4);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.FixedTimeStep), 8);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.SolverIterations), 12);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.Gravity), 16);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.ParticleRadius), 28);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.InitialParticleCapacity), 32);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.Reserved0), 36);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.Reserved1), 40);
            AssertOffset<ApxWorldDesc>(nameof(ApxWorldDesc.Reserved2), 44);
            AssertOffset<ApxParticleDesc>(nameof(ApxParticleDesc.Position), 0);
            AssertOffset<ApxParticleDesc>(nameof(ApxParticleDesc.Velocity), 12);
            AssertOffset<ApxParticleDesc>(nameof(ApxParticleDesc.InverseMass), 24);
            AssertOffset<ApxDistanceConstraintDesc>(
                nameof(ApxDistanceConstraintDesc.ParticleA),
                0);
            AssertOffset<ApxDistanceConstraintDesc>(
                nameof(ApxDistanceConstraintDesc.ParticleB),
                4);
            AssertOffset<ApxDistanceConstraintDesc>(
                nameof(ApxDistanceConstraintDesc.RestLength),
                8);
            AssertOffset<ApxDistanceConstraintDesc>(
                nameof(ApxDistanceConstraintDesc.Compliance),
                12);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.ParticleA),
                0);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.ParticleB),
                4);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.RestLength),
                8);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.Compliance),
                12);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.BreakThreshold),
                16);
            AssertOffset<ApxClothDistanceConstraintDesc>(
                nameof(ApxClothDistanceConstraintDesc.Direction),
                20);
            AssertOffset<ApxBendConstraintDesc>(nameof(ApxBendConstraintDesc.OppositeA), 0);
            AssertOffset<ApxBendConstraintDesc>(nameof(ApxBendConstraintDesc.OppositeB), 4);
            AssertOffset<ApxBendConstraintDesc>(nameof(ApxBendConstraintDesc.EdgeA), 8);
            AssertOffset<ApxBendConstraintDesc>(nameof(ApxBendConstraintDesc.EdgeB), 12);
            AssertOffset<ApxBendConstraintDesc>(
                nameof(ApxBendConstraintDesc.SupportingClothConstraint),
                16);
            AssertOffset<ApxBendConstraintDesc>(nameof(ApxBendConstraintDesc.Compliance), 20);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.ParticleA),
                0);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.ParticleB),
                4);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.ParticleC),
                8);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.WeightA),
                12);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.WeightB),
                16);
            AssertOffset<ApxRenderVertexBindingDesc>(
                nameof(ApxRenderVertexBindingDesc.WeightC),
                20);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Type), 0);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Reserved0), 4);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Reserved1), 8);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Reserved2), 12);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Data0), 16);
            AssertOffset<ApxColliderProxy>(nameof(ApxColliderProxy.Data1), 32);
            AssertOffset<ApxBufferView>(nameof(ApxBufferView.Data), 0);
            AssertOffset<ApxBufferView>(nameof(ApxBufferView.SizeBytes), 8);
            AssertOffset<ApxBufferView>(nameof(ApxBufferView.ElementCount), 16);
            AssertOffset<ApxBufferView>(nameof(ApxBufferView.ElementStrideBytes), 20);
        }

        [Test]
        public void NativeMethodsExposeExactlyTheAdditiveFourteenCdeclEntrypoints()
        {
            Type nativeMethods = typeof(NativeWorld).Assembly.GetType(
                "APEX.Native.NativeMethods",
                true);
            MethodInfo[] imports = nativeMethods
                .GetMethods(BindingFlags.Static | BindingFlags.NonPublic)
                .Where(method => method.GetCustomAttribute<DllImportAttribute>() != null)
                .ToArray();

            string[] expected =
            {
                "apxGetAbiVersion",
                "apxCreateWorld",
                "apxDestroyWorld",
                "apxAddParticles",
                "apxAddDistanceConstraints",
                "apxAddClothDistanceConstraints",
                "apxAddBendConstraints",
                "apxGetBrokenClothDistanceConstraintIds",
                "apxSetRenderVertexBindings",
                "apxGetRenderVertexPositions",
                "apxSetColliderProxies",
                "apxStep",
                "apxMapParticleBuffer",
                "apxUnmapParticleBuffer",
            };

            Assert.That(imports, Has.Length.EqualTo(expected.Length));
            CollectionAssert.AreEquivalent(
                expected,
                imports.Select(method => method.GetCustomAttribute<DllImportAttribute>().EntryPoint));

            foreach (MethodInfo import in imports)
            {
                DllImportAttribute attribute = import.GetCustomAttribute<DllImportAttribute>();
                Assert.That(attribute.Value, Is.EqualTo("adaptorphysx"));
                Assert.That(attribute.CallingConvention, Is.EqualTo(CallingConvention.Cdecl));
                Assert.That(attribute.ExactSpelling, Is.True);
            }
        }

        [Test]
        public void CpuWorldFallsMaintainsConstraintAndDoesNotPenetrateGround()
        {
            using (NativeWorld world = CreateFallingPairWorld(ApxBackendKind.Cpu))
            {
                RunFallingPair(world);
            }
        }

        [Test]
        public void CudaWorldFallsMaintainsConstraintAndDoesNotPenetrateGround()
        {
            NativeWorld world;
            try
            {
                world = CreateFallingPairWorld(ApxBackendKind.Cuda);
            }
            catch (ApxException exception)
                when (exception.Result == ApxResult.Unsupported ||
                      exception.Result == ApxResult.BackendError)
            {
                Assert.Ignore($"CUDA backend is unavailable: {exception.Message}");
                return;
            }

            using (world)
            {
                // Once creation succeeds, any CUDA pipeline, launch, or
                // readback BackendError is a real regression and must fail.
                RunFallingPair(world);
            }
        }

        [TestCase(ApxBackendKind.Cpu)]
        [TestCase(ApxBackendKind.Cuda)]
        public void ClothAndBendBindingsStepSixtyFramesAndReportNoPrematureBreak(
            ApxBackendKind backend)
        {
            NativeWorld world;
            try
            {
                world = NativeWorld.Create(
                    ApxWorldDesc.Create(
                        backend,
                        FixedTimeStep,
                        8,
                        new ApxVec3(0.0F, -1.0F, 0.0F),
                        0.0F,
                        4));
            }
            catch (ApxException exception)
                when (backend == ApxBackendKind.Cuda &&
                      (exception.Result == ApxResult.Unsupported ||
                       exception.Result == ApxResult.BackendError))
            {
                Assert.Ignore($"CUDA backend is unavailable: {exception.Message}");
                return;
            }

            using (world)
            {
                ApxParticleDesc[] particles =
                {
                    new ApxParticleDesc(new ApxVec3(0.0F, 1.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(1.0F, 1.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(0.0F, 0.0F, 0.0F), default, 1.0F),
                    new ApxParticleDesc(new ApxVec3(1.0F, 0.0F, 0.0F), default, 1.0F),
                };
                ApxClothDistanceConstraintDesc[] cloth =
                {
                    new ApxClothDistanceConstraintDesc(
                        0, 1, 1.0F, 0.0F, 10.0F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        0, 2, 1.0F, 1.0e-6F, 10.0F, ApxClothDirection.Weft),
                    new ApxClothDistanceConstraintDesc(
                        1, 3, 1.0F, 1.0e-6F, 10.0F, ApxClothDirection.Weft),
                    new ApxClothDistanceConstraintDesc(
                        2, 3, 1.0F, 0.0F, 10.0F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        0, 3, (float)Math.Sqrt(2.0), 1.0e-6F, 10.0F, ApxClothDirection.Shear),
                };
                ApxBendConstraintDesc[] bend =
                {
                    new ApxBendConstraintDesc(
                        0,
                        3,
                        1,
                        2,
                        ApxBendConstraintDesc.NoSupportingClothConstraint,
                        1.0e-5F),
                };

                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                Assert.That(world.AddClothDistanceConstraints(cloth), Is.EqualTo(0U));
                Assert.That(world.AddBendConstraints(bend), Is.EqualTo(0U));

                for (int frame = 0; frame < 60; ++frame)
                {
                    world.Step(1.0F / 60.0F);
                }

                ApxVec3[] positions = new ApxVec3[particles.Length];
                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(particles.Length));
                foreach (ApxVec3 position in positions)
                {
                    AssertFinite(position);
                }

                CollectionAssert.IsEmpty(world.GetBrokenClothDistanceConstraintIds());
            }
        }

        [TestCase(ApxBackendKind.Cpu)]
        [TestCase(ApxBackendKind.Cuda)]
        public void TearQueryReportsLiteralPropagationAndCallerCapacity(ApxBackendKind backend)
        {
            NativeWorld world;
            try
            {
                world = NativeWorld.Create(
                    ApxWorldDesc.Create(
                        backend,
                        1.0F,
                        1,
                        default,
                        0.0F,
                        3));
            }
            catch (ApxException exception)
                when (backend == ApxBackendKind.Cuda &&
                      (exception.Result == ApxResult.Unsupported ||
                       exception.Result == ApxResult.BackendError))
            {
                Assert.Ignore($"CUDA backend is unavailable: {exception.Message}");
                return;
            }

            using (world)
            {
                ApxParticleDesc[] particles =
                {
                    new ApxParticleDesc(new ApxVec3(0.0F, 0.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(
                        new ApxVec3(1.0F, 0.0F, 0.0F),
                        new ApxVec3(0.5F, 0.0F, 0.0F),
                        1.0F),
                    new ApxParticleDesc(new ApxVec3(1.0F, 1.0F, 0.0F), default, 0.0F),
                };
                ApxClothDistanceConstraintDesc[] cloth =
                {
                    new ApxClothDistanceConstraintDesc(
                        0, 1, 1.0F, 1.0e6F, 1.05F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        1, 2, 1.0F, 1.0e6F, 1.28F, ApxClothDirection.Weft),
                };
                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                Assert.That(world.AddClothDistanceConstraints(cloth), Is.EqualTo(0U));

                world.Step(1.0F);
                Assert.That(world.GetBrokenClothDistanceConstraintCount(), Is.EqualTo(1U));
                uint[] ids = { 77U, 88U };
                Assert.That(world.GetBrokenClothDistanceConstraintIds(ids), Is.EqualTo(1));
                CollectionAssert.AreEqual(new uint[] { 0U, 88U }, ids);

                world.Step(1.0F);
                ids[0] = 77U;
                ids[1] = 88U;
                uint[] undersized = { 99U };
                ApxException capacity = Assert.Throws<ApxException>(
                    () => world.GetBrokenClothDistanceConstraintIds(undersized));
                Assert.That(capacity.Result, Is.EqualTo(ApxResult.CapacityExceeded));
                CollectionAssert.AreEqual(new uint[] { 99U }, undersized);
                Assert.That(world.GetBrokenClothDistanceConstraintCount(), Is.EqualTo(2U));
                CollectionAssert.AreEqual(
                    new uint[] { 0U, 1U },
                    world.GetBrokenClothDistanceConstraintIds());
                Assert.That(world.GetBrokenClothDistanceConstraintIds(ids), Is.EqualTo(2));
                CollectionAssert.AreEqual(new uint[] { 0U, 1U }, ids);
            }
        }

        [TestCase(ApxBackendKind.Cpu)]
        [TestCase(ApxBackendKind.Cuda)]
        public void DualGridMappingStepsSixtyFramesAndSupportsAllQueryShapes(
            ApxBackendKind backend)
        {
            NativeWorld world;
            try
            {
                world = NativeWorld.Create(
                    ApxWorldDesc.Create(
                        backend,
                        FixedTimeStep,
                        4,
                        default,
                        0.0F,
                        3));
            }
            catch (ApxException exception)
                when (backend == ApxBackendKind.Cuda &&
                      (exception.Result == ApxResult.Unsupported ||
                       exception.Result == ApxResult.BackendError))
            {
                Assert.Ignore($"CUDA backend is unavailable: {exception.Message}");
                return;
            }

            using (world)
            {
                ApxParticleDesc[] particles =
                {
                    new ApxParticleDesc(new ApxVec3(0.0F, 0.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(
                        new ApxVec3(2.0F, 0.0F, 0.0F),
                        new ApxVec3(1.0F, 0.0F, 0.0F),
                        1.0F),
                    new ApxParticleDesc(new ApxVec3(0.0F, 2.0F, 0.0F), default, 0.0F),
                };
                ApxRenderVertexBindingDesc[] bindings =
                {
                    new ApxRenderVertexBindingDesc(0, 1, 2, 2.0F, 3.0F, 5.0F),
                    new ApxRenderVertexBindingDesc(0, 1, 2, 1.0F, 1.0F, 0.0F),
                };
                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                world.SetRenderVertexBindings(bindings);

                Assert.That(world.GetRenderVertexPositionCount(), Is.EqualTo(2U));
                ApxVec3[] undersized = { new ApxVec3(77.0F, 88.0F, 99.0F) };
                ApxException capacity = Assert.Throws<ApxException>(
                    () => world.GetRenderVertexPositions(undersized));
                Assert.That(capacity.Result, Is.EqualTo(ApxResult.CapacityExceeded));
                Assert.That(undersized[0].X, Is.EqualTo(77.0F));
                Assert.That(undersized[0].Y, Is.EqualTo(88.0F));
                Assert.That(undersized[0].Z, Is.EqualTo(99.0F));

                ApxVec3[] callerOwned = new ApxVec3[2];
                Assert.That(world.GetRenderVertexPositions(callerOwned), Is.EqualTo(2));
                Assert.That(callerOwned[0].X, Is.EqualTo(0.6F).Within(1.0e-6F));
                Assert.That(callerOwned[0].Y, Is.EqualTo(1.0F).Within(1.0e-6F));

                for (int frame = 0; frame < 60; ++frame)
                {
                    world.Step(1.0F / 60.0F);
                }

                ApxVec3[] particlePositions = new ApxVec3[3];
                Assert.That(world.ReadPositionSnapshot(particlePositions), Is.EqualTo(3));
                ApxVec3[] allocated = world.GetRenderVertexPositions();
                Assert.That(allocated, Has.Length.EqualTo(2));
                Assert.That(world.GetRenderVertexPositionCount(), Is.EqualTo(2U));
                AssertMappedPosition(
                    allocated[0],
                    particlePositions,
                    bindings[0],
                    1.0e-5F);
                AssertMappedPosition(
                    allocated[1],
                    particlePositions,
                    bindings[1],
                    1.0e-5F);
            }
        }

        [Test]
        public void StaleMappedSnapshotCannotReadOrUnmapANewerGeneration()
        {
            using (NativeWorld world = CreateFallingPairWorld(ApxBackendKind.Cpu))
            {
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(
                            new ApxVec3(0.0F, InitialHeight, 0.0F),
                            default,
                            1.0F),
                    });

                MappedPositionSnapshot stale = world.MapPositions();
                world.UnmapPositions();
                MappedPositionSnapshot current = world.MapPositions();
                try
                {
                    bool staleReadRejected = false;
                    try
                    {
                        _ = stale[0];
                    }
                    catch (ObjectDisposedException)
                    {
                        staleReadRejected = true;
                    }

                    Assert.That(staleReadRejected, Is.True);
                    stale.Dispose();

                    // Disposing the stale generation must not unmap this one.
                    AssertFinite(current[0]);
                }
                finally
                {
                    current.Dispose();
                    stale.Dispose();
                }
            }
        }

        private static NativeWorld CreateFallingPairWorld(ApxBackendKind backend)
        {
            ApxWorldDesc description = ApxWorldDesc.Create(
                backend,
                FixedTimeStep,
                8,
                new ApxVec3(0.0F, -9.81F, 0.0F),
                ParticleRadius,
                2);

            return NativeWorld.Create(description);
        }

        private static void RunFallingPair(NativeWorld world)
        {
            ApxParticleDesc[] particles =
            {
                new ApxParticleDesc(
                    new ApxVec3(-0.5F, InitialHeight, 0.0F),
                    default,
                    1.0F),
                new ApxParticleDesc(
                    new ApxVec3(0.5F, InitialHeight, 0.0F),
                    default,
                    1.0F),
            };
            ApxDistanceConstraintDesc[] constraints =
            {
                new ApxDistanceConstraintDesc(0, 1, 1.0F, 0.0F),
            };
            ApxColliderProxy[] proxies =
            {
                new ApxColliderProxy
                {
                    Type = ApxColliderProxyType.Plane,
                    Data0 = new ApxVec4(0.0F, 1.0F, 0.0F, 0.0F),
                },
            };

            Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
            Assert.That(world.AddDistanceConstraints(constraints), Is.EqualTo(0U));
            world.SetColliderProxies(proxies);

            for (int frame = 0; frame < 60; ++frame)
            {
                world.Step(1.0F / 60.0F);
            }

            ApxVec3[] copiedPositions = new ApxVec3[2];
            using (MappedPositionSnapshot mapped = world.MapPositions())
            {
                Assert.That(mapped.Count, Is.EqualTo(2));
                mapped.CopyTo(copiedPositions);
            }

            Assert.That(world.ReadPositionSnapshot(copiedPositions), Is.EqualTo(2));
            ApxVec3 first = copiedPositions[0];
            ApxVec3 second = copiedPositions[1];

            AssertFinite(first);
            AssertFinite(second);
            Assert.That(first.Y, Is.LessThan(InitialHeight));
            Assert.That(second.Y, Is.LessThan(InitialHeight));
            Assert.That(first.Y, Is.GreaterThanOrEqualTo(ParticleRadius - 1.0e-3F));
            Assert.That(second.Y, Is.GreaterThanOrEqualTo(ParticleRadius - 1.0e-3F));

            float deltaX = first.X - second.X;
            float deltaY = first.Y - second.Y;
            float deltaZ = first.Z - second.Z;
            float distanceSquared =
                deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
            Assert.That(Math.Abs(distanceSquared - 1.0F), Is.LessThan(1.0e-3F));
        }

        private static void AssertOffset<T>(string fieldName, int expected)
        {
            Assert.That(Marshal.OffsetOf<T>(fieldName).ToInt32(), Is.EqualTo(expected));
        }

        private static void AssertFinite(ApxVec3 value)
        {
            Assert.That(float.IsNaN(value.X) || float.IsInfinity(value.X), Is.False);
            Assert.That(float.IsNaN(value.Y) || float.IsInfinity(value.Y), Is.False);
            Assert.That(float.IsNaN(value.Z) || float.IsInfinity(value.Z), Is.False);
        }

        private static void AssertMappedPosition(
            ApxVec3 actual,
            ApxVec3[] particles,
            ApxRenderVertexBindingDesc binding,
            float tolerance)
        {
            float sum = (binding.WeightA + binding.WeightB) + binding.WeightC;
            float weightA = binding.WeightA / sum;
            float weightB = binding.WeightB / sum;
            float weightC = binding.WeightC / sum;
            ApxVec3 expected = new ApxVec3(
                particles[binding.ParticleA].X * weightA +
                    particles[binding.ParticleB].X * weightB +
                    particles[binding.ParticleC].X * weightC,
                particles[binding.ParticleA].Y * weightA +
                    particles[binding.ParticleB].Y * weightB +
                    particles[binding.ParticleC].Y * weightC,
                particles[binding.ParticleA].Z * weightA +
                    particles[binding.ParticleB].Z * weightB +
                    particles[binding.ParticleC].Z * weightC);
            Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance));
            Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance));
            Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance));
        }
    }
}
