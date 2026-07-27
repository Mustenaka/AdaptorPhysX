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
        public void AbiVersionMatchesAdditiveVersionZeroPointThree()
        {
            NativeWorld.GetAbiVersion(out uint major, out uint minor);

            Assert.That(major, Is.EqualTo(0U));
            Assert.That(minor, Is.EqualTo(3U));
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
            Assert.That(Marshal.SizeOf<ApxCutQuery>(), Is.EqualTo(40));
            Assert.That(Marshal.SizeOf<ApxCutResult>(), Is.EqualTo(16));
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
            AssertOffset<ApxCutQuery>(nameof(ApxCutQuery.Start), 0);
            AssertOffset<ApxCutQuery>(nameof(ApxCutQuery.End), 12);
            AssertOffset<ApxCutQuery>(nameof(ApxCutQuery.SideNormal), 24);
            AssertOffset<ApxCutQuery>(nameof(ApxCutQuery.Radius), 36);
            AssertOffset<ApxCutResult>(nameof(ApxCutResult.CutId), 0);
            AssertOffset<ApxCutResult>(nameof(ApxCutResult.CutConstraintCount), 4);
            AssertOffset<ApxCutResult>(nameof(ApxCutResult.SplitParticleCount), 8);
            AssertOffset<ApxCutResult>(nameof(ApxCutResult.FirstSplitParticleId), 12);
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
        public void NativeMethodsExposeExactlyTheAdditiveFifteenCdeclEntrypoints()
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
                "apxCut",
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
        public void CurrentDegenerateBendStencilRemainsFiniteAndAppliesProjection(
            ApxBackendKind backend)
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
                    new ApxParticleDesc(new ApxVec3(0.0F, 0.0F, 0.0F), default, 1.0F),
                    new ApxParticleDesc(new ApxVec3(1.0F, 0.0F, 0.0F), default, 1.0F),
                    new ApxParticleDesc(
                        new ApxVec3(0.0F, 1.0F, 0.0F),
                        new ApxVec3(0.5F, -1.0F, 0.0F),
                        1.0F),
                    new ApxParticleDesc(
                        new ApxVec3(1.0F, -1.0F, 0.0F),
                        new ApxVec3(0.0F, 0.0F, 1.0F),
                        1.0F),
                };
                ApxBendConstraintDesc[] bend =
                {
                    new ApxBendConstraintDesc(
                        2,
                        3,
                        0,
                        1,
                        ApxBendConstraintDesc.NoSupportingClothConstraint,
                        0.0F),
                };

                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                Assert.That(world.AddBendConstraints(bend), Is.EqualTo(0U));
                world.Step(1.0F);

                ApxVec3[] positions = new ApxVec3[4];
                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(positions.Length));
                foreach (ApxVec3 position in positions)
                {
                    AssertFinite(position);
                }
                Assert.That(positions[2].Y, Is.Not.EqualTo(0.0F));
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
                CollectionAssert.AreEqual(
                    new uint[] { 0U },
                    world.GetBrokenClothDistanceConstraintIds());
                CollectionAssert.AreEqual(
                    new uint[] { 0U },
                    world.GetBrokenClothDistanceConstraintIds());

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

        [TestCase(ApxBackendKind.Cpu)]
        [TestCase(ApxBackendKind.Cuda)]
        public void CutQuerySplitsStableParticleAndBothSidesEvolveIndependently(
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
                        5));
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
                    new ApxParticleDesc(
                        new ApxVec3(0.0F, 0.0F, 0.0F),
                        new ApxVec3(0.125F, 0.0F, 0.0F),
                        1.0F),
                    new ApxParticleDesc(
                        new ApxVec3(1.0F, 0.0F, 0.0F),
                        new ApxVec3(0.25F, 0.0F, 0.0F),
                        1.0F),
                    new ApxParticleDesc(new ApxVec3(-1.0F, 0.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(0.0F, 1.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(0.0F, -1.0F, 0.0F), default, 0.0F),
                };
                ApxClothDistanceConstraintDesc[] cloth =
                {
                    new ApxClothDistanceConstraintDesc(
                        0, 1, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        0, 2, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        0, 3, 1.0F, 0.0F, 100.0F, ApxClothDirection.Weft),
                    new ApxClothDistanceConstraintDesc(
                        0, 4, 1.0F, 0.0F, 100.0F, ApxClothDirection.Weft),
                };
                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                Assert.That(world.AddClothDistanceConstraints(cloth), Is.EqualTo(0U));

                world.Step(FixedTimeStep * 0.5F);
                ApxCutResult result = world.Cut(
                    new ApxCutQuery(
                        new ApxVec3(0.0F, -2.0F, 0.0F),
                        new ApxVec3(0.0F, 2.0F, 0.0F),
                        new ApxVec3(2.0F, 0.0F, 0.0F),
                        0.1F));
                Assert.That(result.CutId, Is.EqualTo(0U));
                Assert.That(result.CutConstraintCount, Is.EqualTo(2U));
                Assert.That(result.SplitParticleCount, Is.EqualTo(1U));
                Assert.That(result.FirstSplitParticleId, Is.EqualTo(5U));
                Assert.That(world.ParticleCount, Is.EqualTo(6U));
                CollectionAssert.AreEqual(
                    new uint[] { 2U, 3U },
                    world.GetBrokenClothDistanceConstraintIds());

                for (int frame = 0; frame < 60; ++frame)
                {
                    world.Step(1.0F / 60.0F);
                }

                ApxVec3[] positions = new ApxVec3[6];
                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(6));
                AssertFinite(positions[0]);
                AssertFinite(positions[5]);
                float deltaX = positions[0].X - positions[5].X;
                float deltaY = positions[0].Y - positions[5].Y;
                float deltaZ = positions[0].Z - positions[5].Z;
                Assert.That(
                    deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ,
                    Is.GreaterThan(1.0e-8F));
            }
        }

        [TestCase(ApxBackendKind.Cpu)]
        [TestCase(ApxBackendKind.Cuda)]
        public void RuntimeSplitWithSelfCollisionSeparatesSeamForSixtyFrames(
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
                        0.1F,
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
                    new ApxParticleDesc(new ApxVec3(0.0F, 0.0F, 0.0F), default, 1.0F),
                    new ApxParticleDesc(new ApxVec3(1.0F, 0.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(-1.0F, 0.0F, 0.0F), default, 0.0F),
                };
                ApxClothDistanceConstraintDesc[] cloth =
                {
                    new ApxClothDistanceConstraintDesc(
                        0, 1, 1.0F, 1.0e-5F, 100.0F, ApxClothDirection.Warp),
                    new ApxClothDistanceConstraintDesc(
                        0, 2, 1.0F, 1.0e-5F, 100.0F, ApxClothDirection.Warp),
                };
                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                Assert.That(world.AddClothDistanceConstraints(cloth), Is.EqualTo(0U));

                world.Step(FixedTimeStep * 0.5F);
                ApxCutResult result = world.Cut(
                    new ApxCutQuery(
                        new ApxVec3(0.0F, -0.5F, 0.0F),
                        new ApxVec3(0.0F, 0.5F, 0.0F),
                        new ApxVec3(1.0F, 0.0F, 0.0F),
                        0.01F));
                Assert.That(result.CutConstraintCount, Is.EqualTo(0U));
                Assert.That(result.SplitParticleCount, Is.EqualTo(1U));
                Assert.That(result.FirstSplitParticleId, Is.EqualTo(3U));
                Assert.That(world.ParticleCount, Is.EqualTo(4U));

                world.Step(FixedTimeStep);
                ApxVec3[] positions = new ApxVec3[4];
                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(4));
                Assert.That(SeparationSquared(positions[0], positions[3]), Is.GreaterThan(0.0225F));
                for (int frame = 1; frame < 60; ++frame)
                {
                    world.Step(FixedTimeStep);
                }

                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(4));
                AssertFinite(positions[0]);
                AssertFinite(positions[3]);
                Assert.That(SeparationSquared(positions[0], positions[3]), Is.GreaterThan(0.0225F));
            }
        }

        [Test]
        public void RenderMeshCutterCreatesStableInteriorSeamAndInterpolatesAttributes()
        {
            RenderMeshData input = CreateSquareRenderMesh();
            RenderMeshCutResult result = RenderMeshCutter.Cut(input, CreateSquareCutQuery());

            Assert.That(result.CutTriangleCount, Is.EqualTo(2U));
            Assert.That(result.CreatedSeamPairCount, Is.EqualTo(3U));
            Assert.That(result.Mesh.Vertices, Has.Length.EqualTo(10));
            Assert.That(result.Mesh.Indices, Has.Length.EqualTo(18));
            for (int pairIndex = 0; pairIndex < result.SeamPairs.Length; ++pairIndex)
            {
                RenderMeshSeamPair pair = result.SeamPairs[pairIndex];
                Assert.That(pair.NegativeVertexId, Is.EqualTo(4U + (uint)pairIndex * 2U));
                Assert.That(pair.PositiveVertexId, Is.EqualTo(pair.NegativeVertexId + 1U));
                AssertRenderVertexBitsEqual(
                    result.Mesh.Vertices[pair.NegativeVertexId],
                    result.Mesh.Vertices[pair.PositiveVertexId]);
                RenderMeshVertex seam = result.Mesh.Vertices[pair.NegativeVertexId];
                Assert.That(seam.Position.X, Is.EqualTo(0.5F));
                Assert.That(seam.Normal.X, Is.EqualTo(0.0F));
                Assert.That(seam.Normal.Y, Is.EqualTo(0.0F));
                Assert.That(seam.Normal.Z, Is.EqualTo(1.0F));
                Assert.That(seam.U, Is.EqualTo(0.5F));
            }

            Assert.That(result.Mesh.Vertices[4].V, Is.EqualTo(0.0F));
            Assert.That(result.Mesh.Vertices[6].V, Is.EqualTo(0.5F));
            Assert.That(result.Mesh.Vertices[8].V, Is.EqualTo(1.0F));
        }

        [Test]
        public void RenderMeshCutterRejectsDegenerateInputAndFiniteSegmentAvoidsExtensionHit()
        {
            RenderMeshData input = CreateSquareRenderMesh();
            RenderMeshVertex[] verticesBefore = (RenderMeshVertex[])input.Vertices.Clone();
            uint[] indicesBefore = (uint[])input.Indices.Clone();
            ApxCutQuery farSegment = new ApxCutQuery(
                new ApxVec3(0.5F, 10.0F, 0.0F),
                new ApxVec3(0.5F, 11.0F, 0.0F),
                new ApxVec3(1.0F, 0.0F, 0.0F),
                0.1F);

            RenderMeshCutResult noHit = RenderMeshCutter.Cut(input, farSegment);
            Assert.That(noHit.CutTriangleCount, Is.EqualTo(0U));
            Assert.That(noHit.CreatedSeamPairCount, Is.EqualTo(0U));
            CollectionAssert.AreEqual(indicesBefore, noHit.Mesh.Indices);

            ApxCutQuery degenerate = new ApxCutQuery(
                default,
                default,
                new ApxVec3(1.0F, 0.0F, 0.0F),
                0.0F);
            Assert.Throws<ArgumentException>(() => RenderMeshCutter.Cut(input, degenerate));
            Assert.Throws<ArgumentException>(
                () => RenderMeshCutter.Cut(
                    new RenderMeshData(input.Vertices, new uint[] { 0U, 1U, 99U }),
                    CreateSquareCutQuery()));
            CollectionAssert.AreEqual(indicesBefore, input.Indices);
            for (int index = 0; index < verticesBefore.Length; ++index)
            {
                AssertRenderVertexBitsEqual(verticesBefore[index], input.Vertices[index]);
            }

            Assert.That(
                RenderMeshCutter.TryIntersectSegmentPlane(
                    new ApxVec3(-1.0F, 0.0F, 0.0F),
                    new ApxVec3(1.0F, 0.0F, 0.0F),
                    default,
                    new ApxVec3(1.0F, 0.0F, 0.0F),
                    out float parameter,
                    out ApxVec3 point),
                Is.True);
            Assert.That(parameter, Is.EqualTo(0.5F));
            Assert.That(point.X, Is.EqualTo(0.0F));
        }

        [Test]
        public void RenderMeshCutterDeterminismMatchesCompleteHashAcrossSixtySegmentCommits()
        {
            ApxCutQuery[] trajectory = new ApxCutQuery[60];
            for (int segment = 0; segment < trajectory.Length; ++segment)
            {
                trajectory[segment] = CreateSquareCutQuery();
            }

            RenderMeshCutResult first =
                RenderMeshCutter.CutPolyline(CreateSquareRenderMesh(), trajectory);
            RenderMeshCutResult second =
                RenderMeshCutter.CutPolyline(CreateSquareRenderMesh(), trajectory);
            Assert.That(HashRenderMeshCutResult(first), Is.EqualTo(HashRenderMeshCutResult(second)));
            Assert.That(first.CutTriangleCount, Is.EqualTo(2U));
            Assert.That(first.CreatedSeamPairCount, Is.EqualTo(3U));
        }

        [Test]
        public void RenderMeshCutterCrosscheckMatchesAnalyticSquareTopologyExactly()
        {
            RenderMeshData input = CreateSquareRenderMesh();
            ApxCutQuery query = CreateSquareCutQuery();
            foreach (RenderMeshVertex vertex in input.Vertices)
            {
                Assert.That(
                    Math.Abs(vertex.Position.X - query.Start.X),
                    Is.EqualTo(0.5F),
                    "Fixture vertices must stay away from the side-classification plane.");
            }
            Assert.That(
                query.Radius * query.Radius,
                Is.GreaterThan(0.0024F),
                "The exact trajectory overlap must stay away from the radius boundary.");

            RenderMeshCutResult result = RenderMeshCutter.Cut(input, query);
            CollectionAssert.AreEqual(
                new uint[]
                {
                    0U, 4U, 6U,
                    5U, 1U, 2U,
                    5U, 2U, 7U,
                    0U, 6U, 8U,
                    0U, 8U, 3U,
                    7U, 2U, 9U,
                },
                result.Mesh.Indices);
            Assert.That(result.SeamPairs[0].NegativeVertexId, Is.EqualTo(4U));
            Assert.That(result.SeamPairs[1].NegativeVertexId, Is.EqualTo(6U));
            Assert.That(result.SeamPairs[2].NegativeVertexId, Is.EqualTo(8U));
        }

        [Test]
        public void RenderMeshCutterPerfMeetsFixedHundredThousandTriangleGate()
        {
            RenderMeshData input = CreateRenderGrid(251, 201);
            Assert.That(input.Vertices, Has.Length.EqualTo(50451));
            Assert.That(input.Indices, Has.Length.EqualTo(300000));
            ApxCutQuery query = new ApxCutQuery(
                new ApxVec3(125.5F, -1.0F, 0.0F),
                new ApxVec3(125.5F, 201.0F, 0.0F),
                new ApxVec3(1.0F, 0.0F, 0.0F),
                0.01F);
            RenderMeshCutResult warmup = RenderMeshCutter.Cut(input, query);
            Assert.That(warmup.CutTriangleCount, Is.EqualTo(400U));

            double[] samples = new double[5];
            for (int sample = 0; sample < samples.Length; ++sample)
            {
                GC.Collect();
                GC.WaitForPendingFinalizers();
                System.Diagnostics.Stopwatch stopwatch = System.Diagnostics.Stopwatch.StartNew();
                RenderMeshCutResult result = RenderMeshCutter.Cut(input, query);
                stopwatch.Stop();
                Assert.That(result.CutTriangleCount, Is.EqualTo(400U));
                Assert.That(result.CreatedSeamPairCount, Is.EqualTo(401U));
                samples[sample] = stopwatch.Elapsed.TotalMilliseconds;
            }

            Array.Sort(samples);
            double medianMilliseconds = samples[samples.Length / 2];
            TestContext.Out.WriteLine(
                "APX_RENDER_CUT_PERF triangles=100000 samples=5 median_ms={0:F4} gate_ms=1500.0000",
                medianMilliseconds);
            Assert.That(medianMilliseconds, Is.LessThanOrEqualTo(1500.0));
        }

        [Test]
        public void BladeInteractionBuffersEnforceSamplingCapacityTicksAndGrabLifecycle()
        {
            BladeTrajectoryBuffer trajectory = new BladeTrajectoryBuffer(1.0F, 0.25F, 1);
            ApxVec3 normal = new ApxVec3(2.0F, 0.0F, 0.0F);
            trajectory.BeginStroke(default, normal, 10);
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(0.5F, 0.0F, 0.0F),
                    normal,
                    11,
                    out _),
                Is.EqualTo(BladeAppendResult.BelowMinimumDistance));
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(1.0F, 0.0F, 0.0F),
                    normal,
                    12,
                    out BladeTrajectorySegment accepted),
                Is.EqualTo(BladeAppendResult.Accepted));
            Assert.That(accepted.SegmentId, Is.EqualTo(0U));
            Assert.That(accepted.Query.Start.X, Is.EqualTo(0.0F));
            Assert.That(accepted.Query.End.X, Is.EqualTo(1.0F));
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(2.0F, 0.0F, 0.0F),
                    normal,
                    13,
                    out _),
                Is.EqualTo(BladeAppendResult.CapacityExceeded));
            Assert.That(trajectory.NextSegmentId, Is.EqualTo(1U));
            Assert.That(trajectory.Drain(), Has.Length.EqualTo(1));
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(2.0F, 0.0F, 0.0F),
                    normal,
                    13,
                    out accepted),
                Is.EqualTo(BladeAppendResult.Accepted));
            Assert.That(accepted.Query.Start.X, Is.EqualTo(1.0F));
            trajectory.EndStroke(14);
            Assert.Throws<InvalidOperationException>(() => trajectory.EndStroke(15));

            GrabCommandBuffer grabs = new GrabCommandBuffer(2);
            Assert.That(grabs.Begin(7U, new ApxVec3(1.0F, 2.0F, 3.0F), 20), Is.True);
            Assert.That(grabs.Move(new ApxVec3(2.0F, 3.0F, 4.0F), 21), Is.True);
            Assert.That(grabs.End(new ApxVec3(3.0F, 4.0F, 5.0F), 22), Is.False);
            Assert.That(grabs.IsGrabActive, Is.True);
            Assert.That(grabs.Drain(), Has.Length.EqualTo(2));
            Assert.That(grabs.End(new ApxVec3(3.0F, 4.0F, 5.0F), 22), Is.True);
            Assert.That(grabs.IsGrabActive, Is.False);
            Assert.Throws<InvalidOperationException>(
                () => grabs.Move(new ApxVec3(4.0F, 5.0F, 6.0F), 23));
        }

        [Test]
        public void BladeInteractionDeterminismMatchesCompleteSixtyTickCommandHash()
        {
            ulong first = RunBladeInteractionHash();
            ulong second = RunBladeInteractionHash();
            Assert.That(first, Is.EqualTo(second));
        }

        [Test]
        public void BladeInteractionCrosscheckPreservesExactFifoIdsTicksAndFields()
        {
            BladeTrajectoryBuffer trajectory = new BladeTrajectoryBuffer(0.5F, 0.125F, 4);
            ApxVec3 normal = new ApxVec3(2.0F, 0.0F, 0.0F);
            trajectory.BeginStroke(new ApxVec3(0.0F, 1.0F, 2.0F), normal, 10);
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(0.25F, 1.0F, 2.0F),
                    normal,
                    11,
                    out _),
                Is.EqualTo(BladeAppendResult.BelowMinimumDistance));
            Assert.That(
                trajectory.AppendSample(
                    new ApxVec3(1.0F, 1.0F, 2.0F),
                    normal,
                    12,
                    out _),
                Is.EqualTo(BladeAppendResult.Accepted));
            BladeTrajectorySegment[] segments = trajectory.Drain();
            Assert.That(segments, Has.Length.EqualTo(1));
            Assert.That(segments[0].SegmentId, Is.EqualTo(0U));
            Assert.That(segments[0].FixedTick, Is.EqualTo(12UL));
            Assert.That(segments[0].Query.Start.X, Is.EqualTo(0.0F));
            Assert.That(segments[0].Query.End.X, Is.EqualTo(1.0F));
            Assert.That(segments[0].Query.SideNormal.X, Is.EqualTo(2.0F));
            Assert.That(segments[0].Query.Radius, Is.EqualTo(0.125F));

            GrabCommandBuffer grabs = new GrabCommandBuffer(3);
            Assert.That(grabs.Begin(7U, new ApxVec3(1.0F, 2.0F, 3.0F), 20), Is.True);
            Assert.That(grabs.Move(new ApxVec3(2.0F, 3.0F, 4.0F), 21), Is.True);
            Assert.That(grabs.End(new ApxVec3(3.0F, 4.0F, 5.0F), 22), Is.True);
            GrabCommand[] commands = grabs.Drain();
            Assert.That(commands, Has.Length.EqualTo(3));
            for (int index = 0; index < commands.Length; ++index)
            {
                Assert.That(commands[index].SequenceId, Is.EqualTo((uint)index));
                Assert.That(commands[index].FixedTick, Is.EqualTo((ulong)(20 + index)));
                Assert.That(commands[index].ParticleId, Is.EqualTo(7U));
                Assert.That(commands[index].Phase, Is.EqualTo((GrabCommandPhase)index));
                Assert.That(commands[index].Position.X, Is.EqualTo(1.0F + index));
            }
        }

        [Test]
        public void BladeInteractionPerfMeetsHundredThousandSampleResponseGate()
        {
            const int sampleCount = 100000;
            double[] samples = new double[5];
            for (int window = 0; window < samples.Length; ++window)
            {
                GC.Collect();
                GC.WaitForPendingFinalizers();
                System.Diagnostics.Stopwatch stopwatch = System.Diagnostics.Stopwatch.StartNew();
                BladeTrajectoryBuffer trajectory =
                    new BladeTrajectoryBuffer(0.0F, 0.001F, sampleCount);
                ApxVec3 normal = new ApxVec3(0.0F, 1.0F, 0.0F);
                trajectory.BeginStroke(default, normal, 0);
                for (int sample = 1; sample <= sampleCount; ++sample)
                {
                    BladeAppendResult append = trajectory.AppendSample(
                        new ApxVec3(sample, 0.0F, 0.0F),
                        normal,
                        (ulong)sample,
                        out _);
                    if (append != BladeAppendResult.Accepted)
                    {
                        throw new InvalidOperationException(
                            $"Unexpected append result at sample {sample}: {append}");
                    }
                }
                BladeTrajectorySegment[] drained = trajectory.Drain();
                stopwatch.Stop();
                Assert.That(drained, Has.Length.EqualTo(sampleCount));
                Assert.That(drained[sampleCount - 1].SegmentId, Is.EqualTo(99999U));
                samples[window] = stopwatch.Elapsed.TotalMilliseconds;
            }

            Array.Sort(samples);
            double medianMilliseconds = samples[samples.Length / 2];
            TestContext.Out.WriteLine(
                "APX_BLADE_INPUT_PERF samples=100000 windows=5 median_ms={0:F4} gate_ms=250.0000",
                medianMilliseconds);
            Assert.That(medianMilliseconds, Is.LessThanOrEqualTo(250.0));
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

        private static float SeparationSquared(ApxVec3 left, ApxVec3 right)
        {
            float deltaX = left.X - right.X;
            float deltaY = left.Y - right.Y;
            float deltaZ = left.Z - right.Z;
            return deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
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

        private static RenderMeshData CreateSquareRenderMesh()
        {
            ApxVec3 normal = new ApxVec3(0.0F, 0.0F, 1.0F);
            return new RenderMeshData(
                new[]
                {
                    new RenderMeshVertex(new ApxVec3(0.0F, 0.0F, 0.0F), normal, 0.0F, 0.0F),
                    new RenderMeshVertex(new ApxVec3(1.0F, 0.0F, 0.0F), normal, 1.0F, 0.0F),
                    new RenderMeshVertex(new ApxVec3(1.0F, 1.0F, 0.0F), normal, 1.0F, 1.0F),
                    new RenderMeshVertex(new ApxVec3(0.0F, 1.0F, 0.0F), normal, 0.0F, 1.0F),
                },
                new uint[] { 0U, 1U, 2U, 0U, 2U, 3U });
        }

        private static ApxCutQuery CreateSquareCutQuery()
        {
            return new ApxCutQuery(
                new ApxVec3(0.5F, -1.0F, 0.0F),
                new ApxVec3(0.5F, 2.0F, 0.0F),
                new ApxVec3(1.0F, 0.0F, 0.0F),
                0.05F);
        }

        private static RenderMeshData CreateRenderGrid(int columns, int rows)
        {
            RenderMeshVertex[] vertices = new RenderMeshVertex[checked(columns * rows)];
            ApxVec3 normal = new ApxVec3(0.0F, 0.0F, 1.0F);
            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    vertices[row * columns + column] = new RenderMeshVertex(
                        new ApxVec3(column, row, 0.0F),
                        normal,
                        column / (float)(columns - 1),
                        row / (float)(rows - 1));
                }
            }

            uint[] indices = new uint[checked((columns - 1) * (rows - 1) * 6)];
            int offset = 0;
            for (int row = 0; row + 1 < rows; ++row)
            {
                for (int column = 0; column + 1 < columns; ++column)
                {
                    uint lowerLeft = checked((uint)(row * columns + column));
                    uint lowerRight = lowerLeft + 1U;
                    uint upperLeft = lowerLeft + checked((uint)columns);
                    uint upperRight = upperLeft + 1U;
                    indices[offset++] = lowerLeft;
                    indices[offset++] = lowerRight;
                    indices[offset++] = upperRight;
                    indices[offset++] = lowerLeft;
                    indices[offset++] = upperRight;
                    indices[offset++] = upperLeft;
                }
            }

            return new RenderMeshData(vertices, indices);
        }

        private static void AssertRenderVertexBitsEqual(
            RenderMeshVertex expected,
            RenderMeshVertex actual)
        {
            Assert.That(FloatBits(actual.Position.X), Is.EqualTo(FloatBits(expected.Position.X)));
            Assert.That(FloatBits(actual.Position.Y), Is.EqualTo(FloatBits(expected.Position.Y)));
            Assert.That(FloatBits(actual.Position.Z), Is.EqualTo(FloatBits(expected.Position.Z)));
            Assert.That(FloatBits(actual.Normal.X), Is.EqualTo(FloatBits(expected.Normal.X)));
            Assert.That(FloatBits(actual.Normal.Y), Is.EqualTo(FloatBits(expected.Normal.Y)));
            Assert.That(FloatBits(actual.Normal.Z), Is.EqualTo(FloatBits(expected.Normal.Z)));
            Assert.That(FloatBits(actual.U), Is.EqualTo(FloatBits(expected.U)));
            Assert.That(FloatBits(actual.V), Is.EqualTo(FloatBits(expected.V)));
        }

        private static ulong HashRenderMeshCutResult(RenderMeshCutResult result)
        {
            const ulong offsetBasis = 1469598103934665603UL;
            const ulong prime = 1099511628211UL;
            ulong hash = offsetBasis;
            foreach (RenderMeshVertex vertex in result.Mesh.Vertices)
            {
                hash = (hash ^ FloatBits(vertex.Position.X)) * prime;
                hash = (hash ^ FloatBits(vertex.Position.Y)) * prime;
                hash = (hash ^ FloatBits(vertex.Position.Z)) * prime;
                hash = (hash ^ FloatBits(vertex.Normal.X)) * prime;
                hash = (hash ^ FloatBits(vertex.Normal.Y)) * prime;
                hash = (hash ^ FloatBits(vertex.Normal.Z)) * prime;
                hash = (hash ^ FloatBits(vertex.U)) * prime;
                hash = (hash ^ FloatBits(vertex.V)) * prime;
            }
            foreach (uint index in result.Mesh.Indices)
            {
                hash = (hash ^ index) * prime;
            }
            foreach (RenderMeshSeamPair seam in result.SeamPairs)
            {
                hash = (hash ^ seam.NegativeVertexId) * prime;
                hash = (hash ^ seam.PositiveVertexId) * prime;
            }
            hash = (hash ^ result.CutTriangleCount) * prime;
            return hash;
        }

        private static uint FloatBits(float value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            return BitConverter.ToUInt32(bytes, 0);
        }

        private static ulong RunBladeInteractionHash()
        {
            const ulong offsetBasis = 1469598103934665603UL;
            const ulong prime = 1099511628211UL;
            BladeTrajectoryBuffer trajectory = new BladeTrajectoryBuffer(0.0F, 0.01F, 60);
            GrabCommandBuffer grabs = new GrabCommandBuffer(61);
            ApxVec3 normal = new ApxVec3(0.0F, 2.0F, 0.0F);
            trajectory.BeginStroke(default, normal, 0);
            Assert.That(grabs.Begin(17U, default, 0), Is.True);
            for (ulong tick = 1; tick <= 60; ++tick)
            {
                ApxVec3 position =
                    new ApxVec3((float)tick * 0.125F, (float)tick * 0.03125F, 0.0F);
                Assert.That(
                    trajectory.AppendSample(position, normal, tick, out _),
                    Is.EqualTo(BladeAppendResult.Accepted));
                if (tick < 60)
                {
                    Assert.That(grabs.Move(position, tick), Is.True);
                }
                else
                {
                    Assert.That(grabs.End(position, tick), Is.True);
                }
            }
            trajectory.EndStroke(60);

            ulong hash = offsetBasis;
            foreach (BladeTrajectorySegment segment in trajectory.Drain())
            {
                hash = (hash ^ segment.SegmentId) * prime;
                hash = (hash ^ segment.FixedTick) * prime;
                hash = (hash ^ FloatBits(segment.Query.Start.X)) * prime;
                hash = (hash ^ FloatBits(segment.Query.Start.Y)) * prime;
                hash = (hash ^ FloatBits(segment.Query.End.X)) * prime;
                hash = (hash ^ FloatBits(segment.Query.End.Y)) * prime;
                hash = (hash ^ FloatBits(segment.Query.SideNormal.Y)) * prime;
                hash = (hash ^ FloatBits(segment.Query.Radius)) * prime;
            }
            foreach (GrabCommand command in grabs.Drain())
            {
                hash = (hash ^ command.SequenceId) * prime;
                hash = (hash ^ command.FixedTick) * prime;
                hash = (hash ^ command.ParticleId) * prime;
                hash = (hash ^ (uint)command.Phase) * prime;
                hash = (hash ^ FloatBits(command.Position.X)) * prime;
                hash = (hash ^ FloatBits(command.Position.Y)) * prime;
            }
            return hash;
        }
    }
}
