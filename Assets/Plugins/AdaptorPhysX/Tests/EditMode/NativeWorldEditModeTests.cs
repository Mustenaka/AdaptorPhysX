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
        public void AbiVersionMatchesAdditiveVersionZeroPointSix()
        {
            NativeWorld.GetAbiVersion(out uint major, out uint minor);

            Assert.That(major, Is.EqualTo(0U));
            Assert.That(minor, Is.EqualTo(6U));
        }

        [Test]
        public void ManagedLayoutsMatchCAbiV0()
        {
            Assert.That(IntPtr.Size, Is.EqualTo(8), "P1-0 packages only Windows x64.");
            Assert.That(Marshal.SizeOf<ApxVec3>(), Is.EqualTo(12));
            Assert.That(Marshal.SizeOf<ApxVec4>(), Is.EqualTo(16));
            Assert.That(Marshal.SizeOf<ApxWorldDesc>(), Is.EqualTo(48));
            Assert.That(Marshal.SizeOf<ApxParticleDesc>(), Is.EqualTo(28));
            Assert.That(Marshal.SizeOf<ApxKinematicTarget>(), Is.EqualTo(20));
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
            AssertOffset<ApxKinematicTarget>(nameof(ApxKinematicTarget.ParticleId), 0);
            AssertOffset<ApxKinematicTarget>(nameof(ApxKinematicTarget.Active), 4);
            AssertOffset<ApxKinematicTarget>(nameof(ApxKinematicTarget.Position), 8);
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
        public void NativeMethodsExposeExactlyTheAdditiveTwentyTwoCdeclEntrypoints()
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
                "apxCreateWorldWithParticleCapacity",
                "apxDestroyWorld",
                "apxAddParticles",
                "apxAddDistanceConstraints",
                "apxAddClothDistanceConstraints",
                "apxAddBendConstraints",
                "apxGetBrokenClothDistanceConstraintIds",
                "apxSetRenderVertexBindings",
                "apxBindRenderVertices",
                "apxGetRenderVertexBindings",
                "apxSetRenderTriangles",
                "apxGetRenderVertexPositions",
                "apxGetRenderVertexNormals",
                "apxCut",
                "apxGetLastCutDetails",
                "apxSetKinematicTargets",
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
                Assert.Throws<ArgumentOutOfRangeException>(
                    () => world.GetRenderVertexPositions(IntPtr.Zero, -1));
                Assert.Throws<ArgumentNullException>(
                    () => world.GetRenderVertexPositions(IntPtr.Zero, 1));
                Assert.Throws<ArgumentOutOfRangeException>(
                    () => world.GetRenderVertexNormals(IntPtr.Zero, -1));
                Assert.Throws<ArgumentNullException>(
                    () => world.GetRenderVertexNormals(IntPtr.Zero, 1));
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
        public void DualGridAutoBindingSerializesAndRebuildsFiniteNormals(
            ApxBackendKind backend)
        {
            NativeWorld world;
            try
            {
                world = NativeWorld.Create(
                    ApxWorldDesc.Create(backend, FixedTimeStep, 2U, default, 0.0F, 3U));
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
                    new ApxParticleDesc(new ApxVec3(2.0F, 0.0F, 0.0F), default, 0.0F),
                    new ApxParticleDesc(new ApxVec3(0.0F, 2.0F, 0.0F), default, 0.0F),
                };
                Assert.That(world.AddParticles(particles), Is.EqualTo(0U));
                world.BindRenderVertices(
                    new[]
                    {
                        new ApxVec3(0.0F, 0.0F, 0.0F),
                        new ApxVec3(2.0F, 0.0F, 0.0F),
                        new ApxVec3(0.0F, 2.0F, 0.0F),
                    },
                    new uint[] { 0U, 1U, 2U });
                ApxRenderVertexBindingDesc[] serialized = world.GetRenderVertexBindings();
                Assert.That(serialized, Has.Length.EqualTo(3));
                Assert.That(serialized[0].WeightA, Is.EqualTo(1.0F));
                Assert.That(serialized[1].WeightB, Is.EqualTo(1.0F));
                Assert.That(serialized[2].WeightC, Is.EqualTo(1.0F));
                world.SetRenderTriangles(new uint[] { 0U, 1U, 2U });
                world.Step(FixedTimeStep);

                ApxVec3[] normals = new ApxVec3[3];
                Assert.That(world.GetRenderVertexNormalCount(), Is.EqualTo(3U));
                Assert.That(world.GetRenderVertexNormals(normals), Is.EqualTo(3));
                foreach (ApxVec3 normal in normals)
                {
                    Assert.That(float.IsNaN(normal.X), Is.False);
                    Assert.That(float.IsNaN(normal.Y), Is.False);
                    Assert.That(float.IsNaN(normal.Z), Is.False);
                    Assert.That(normal.Z, Is.EqualTo(1.0F).Within(1.0e-5F));
                }
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
                        5),
                    6U);
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
                ApxCutDetails details = world.GetLastCutDetails();
                CollectionAssert.AreEqual(
                    new uint[] { 2U, 3U },
                    details.DeactivatedConstraintIds);
                CollectionAssert.AreEqual(
                    new uint[] { 0U, 3U, 4U },
                    details.AffectedParticleIds);
                CollectionAssert.AreEqual(
                    new uint[] { 5U },
                    details.ActivatedParticleIds);
                uint[] undersizedConstraints = { 77U };
                uint[] affectedIds = { 77U, 88U, 99U };
                uint[] activatedIds = { 77U };
                ApxException capacity = Assert.Throws<ApxException>(
                    () => world.GetLastCutDetails(
                        undersizedConstraints,
                        affectedIds,
                        activatedIds,
                        out _,
                        out _,
                        out _));
                Assert.That(capacity.Result, Is.EqualTo(ApxResult.CapacityExceeded));
                CollectionAssert.AreEqual(new uint[] { 77U }, undersizedConstraints);
                CollectionAssert.AreEqual(new uint[] { 77U, 88U, 99U }, affectedIds);
                CollectionAssert.AreEqual(new uint[] { 77U }, activatedIds);
                ApxCutDetails repeatedDetails = world.GetLastCutDetails();
                CollectionAssert.AreEqual(
                    details.DeactivatedConstraintIds,
                    repeatedDetails.DeactivatedConstraintIds);
                CollectionAssert.AreEqual(
                    details.AffectedParticleIds,
                    repeatedDetails.AffectedParticleIds);
                CollectionAssert.AreEqual(
                    details.ActivatedParticleIds,
                    repeatedDetails.ActivatedParticleIds);
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
        public void CutCapacityExhaustionPreservesManagedAndNativeState(
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
                        3),
                    3U);
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
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(default, default, 1.0F),
                        new ApxParticleDesc(
                            new ApxVec3(1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                        new ApxParticleDesc(
                            new ApxVec3(-1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                    });
                world.AddClothDistanceConstraints(
                    new[]
                    {
                        new ApxClothDistanceConstraintDesc(
                            0, 1, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                        new ApxClothDistanceConstraintDesc(
                            0, 2, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                    });
                world.Step(FixedTimeStep * 0.5F);

                ApxException capacity = Assert.Throws<ApxException>(
                    () => world.Cut(
                        new ApxCutQuery(
                            new ApxVec3(0.0F, -0.5F, 0.0F),
                            new ApxVec3(0.0F, 0.5F, 0.0F),
                            new ApxVec3(1.0F, 0.0F, 0.0F),
                            0.01F)));
                Assert.That(capacity.Result, Is.EqualTo(ApxResult.CapacityExceeded));
                Assert.That(world.ParticleCount, Is.EqualTo(3U));
                ApxException noDetails = Assert.Throws<ApxException>(
                    () => world.GetLastCutDetails());
                Assert.That(noDetails.Result, Is.EqualTo(ApxResult.InvalidState));
                CollectionAssert.IsEmpty(world.GetBrokenClothDistanceConstraintIds());
                ApxVec3[] positions = new ApxVec3[3];
                Assert.That(world.ReadPositionSnapshot(positions), Is.EqualTo(3));
                foreach (ApxVec3 position in positions)
                {
                    AssertFinite(position);
                }
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
                        3),
                    4U);
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

            bool[] referenced = new bool[result.Mesh.Vertices.Length];
            float outputArea = 0.0F;
            int negativeTriangleCount = 0;
            int positiveTriangleCount = 0;
            for (int offset = 0; offset < result.Mesh.Indices.Length; offset += 3)
            {
                uint id0 = result.Mesh.Indices[offset];
                uint id1 = result.Mesh.Indices[offset + 1];
                uint id2 = result.Mesh.Indices[offset + 2];
                referenced[id0] = true;
                referenced[id1] = true;
                referenced[id2] = true;
                float triangleArea = TriangleArea(
                    result.Mesh.Vertices[id0].Position,
                    result.Mesh.Vertices[id1].Position,
                    result.Mesh.Vertices[id2].Position);
                Assert.That(triangleArea, Is.GreaterThan(0.0F));
                outputArea += triangleArea;
                float centroidX =
                    (result.Mesh.Vertices[id0].Position.X +
                     result.Mesh.Vertices[id1].Position.X +
                     result.Mesh.Vertices[id2].Position.X) / 3.0F;
                if (centroidX < 0.5F)
                {
                    ++negativeTriangleCount;
                }
                else
                {
                    Assert.That(centroidX, Is.GreaterThan(0.5F));
                    ++positiveTriangleCount;
                }
            }
            CollectionAssert.DoesNotContain(referenced, false);
            Assert.That(negativeTriangleCount, Is.EqualTo(3));
            Assert.That(positiveTriangleCount, Is.EqualTo(3));
            Assert.That(outputArea, Is.EqualTo(1.0F).Within(1.0e-6F));
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
            Assert.Throws<ArgumentException>(
                () => RenderMeshCutter.Cut(
                    new RenderMeshData(
                        new[]
                        {
                            input.Vertices[0],
                            input.Vertices[1],
                            new RenderMeshVertex(
                                new ApxVec3(2.0F, 0.0F, 0.0F),
                                new ApxVec3(0.0F, 0.0F, 1.0F),
                                2.0F,
                                0.0F),
                        },
                        new uint[] { 0U, 1U, 2U }),
                    CreateSquareCutQuery()));
            Assert.Throws<ArgumentException>(
                () => RenderMeshCutter.Cut(
                    new RenderMeshData(
                        new[]
                        {
                            input.Vertices[0],
                            input.Vertices[1],
                            input.Vertices[2],
                            input.Vertices[3],
                        },
                        new uint[] { 0U, 1U, 2U }),
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
        public void RenderMeshCutterHandlesVertexAndEdgeDegeneraciesWithoutInvalidTopology()
        {
            RenderMeshData input = CreateSquareRenderMesh();
            ApxCutQuery throughOppositeVertices = new ApxCutQuery(
                new ApxVec3(-1.0F, 1.0F, 0.0F),
                new ApxVec3(2.0F, -2.0F, 0.0F),
                new ApxVec3(1.0F, 1.0F, 0.0F),
                0.05F);
            ApxCutQuery alongSharedEdge = new ApxCutQuery(
                new ApxVec3(-1.0F, -1.0F, 0.0F),
                new ApxVec3(2.0F, 2.0F, 0.0F),
                new ApxVec3(1.0F, -1.0F, 0.0F),
                0.05F);

            RenderMeshCutResult vertexResult =
                RenderMeshCutter.Cut(input, throughOppositeVertices);
            RenderMeshCutResult edgeResult =
                RenderMeshCutter.Cut(input, alongSharedEdge);
            Assert.That(vertexResult.CutTriangleCount, Is.EqualTo(0U));
            Assert.That(vertexResult.CreatedSeamPairCount, Is.EqualTo(0U));
            Assert.That(edgeResult.CutTriangleCount, Is.EqualTo(0U));
            Assert.That(edgeResult.CreatedSeamPairCount, Is.EqualTo(0U));
            CollectionAssert.AreEqual(input.Indices, vertexResult.Mesh.Indices);
            CollectionAssert.AreEqual(input.Indices, edgeResult.Mesh.Indices);
        }

        [Test]
        public void RenderMeshCutterRebuildsFiniteSurfaceNormalsAfterCut()
        {
            RenderMeshData input = CreateSquareRenderMesh();
            for (int vertexId = 0; vertexId < input.Vertices.Length; ++vertexId)
            {
                RenderMeshVertex vertex = input.Vertices[vertexId];
                vertex.Normal = new ApxVec3(1.0F, 0.0F, 0.0F);
                input.Vertices[vertexId] = vertex;
            }

            RenderMeshCutResult result =
                RenderMeshCutter.Cut(input, CreateSquareCutQuery());
            foreach (RenderMeshVertex vertex in result.Mesh.Vertices)
            {
                AssertFinite(vertex.Normal);
                Assert.That(vertex.Normal.X, Is.EqualTo(0.0F));
                Assert.That(vertex.Normal.Y, Is.EqualTo(0.0F));
                Assert.That(vertex.Normal.Z, Is.EqualTo(1.0F));
            }
        }

        [Test]
        public void RenderMeshCutterProcessesMultiSegmentPolylineInCallerOrder()
        {
            ApxCutQuery[] trajectory =
            {
                new ApxCutQuery(
                    new ApxVec3(0.25F, -1.0F, 0.0F),
                    new ApxVec3(0.25F, 2.0F, 0.0F),
                    new ApxVec3(1.0F, 0.0F, 0.0F),
                    0.05F),
                new ApxCutQuery(
                    new ApxVec3(-1.0F, 0.75F, 0.0F),
                    new ApxVec3(2.0F, 0.75F, 0.0F),
                    new ApxVec3(0.0F, 1.0F, 0.0F),
                    0.05F),
            };

            RenderMeshCutResult first =
                RenderMeshCutter.CutPolyline(CreateSquareRenderMesh(), trajectory);
            RenderMeshCutResult second =
                RenderMeshCutter.CutPolyline(CreateSquareRenderMesh(), trajectory);
            Assert.That(first.CutTriangleCount, Is.GreaterThan(2U));
            Assert.That(first.CreatedSeamPairCount, Is.GreaterThan(3U));
            Assert.That(HashRenderMeshCutResult(first), Is.EqualTo(HashRenderMeshCutResult(second)));
            Assert.That(RenderMeshArea(first.Mesh), Is.EqualTo(1.0F).Within(1.0e-5F));
            AssertValidRenderTopology(first.Mesh);
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
        public void CoupledMeshCutterCommitsOneLiteralQueryToSimulationAndRenderMeshes()
        {
            using (NativeWorld world = NativeWorld.Create(
                ApxWorldDesc.Create(
                    ApxBackendKind.Cpu,
                    FixedTimeStep,
                    4,
                    default,
                    0.0F,
                    5),
                6U))
            {
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(default, default, 1.0F),
                        new ApxParticleDesc(
                            new ApxVec3(1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                        new ApxParticleDesc(
                            new ApxVec3(-1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                        new ApxParticleDesc(
                            new ApxVec3(0.0F, 1.0F, 0.0F),
                            default,
                            0.0F),
                        new ApxParticleDesc(
                            new ApxVec3(0.0F, -1.0F, 0.0F),
                            default,
                            0.0F),
                    });
                world.AddClothDistanceConstraints(
                    new[]
                    {
                        new ApxClothDistanceConstraintDesc(
                            0, 1, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                        new ApxClothDistanceConstraintDesc(
                            0, 2, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                        new ApxClothDistanceConstraintDesc(
                            0, 3, 1.0F, 0.0F, 100.0F, ApxClothDirection.Weft),
                        new ApxClothDistanceConstraintDesc(
                            0, 4, 1.0F, 0.0F, 100.0F, ApxClothDirection.Weft),
                    });
                world.Step(FixedTimeStep * 0.5F);
                ApxCutQuery query = new ApxCutQuery(
                    new ApxVec3(0.0F, -2.0F, 0.0F),
                    new ApxVec3(0.0F, 2.0F, 0.0F),
                    new ApxVec3(1.0F, 0.0F, 0.0F),
                    0.1F);

                CoupledMeshCutResult result =
                    CoupledMeshCutter.Cut(world, CreateCenteredSquareRenderMesh(), query);
                Assert.That(result.Render.CutTriangleCount, Is.EqualTo(2U));
                Assert.That(result.Render.CreatedSeamPairCount, Is.EqualTo(3U));
                Assert.That(result.Simulation.CutId, Is.EqualTo(0U));
                Assert.That(result.Simulation.CutConstraintCount, Is.EqualTo(2U));
                Assert.That(result.Simulation.SplitParticleCount, Is.EqualTo(1U));
                Assert.That(result.Simulation.FirstSplitParticleId, Is.EqualTo(5U));
                CollectionAssert.AreEqual(
                    new uint[] { 2U, 3U },
                    result.SimulationDetails.DeactivatedConstraintIds);
                CollectionAssert.AreEqual(
                    new uint[] { 0U, 3U, 4U },
                    result.SimulationDetails.AffectedParticleIds);
                CollectionAssert.AreEqual(
                    new uint[] { 5U },
                    result.SimulationDetails.ActivatedParticleIds);
            }
        }

        [Test]
        public void ObjCutLegacyPlanePathRemainsDefaultAndNumericallyStable()
        {
            Type objCutType = Type.GetType("ObjCut, Assembly-CSharp", true);
            UnityEngine.GameObject gameObject =
                new UnityEngine.GameObject("P1-5 Legacy ObjCut Regression");
            try
            {
                UnityEngine.Component component = gameObject.AddComponent(objCutType);
                FieldInfo preciseField = objCutType.GetField("usePreciseRenderMeshCut");
                Assert.That(preciseField, Is.Not.Null);
                Assert.That((bool)preciseField.GetValue(component), Is.False);

                objCutType.GetField("planeNormal").SetValue(
                    component,
                    new UnityEngine.Vector3(1.0F, 0.0F, 0.0F));
                objCutType.GetField("planePoint").SetValue(
                    component,
                    UnityEngine.Vector3.zero);
                MethodInfo trianglePlane = objCutType.GetMethod(
                    "DoesTriIntersectPlane",
                    BindingFlags.Instance | BindingFlags.NonPublic);
                bool[] classifications = (bool[])trianglePlane.Invoke(
                    component,
                    new object[]
                    {
                        new UnityEngine.Vector3(-1.0F, 0.0F, 0.0F),
                        new UnityEngine.Vector3(1.0F, 0.0F, 0.0F),
                        new UnityEngine.Vector3(1.0F, 1.0F, 0.0F),
                    });
                CollectionAssert.AreEqual(new[] { true, false, true }, classifications);

                MethodInfo intersect = objCutType.GetMethod(
                    "GetIntersectPoint",
                    BindingFlags.Instance | BindingFlags.NonPublic);
                UnityEngine.Vector3 point = (UnityEngine.Vector3)intersect.Invoke(
                    component,
                    new object[]
                    {
                        new UnityEngine.Vector3(1.0F, 0.0F, 0.0F),
                        UnityEngine.Vector3.zero,
                        new UnityEngine.Vector3(-1.0F, 0.0F, 0.0F),
                        new UnityEngine.Vector3(1.0F, 0.0F, 0.0F),
                    });
                Assert.That(point, Is.EqualTo(UnityEngine.Vector3.zero));
                Assert.That(
                    UnityEditor.AssetDatabase.AssetPathToGUID(
                        "Assets/Scripts/APEX/Usage/ObjCut.cs"),
                    Is.EqualTo("14ff20beb35d4474883ef8a2d59846e0"));
                Assert.That(
                    UnityEditor.AssetDatabase.AssetPathToGUID(
                        "Assets/Scenes/CutTest.unity"),
                    Is.EqualTo("cde88b09b7379fa4dad0251dd7c34824"));

                Type intersectUtilType = Type.GetType(
                    "APEX.Math.Graphics.IntersectUtil, Assembly-CSharp",
                    true);
                MethodInfo sharedPrimitive = intersectUtilType.GetMethod(
                    "TryIntersectSegmentPlane",
                    BindingFlags.Static | BindingFlags.Public);
                Type float3Type = Type.GetType(
                    "Unity.Mathematics.float3, Unity.Mathematics",
                    true);
                object zero = Activator.CreateInstance(
                    float3Type,
                    new object[] { 0.0F, 0.0F, 0.0F });
                object[] primitiveArguments =
                {
                    Activator.CreateInstance(
                        float3Type,
                        new object[] { -1.0F, 0.0F, 0.0F }),
                    Activator.CreateInstance(
                        float3Type,
                        new object[] { 1.0F, 0.0F, 0.0F }),
                    zero,
                    Activator.CreateInstance(
                        float3Type,
                        new object[] { 1.0F, 0.0F, 0.0F }),
                    0.0F,
                    zero,
                };
                Assert.That(
                    (bool)sharedPrimitive.Invoke(null, primitiveArguments),
                    Is.True);
                Assert.That((float)primitiveArguments[4], Is.EqualTo(0.5F));
                Assert.That(
                    (float)float3Type.GetField("x").GetValue(primitiveArguments[5]),
                    Is.EqualTo(0.0F));
                Assert.That(
                    (float)float3Type.GetField("y").GetValue(primitiveArguments[5]),
                    Is.EqualTo(0.0F));
                Assert.That(
                    (float)float3Type.GetField("z").GetValue(primitiveArguments[5]),
                    Is.EqualTo(0.0F));
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(gameObject);
            }
        }

        [Test]
        public void ObjCutPreciseEntryPublishesFiniteMeshUvNormalsAndCollider()
        {
            Type objCutType = Type.GetType("ObjCut, Assembly-CSharp", true);
            UnityEngine.GameObject cutterObject =
                new UnityEngine.GameObject("P1-5 Precise Cutter");
            UnityEngine.GameObject targetObject =
                new UnityEngine.GameObject("P1-5 Precise Target");
            UnityEngine.Mesh source = new UnityEngine.Mesh { name = "P1-5 Square" };
            UnityEngine.Mesh published = null;
            try
            {
                source.vertices = new[]
                {
                    new UnityEngine.Vector3(0.0F, 0.0F, 0.0F),
                    new UnityEngine.Vector3(1.0F, 0.0F, 0.0F),
                    new UnityEngine.Vector3(1.0F, 1.0F, 0.0F),
                    new UnityEngine.Vector3(0.0F, 1.0F, 0.0F),
                };
                source.normals = new[]
                {
                    UnityEngine.Vector3.forward,
                    UnityEngine.Vector3.forward,
                    UnityEngine.Vector3.forward,
                    UnityEngine.Vector3.forward,
                };
                source.uv = new[]
                {
                    new UnityEngine.Vector2(0.0F, 0.0F),
                    new UnityEngine.Vector2(1.0F, 0.0F),
                    new UnityEngine.Vector2(1.0F, 1.0F),
                    new UnityEngine.Vector2(0.0F, 1.0F),
                };
                source.triangles = new[] { 0, 1, 2, 0, 2, 3 };
                UnityEngine.MeshFilter meshFilter =
                    targetObject.AddComponent<UnityEngine.MeshFilter>();
                UnityEngine.MeshCollider meshCollider =
                    targetObject.AddComponent<UnityEngine.MeshCollider>();
                meshFilter.sharedMesh = source;
                meshCollider.sharedMesh = source;

                UnityEngine.Component cutter = cutterObject.AddComponent(objCutType);
                objCutType.GetField("target").SetValue(cutter, targetObject);
                MethodInfo preciseCut = objCutType.GetMethod(
                    "ApplyPreciseWorldCut",
                    BindingFlags.Instance | BindingFlags.Public);
                preciseCut.Invoke(
                    cutter,
                    new object[] { CreateSquareCutQuery() });

                published = meshFilter.sharedMesh;
                Assert.That(published, Is.Not.SameAs(source));
                Assert.That(published.vertexCount, Is.EqualTo(10));
                Assert.That(published.triangles, Has.Length.EqualTo(18));
                Assert.That(published.uv, Has.Length.EqualTo(10));
                Assert.That(published.normals, Has.Length.EqualTo(10));
                foreach (UnityEngine.Vector3 normal in published.normals)
                {
                    Assert.That(float.IsNaN(normal.x), Is.False);
                    Assert.That(float.IsNaN(normal.y), Is.False);
                    Assert.That(float.IsNaN(normal.z), Is.False);
                    Assert.That(normal.z, Is.EqualTo(1.0F));
                }
                Assert.That(meshCollider.sharedMesh, Is.SameAs(published));
            }
            finally
            {
                if (published != null && published != source)
                {
                    UnityEngine.Object.DestroyImmediate(published);
                }
                UnityEngine.Object.DestroyImmediate(source);
                UnityEngine.Object.DestroyImmediate(cutterObject);
                UnityEngine.Object.DestroyImmediate(targetObject);
            }
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
            warmup = null;

            double[] samples = new double[5];
            int[] allocationBlocks = new int[5];
            long retainedPayloadBytes = 0L;
            UnityEngine.Profiling.Recorder allocationRecorder =
                UnityEngine.Profiling.Recorder.Get("GC.Alloc");
            allocationRecorder.enabled = false;
            allocationRecorder.FilterToCurrentThread();
            try
            {
                for (int sample = 0; sample < samples.Length; ++sample)
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    System.Diagnostics.Stopwatch stopwatch =
                        new System.Diagnostics.Stopwatch();
                    allocationRecorder.enabled = false;
                    allocationRecorder.enabled = true;
                    stopwatch.Start();
                    RenderMeshCutResult result = RenderMeshCutter.Cut(input, query);
                    stopwatch.Stop();
                    allocationRecorder.enabled = false;
                    allocationBlocks[sample] = allocationRecorder.sampleBlockCount;
                    Assert.That(result.CutTriangleCount, Is.EqualTo(400U));
                    Assert.That(result.CreatedSeamPairCount, Is.EqualTo(401U));
                    samples[sample] = stopwatch.Elapsed.TotalMilliseconds;
                    retainedPayloadBytes =
                        result.Mesh.Vertices.LongLength *
                            Marshal.SizeOf<RenderMeshVertex>() +
                        result.Mesh.Indices.LongLength * sizeof(uint) +
                        result.SeamPairs.LongLength *
                            Marshal.SizeOf<RenderMeshSeamPair>();
                    GC.KeepAlive(result);
                }
            }
            finally
            {
                allocationRecorder.enabled = false;
                allocationRecorder.CollectFromAllThreads();
            }

            Array.Sort(samples);
            Array.Sort(allocationBlocks);
            double medianMilliseconds = samples[samples.Length / 2];
            int medianAllocationBlocks =
                allocationBlocks[allocationBlocks.Length / 2];
            TestContext.Out.WriteLine(
                "APX_RENDER_CUT_PERF triangles=100000 samples=5 median_ms={0:F4} " +
                "median_gc_alloc_blocks={1} retained_payload_bytes={2} " +
                "gate_ms=1500.0000 gate_gc_alloc_blocks=5000 " +
                "gate_retained_payload_bytes=8388608",
                medianMilliseconds,
                medianAllocationBlocks,
                retainedPayloadBytes);
            Assert.That(medianMilliseconds, Is.LessThanOrEqualTo(1500.0));
            Assert.That(medianAllocationBlocks, Is.LessThanOrEqualTo(5000));
            Assert.That(retainedPayloadBytes, Is.LessThanOrEqualTo(8L * 1024L * 1024L));
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
        public void KinematicGrabMovesPinsAndReleasesNativeParticle()
        {
            using (NativeWorld world = NativeWorld.Create(
                ApxWorldDesc.Create(
                    ApxBackendKind.Cpu,
                    FixedTimeStep,
                    4,
                    new ApxVec3(0.0F, -9.81F, 0.0F))))
            {
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(default, default, 1.0F),
                    });
                ApxKinematicTarget[] batch =
                {
                    new ApxKinematicTarget(
                        0U,
                        true,
                        new ApxVec3(2.0F, 3.0F, 4.0F)),
                };

                world.SetKinematicTargets(batch);
                world.Step(FixedTimeStep);
                AssertPosition(world, 2.0F, 3.0F, 4.0F);

                batch[0] = new ApxKinematicTarget(
                    0U,
                    true,
                    new ApxVec3(-1.0F, 5.0F, 0.5F));
                world.SetKinematicTargets(batch);
                world.Step(FixedTimeStep);
                AssertPosition(world, -1.0F, 5.0F, 0.5F);

                batch[0] = new ApxKinematicTarget(
                    0U,
                    false,
                    new ApxVec3(-1.0F, 5.0F, 0.5F));
                world.SetKinematicTargets(batch);
                world.Step(FixedTimeStep);
                using (MappedPositionSnapshot mapped = world.MapPositions())
                {
                    Assert.That(mapped[0].X, Is.EqualTo(-1.0F));
                    Assert.That(mapped[0].Y, Is.LessThan(5.0F));
                    Assert.That(mapped[0].Z, Is.EqualTo(0.5F));
                    AssertFinite(mapped[0]);
                }

                Assert.Throws<ArgumentNullException>(
                    () => world.SetKinematicTargets(null));
                ApxException badId = Assert.Throws<ApxException>(
                    () => world.SetKinematicTargets(
                        new[]
                        {
                            new ApxKinematicTarget(1U, true, default),
                        }));
                Assert.That(badId.Result, Is.EqualTo(ApxResult.InvalidArgument));
                ApxException badFlag = Assert.Throws<ApxException>(
                    () => world.SetKinematicTargets(
                        new[]
                        {
                            new ApxKinematicTarget
                            {
                                ParticleId = 0U,
                                Active = 2U,
                                Position = default,
                            },
                        }));
                Assert.That(badFlag.Result, Is.EqualTo(ApxResult.InvalidArgument));
                ApxException badPosition = Assert.Throws<ApxException>(
                    () => world.SetKinematicTargets(
                        new[]
                        {
                            new ApxKinematicTarget(
                                0U,
                                true,
                                new ApxVec3(float.NaN, 0.0F, 0.0F)),
                        }));
                Assert.That(badPosition.Result, Is.EqualTo(ApxResult.InvalidArgument));
            }
        }

        [Test]
        public void PointerRayFeedsFixedTickTrajectoryIntoNativeCut()
        {
            Type interactorType = Type.GetType(
                "APEX.Usage.ApxBladeInteractor, Assembly-CSharp",
                true);
            Type pickTargetType = Type.GetType(
                "APEX.Usage.ApxParticlePickTarget, Assembly-CSharp",
                true);
            UnityEngine.GameObject interactorObject =
                new UnityEngine.GameObject("P1-6 ray interactor");
            UnityEngine.GameObject hitObject =
                UnityEngine.GameObject.CreatePrimitive(UnityEngine.PrimitiveType.Cube);
            NativeWorld world = null;
            try
            {
                world = NativeWorld.Create(
                    ApxWorldDesc.Create(
                        ApxBackendKind.Cpu,
                        FixedTimeStep,
                        4,
                        default,
                        0.1F,
                        3),
                    4U);
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(default, default, 1.0F),
                        new ApxParticleDesc(
                            new ApxVec3(1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                        new ApxParticleDesc(
                            new ApxVec3(-1.0F, 0.0F, 0.0F),
                            default,
                            0.0F),
                    });
                world.AddClothDistanceConstraints(
                    new[]
                    {
                        new ApxClothDistanceConstraintDesc(
                            0U, 1U, 1.0F, 0.0F, 100.0F, ApxClothDirection.Warp),
                        new ApxClothDistanceConstraintDesc(
                            0U, 2U, 1.0F, 0.0F, 100.0F, ApxClothDirection.Weft),
                    });
                world.Step(FixedTimeStep * 0.5F);

                UnityEngine.Component interactor =
                    interactorObject.AddComponent(interactorType);
                int interactionLayer = 30;
                hitObject.layer = interactionLayer;
                interactorType.GetField("minimumSampleDistance").SetValue(interactor, 0.0F);
                interactorType.GetField("cutRadius").SetValue(interactor, 0.01F);
                interactorType.GetField("interactionMask").SetValue(
                    interactor,
                    (UnityEngine.LayerMask)(1 << interactionLayer));
                interactorType.GetMethod(
                    "Awake",
                    BindingFlags.Instance | BindingFlags.NonPublic).Invoke(
                    interactor,
                    null);
                interactorObject.transform.rotation = UnityEngine.Quaternion.LookRotation(
                    UnityEngine.Vector3.right,
                    UnityEngine.Vector3.up);
                interactorType.GetMethod("BindNativeWorld").Invoke(
                    interactor,
                    new object[] { world });

                UnityEngine.Component pickTarget = hitObject.AddComponent(pickTargetType);
                pickTargetType.GetField("particleId").SetValue(pickTarget, 0U);
                hitObject.transform.localScale =
                    new UnityEngine.Vector3(0.1F, 0.1F, 0.0001F);
                MethodInfo submitRay = interactorType.GetMethod("TrySubmitPointerRay");
                MethodInfo fixedUpdate = interactorType.GetMethod(
                    "FixedUpdate",
                    BindingFlags.Instance | BindingFlags.NonPublic);

                hitObject.transform.position =
                    new UnityEngine.Vector3(0.0F, -0.5F, 0.00005F);
                UnityEngine.Physics.SyncTransforms();
                Assert.That(
                    submitRay.Invoke(
                        interactor,
                        new object[]
                        {
                            new UnityEngine.Ray(
                                new UnityEngine.Vector3(0.0F, -0.5F, -2.0F),
                                UnityEngine.Vector3.forward),
                            true,
                            false,
                        }),
                    Is.True);
                fixedUpdate.Invoke(interactor, null);

                hitObject.transform.position =
                    new UnityEngine.Vector3(0.0F, 0.5F, 0.00005F);
                UnityEngine.Physics.SyncTransforms();
                Assert.That(
                    submitRay.Invoke(
                        interactor,
                        new object[]
                        {
                            new UnityEngine.Ray(
                                new UnityEngine.Vector3(0.0F, 0.5F, -2.0F),
                                UnityEngine.Vector3.forward),
                            true,
                            false,
                        }),
                    Is.True);
                fixedUpdate.Invoke(interactor, null);

                Assert.That(
                    interactorType.GetField("nativeGrabParticleId").GetValue(interactor),
                    Is.EqualTo(0U));
                Assert.That(world.ParticleCount, Is.EqualTo(4U));
                CollectionAssert.AreEqual(
                    new uint[] { 3U },
                    world.GetLastCutDetails().ActivatedParticleIds);
            }
            finally
            {
                world?.Dispose();
                UnityEngine.Object.DestroyImmediate(interactorObject);
                UnityEngine.Object.DestroyImmediate(hitObject);
            }
        }

        [Test]
        public void InteractionRecordingRoundTripsImmutableFixedTickEvents()
        {
            InteractionReplayTimeline timeline = CreateRecordedGrabTimeline();
            Assert.That(timeline.Count, Is.EqualTo(60));
            RecordedInteractionEvent[] serialized = timeline.ToArray();
            Assert.That(serialized[0].SequenceId, Is.EqualTo(0U));
            Assert.That(serialized[0].FixedTick, Is.EqualTo(1UL));
            Assert.That(serialized[0].Kind, Is.EqualTo(InteractionEventKind.GrabBegin));
            Assert.That(serialized[59].Kind, Is.EqualTo(InteractionEventKind.GrabEnd));
            ulong hash = timeline.ContentHash;
            serialized[0].Position.X = 999.0F;
            Assert.That(timeline[0].Position.X, Is.EqualTo(0.125F));
            Assert.That(timeline.ContentHash, Is.EqualTo(hash));

            InteractionReplayCursor cursor = new InteractionReplayCursor(timeline);
            for (ulong tick = 1; tick <= 60; ++tick)
            {
                Assert.That(cursor.TryDequeue(tick, out RecordedInteractionEvent item), Is.True);
                Assert.That(item.FixedTick, Is.EqualTo(tick));
                Assert.That(cursor.TryDequeue(tick, out _), Is.False);
            }
            Assert.That(cursor.IsComplete, Is.True);
            Assert.Throws<ArgumentOutOfRangeException>(
                () => cursor.TryDequeue(59UL, out _));
            Assert.Throws<ArgumentException>(
                () => new InteractionReplayTimeline(
                    new[]
                    {
                        new RecordedInteractionEvent
                        {
                            SequenceId = 1U,
                            FixedTick = 1UL,
                            Kind = InteractionEventKind.GrabMove,
                        },
                    }));
        }

        [Test]
        public void BladeInteractionDeterminismMatchesCompleteSixtyTickCommandHash()
        {
            ulong first = RunBladeInteractionHash();
            ulong second = RunBladeInteractionHash();
            Assert.That(first, Is.EqualTo(second));
        }

        [Test]
        public void RecordedGrabReplayProducesBitExactNativeStateForSixtyTicks()
        {
            InteractionReplayTimeline timeline = CreateRecordedGrabTimeline();
            ulong first = RunNativeInteractionReplayHash(timeline);
            ulong second = RunNativeInteractionReplayHash(
                new InteractionReplayTimeline(timeline.ToArray()));
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

        private static RenderMeshData CreateCenteredSquareRenderMesh()
        {
            ApxVec3 normal = new ApxVec3(0.0F, 0.0F, 1.0F);
            return new RenderMeshData(
                new[]
                {
                    new RenderMeshVertex(
                        new ApxVec3(-1.0F, -1.0F, 0.0F),
                        normal,
                        0.0F,
                        0.0F),
                    new RenderMeshVertex(
                        new ApxVec3(1.0F, -1.0F, 0.0F),
                        normal,
                        1.0F,
                        0.0F),
                    new RenderMeshVertex(
                        new ApxVec3(1.0F, 1.0F, 0.0F),
                        normal,
                        1.0F,
                        1.0F),
                    new RenderMeshVertex(
                        new ApxVec3(-1.0F, 1.0F, 0.0F),
                        normal,
                        0.0F,
                        1.0F),
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

        private static float RenderMeshArea(RenderMeshData mesh)
        {
            float area = 0.0F;
            for (int offset = 0; offset < mesh.Indices.Length; offset += 3)
            {
                area += TriangleArea(
                    mesh.Vertices[mesh.Indices[offset]].Position,
                    mesh.Vertices[mesh.Indices[offset + 1]].Position,
                    mesh.Vertices[mesh.Indices[offset + 2]].Position);
            }
            return area;
        }

        private static float TriangleArea(ApxVec3 first, ApxVec3 second, ApxVec3 third)
        {
            float firstX = second.X - first.X;
            float firstY = second.Y - first.Y;
            float firstZ = second.Z - first.Z;
            float secondX = third.X - first.X;
            float secondY = third.Y - first.Y;
            float secondZ = third.Z - first.Z;
            float crossX = firstY * secondZ - firstZ * secondY;
            float crossY = firstZ * secondX - firstX * secondZ;
            float crossZ = firstX * secondY - firstY * secondX;
            return 0.5F * (float)Math.Sqrt(
                crossX * crossX + crossY * crossY + crossZ * crossZ);
        }

        private static void AssertValidRenderTopology(RenderMeshData mesh)
        {
            Assert.That(mesh.Indices.Length % 3, Is.EqualTo(0));
            bool[] referenced = new bool[mesh.Vertices.Length];
            for (int offset = 0; offset < mesh.Indices.Length; offset += 3)
            {
                uint id0 = mesh.Indices[offset];
                uint id1 = mesh.Indices[offset + 1];
                uint id2 = mesh.Indices[offset + 2];
                Assert.That(id0, Is.LessThan((uint)mesh.Vertices.Length));
                Assert.That(id1, Is.LessThan((uint)mesh.Vertices.Length));
                Assert.That(id2, Is.LessThan((uint)mesh.Vertices.Length));
                Assert.That(
                    TriangleArea(
                        mesh.Vertices[id0].Position,
                        mesh.Vertices[id1].Position,
                        mesh.Vertices[id2].Position),
                    Is.GreaterThan(0.0F));
                referenced[id0] = true;
                referenced[id1] = true;
                referenced[id2] = true;
            }
            CollectionAssert.DoesNotContain(referenced, false);
            foreach (RenderMeshVertex vertex in mesh.Vertices)
            {
                AssertFinite(vertex.Position);
                AssertFinite(vertex.Normal);
            }
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

        private static void AssertPosition(
            NativeWorld world,
            float expectedX,
            float expectedY,
            float expectedZ)
        {
            using (MappedPositionSnapshot mapped = world.MapPositions())
            {
                Assert.That(mapped.Count, Is.EqualTo(1));
                Assert.That(mapped[0].X, Is.EqualTo(expectedX));
                Assert.That(mapped[0].Y, Is.EqualTo(expectedY));
                Assert.That(mapped[0].Z, Is.EqualTo(expectedZ));
            }
        }

        private static InteractionReplayTimeline CreateRecordedGrabTimeline()
        {
            InteractionRecorder recorder = new InteractionRecorder(60);
            for (uint tick = 1; tick <= 60; ++tick)
            {
                GrabCommandPhase phase = tick == 1
                    ? GrabCommandPhase.Begin
                    : tick == 60
                        ? GrabCommandPhase.End
                        : GrabCommandPhase.Move;
                recorder.Record(
                    new GrabCommand(
                        tick - 1U,
                        tick,
                        0U,
                        phase,
                        new ApxVec3(
                            tick * 0.125F,
                            tick * 0.03125F,
                            -((float)tick * 0.015625F))));
            }
            return recorder.BuildTimeline();
        }

        private static ulong RunNativeInteractionReplayHash(
            InteractionReplayTimeline timeline)
        {
            const ulong offsetBasis = 1469598103934665603UL;
            const ulong prime = 1099511628211UL;
            InteractionReplayCursor cursor = new InteractionReplayCursor(timeline);
            ApxKinematicTarget[] target = new ApxKinematicTarget[1];
            ulong hash = offsetBasis;
            using (NativeWorld world = NativeWorld.Create(
                ApxWorldDesc.Create(
                    ApxBackendKind.Cpu,
                    FixedTimeStep,
                    4,
                    new ApxVec3(0.0F, -9.81F, 0.0F))))
            {
                world.AddParticles(
                    new[]
                    {
                        new ApxParticleDesc(default, default, 1.0F),
                    });
                for (ulong tick = 1; tick <= 60; ++tick)
                {
                    Assert.That(
                        cursor.TryDequeue(tick, out RecordedInteractionEvent item),
                        Is.True);
                    target[0] = new ApxKinematicTarget(
                        item.ParticleId,
                        item.Kind != InteractionEventKind.GrabEnd,
                        item.Position);
                    world.SetKinematicTargets(target);
                    world.Step(FixedTimeStep);
                    using (MappedPositionSnapshot mapped = world.MapPositions())
                    {
                        hash = (hash ^ tick) * prime;
                        hash = (hash ^ FloatBits(mapped[0].X)) * prime;
                        hash = (hash ^ FloatBits(mapped[0].Y)) * prime;
                        hash = (hash ^ FloatBits(mapped[0].Z)) * prime;
                    }
                }
            }
            Assert.That(cursor.IsComplete, Is.True);
            return hash;
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
