using System;
using System.Collections;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

namespace APEX.Native.Tests
{
    public sealed class PhaseOneDemoEditModeTests
    {
        private const float FixedTimeStep = 1.0F / 120.0F;
        private const float FrameDeltaTime = 1.0F / 60.0F;

        [Test]
        public void WorkloadAndReplayValidateStableIdsCountsAndCursorBoundaries()
        {
            PhaseOneClothWorkload workload = PhaseOneClothWorkload.Create(
                4,
                3,
                0.25F,
                1.0e-6F,
                2.0e-6F,
                1.5F);
            Assert.That(workload.Particles, Has.Length.EqualTo(12));
            Assert.That(workload.ClothConstraints, Has.Length.EqualTo(17));
            Assert.That(workload.RenderBindings, Has.Length.EqualTo(12));
            Assert.That(workload.TriangleIndices, Has.Length.EqualTo(36));
            Assert.That(workload.Particles[0].InverseMass, Is.EqualTo(0.0F));
            Assert.That(workload.Particles[3].InverseMass, Is.EqualTo(0.0F));
            Assert.That(workload.Particles[4].InverseMass, Is.EqualTo(1.0F));
            Assert.That(
                workload.ClothConstraints[0].Direction,
                Is.EqualTo(ApxClothDirection.Warp));
            Assert.That(
                workload.ClothConstraints[9].Direction,
                Is.EqualTo(ApxClothDirection.Weft));
            Assert.That(workload.RenderBindings[11].ParticleA, Is.EqualTo(11U));
            Assert.That(workload.RenderBindings[11].ParticleB, Is.EqualTo(0U));
            Assert.That(workload.RenderBindings[11].ParticleC, Is.EqualTo(1U));
            CollectionAssert.AreEqual(
                new uint[] { 0U, 4U, 1U, 1U, 4U, 5U },
                new ArraySegment<uint>(workload.TriangleIndices, 0, 6));

            Assert.Throws<ArgumentOutOfRangeException>(
                () => PhaseOneClothWorkload.Create(2, 3, 0.25F, 0.0F, 0.0F, 1.5F));
            Assert.Throws<ArgumentOutOfRangeException>(
                () => PhaseOneClothWorkload.Create(4, 3, float.NaN, 0.0F, 0.0F, 1.5F));
            Assert.Throws<ArgumentOutOfRangeException>(
                () => workload.CreateVerticalCut(0, 0.01F));

            ApxCutQuery firstQuery = workload.CreateVerticalCut(1, 0.01F);
            ApxCutQuery secondQuery = workload.CreateVerticalCut(2, 0.01F);
            PhaseOneReplayCommand[] source =
            {
                new PhaseOneReplayCommand(0U, 2U, firstQuery),
                new PhaseOneReplayCommand(1U, 2U, secondQuery),
                new PhaseOneReplayCommand(2U, 5U, firstQuery),
            };
            PhaseOneReplayTimeline timeline = new PhaseOneReplayTimeline(source);
            ulong originalHash = timeline.ContentHash;
            source[0] = new PhaseOneReplayCommand(0U, 99U, secondQuery);
            Assert.That(timeline.ContentHash, Is.EqualTo(originalHash));

            PhaseOneReplayCursor cursor = new PhaseOneReplayCursor(timeline);
            Assert.That(cursor.TryDequeue(1U, out _), Is.False);
            Assert.That(cursor.TryDequeue(2U, out PhaseOneReplayCommand first), Is.True);
            Assert.That(first.SequenceId, Is.EqualTo(0U));
            Assert.That(cursor.TryDequeue(2U, out PhaseOneReplayCommand second), Is.True);
            Assert.That(second.SequenceId, Is.EqualTo(1U));
            Assert.That(cursor.TryDequeue(2U, out _), Is.False);
            Assert.That(cursor.TryDequeue(3U, out _), Is.False);
            Assert.That(cursor.TryDequeue(5U, out PhaseOneReplayCommand last), Is.True);
            Assert.That(last.SequenceId, Is.EqualTo(2U));
            Assert.That(cursor.IsComplete, Is.True);
            Assert.Throws<ArgumentOutOfRangeException>(
                () => cursor.TryDequeue(4U, out _));

            PhaseOneReplayCursor skipped = new PhaseOneReplayCursor(timeline);
            Assert.Throws<InvalidOperationException>(
                () => skipped.TryDequeue(3U, out _));
            Assert.Throws<ArgumentException>(
                () => new PhaseOneReplayTimeline(
                    new[] { new PhaseOneReplayCommand(1U, 1U, firstQuery) }));
            ApxCutQuery degenerate = new ApxCutQuery(
                default,
                default,
                new ApxVec3(1.0F, 0.0F, 0.0F),
                0.0F);
            Assert.Throws<ArgumentException>(
                () => new PhaseOneReplayTimeline(
                    new[] { new PhaseOneReplayCommand(0U, 1U, degenerate) }));
        }

        [Test]
        public void ReplayProducesBitExactCpuHashForEveryOneOfSixtyFrames()
        {
            ReplayOutcome first = RunReplayScene(ApxBackendKind.Cpu);
            ReplayOutcome second = RunReplayScene(ApxBackendKind.Cpu);
            CollectionAssert.AreEqual(first.FrameHashes, second.FrameHashes);
            AssertCutResultsEqual(first.CutResult, second.CutResult);
            CollectionAssert.AreEqual(first.BrokenConstraintIds, second.BrokenConstraintIds);
        }

        [Test]
        public void ReplayCpuCudaMatchesDiscreteStateAndDl4FloatTolerances()
        {
            ReplayOutcome cpu = RunReplayScene(ApxBackendKind.Cpu);
            ReplayOutcome cuda = RunReplayScene(ApxBackendKind.Cuda);
            AssertCutResultsEqual(cpu.CutResult, cuda.CutResult);
            Assert.That(cuda.ParticleCount, Is.EqualTo(cpu.ParticleCount));
            CollectionAssert.AreEqual(cpu.BrokenConstraintIds, cuda.BrokenConstraintIds);
            AssertPositionsNear(cpu.StepOnePositions, cuda.StepOnePositions, 1.0e-5F);
            AssertPositionsNear(cpu.FinalPositions, cuda.FinalPositions, 1.0e-4F);
        }

        [Test]
        public unsafe void HundredThousandReplayMeetsStepAndRenderReadbackGate()
        {
            const int columns = 400;
            const int rows = 250;
            const int initialParticleCount = columns * rows;
            const int expectedConstraintCount =
                rows * (columns - 1) + (rows - 1) * columns;
            const int expectedRenderTriangleCount =
                (rows - 1) * (columns - 1) * 2;
            const int warmupFrames = 120;
            const int sampleFrames = 600;
            const double medianGateMilliseconds = 16.667;
            const double p95GateMilliseconds = 33.334;

            PhaseOneClothWorkload workload = PhaseOneClothWorkload.Create(
                columns,
                rows,
                0.02F,
                1.0e-7F,
                2.0e-6F,
                100.0F);
            Assert.That(workload.Particles, Has.Length.EqualTo(initialParticleCount));
            Assert.That(
                workload.ClothConstraints,
                Has.Length.EqualTo(expectedConstraintCount));
            Assert.That(
                workload.RenderBindings,
                Has.Length.EqualTo(initialParticleCount));
            Assert.That(
                workload.TriangleIndices,
                Has.Length.EqualTo(expectedRenderTriangleCount * 3));

            using (NativeWorld world = NativeWorld.Create(
                ApxWorldDesc.Create(
                    ApxBackendKind.Cuda,
                    FixedTimeStep,
                    2U,
                    new ApxVec3(0.0F, -9.81F, 0.0F),
                    0.006F,
                    (uint)initialParticleCount)))
            {
                Assert.That(world.AddParticles(workload.Particles), Is.EqualTo(0U));
                Assert.That(
                    world.AddClothDistanceConstraints(workload.ClothConstraints),
                    Is.EqualTo(0U));
                world.SetRenderVertexBindings(workload.RenderBindings);
                world.SetRenderTriangles(workload.TriangleIndices);
                world.Step(FixedTimeStep * 0.5F);

                Stopwatch cutTimer = Stopwatch.StartNew();
                ApxCutResult cut = world.Cut(
                    workload.CreateVerticalCut(columns / 2, 0.001F));
                cutTimer.Stop();
                Assert.That(cut.CutConstraintCount, Is.EqualTo((uint)(rows - 1)));
                Assert.That(cut.SplitParticleCount, Is.EqualTo((uint)rows));
                Assert.That(cut.FirstSplitParticleId, Is.EqualTo((uint)initialParticleCount));
                Assert.That(
                    world.ParticleCount,
                    Is.EqualTo((uint)(initialParticleCount + rows)));

                NativeArray<Vector3> unityPositions = new NativeArray<Vector3>(
                    initialParticleCount,
                    Allocator.Persistent,
                    NativeArrayOptions.UninitializedMemory);
                NativeArray<Vector3> unityNormals = new NativeArray<Vector3>(
                    initialParticleCount,
                    Allocator.Persistent,
                    NativeArrayOptions.UninitializedMemory);
                try
                {
                    Mesh mesh = new Mesh
                    {
                        indexFormat = UnityEngine.Rendering.IndexFormat.UInt32,
                    };
                    try
                    {
                        int[] triangleIndices = new int[workload.TriangleIndices.Length];
                        for (int index = 0; index < triangleIndices.Length; ++index)
                        {
                            triangleIndices[index] =
                                checked((int)workload.TriangleIndices[index]);
                        }
                        for (int index = 0; index < initialParticleCount; ++index)
                        {
                            ApxVec3 position = workload.Particles[index].Position;
                            unityPositions[index] =
                                new Vector3(position.X, position.Y, position.Z);
                            unityNormals[index] = Vector3.forward;
                        }
                        mesh.SetVertices(unityPositions);
                        mesh.SetNormals(unityNormals);
                        mesh.triangles = triangleIndices;
                        mesh.MarkDynamic();
                        const UnityEngine.Rendering.MeshUpdateFlags uploadFlags =
                            UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds |
                            UnityEngine.Rendering.MeshUpdateFlags.DontValidateIndices |
                            UnityEngine.Rendering.MeshUpdateFlags.DontNotifyMeshUsers |
                            UnityEngine.Rendering.MeshUpdateFlags.DontResetBoneBounds;
                        IntPtr positionBuffer =
                            (IntPtr)NativeArrayUnsafeUtility.GetUnsafePtr(unityPositions);
                        IntPtr normalBuffer =
                            (IntPtr)NativeArrayUnsafeUtility.GetUnsafePtr(unityNormals);

                        for (int frame = 0; frame < warmupFrames; ++frame)
                        {
                            world.Step(FrameDeltaTime);
                            int written = world.GetRenderVertexPositions(
                                positionBuffer,
                                unityPositions.Length);
                            int normalWritten = world.GetRenderVertexNormals(
                                normalBuffer,
                                unityNormals.Length);
                            Assert.That(written, Is.EqualTo(initialParticleCount));
                            Assert.That(normalWritten, Is.EqualTo(initialParticleCount));
                            mesh.SetVertices(
                                unityPositions,
                                0,
                                written,
                                uploadFlags);
                            mesh.SetNormals(
                                unityNormals,
                                0,
                                written,
                                uploadFlags);
                        }

                        double[] samples = new double[sampleFrames];
                        Stopwatch timer = new Stopwatch();
                        long allocatedBefore = GC.GetAllocatedBytesForCurrentThread();
                        for (int frame = 0; frame < sampleFrames; ++frame)
                        {
                            timer.Restart();
                            world.Step(FrameDeltaTime);
                            int written = world.GetRenderVertexPositions(
                                positionBuffer,
                                unityPositions.Length);
                            int normalWritten = world.GetRenderVertexNormals(
                                normalBuffer,
                                unityNormals.Length);
                            if (written != initialParticleCount ||
                                normalWritten != initialParticleCount)
                            {
                                throw new InvalidOperationException(
                                    "Render output count changed in the fixed workload.");
                            }
                            mesh.SetVertices(
                                unityPositions,
                                0,
                                written,
                                uploadFlags);
                            mesh.SetNormals(
                                unityNormals,
                                0,
                                written,
                                uploadFlags);
                            timer.Stop();
                            samples[frame] = timer.Elapsed.TotalMilliseconds;
                        }
                        long managedAllocationBytes =
                            GC.GetAllocatedBytesForCurrentThread() - allocatedBefore;

                        Array.Sort(samples);
                        double median =
                            (samples[sampleFrames / 2 - 1] + samples[sampleFrames / 2]) *
                            0.5;
                        double p95 = samples[(sampleFrames * 95 + 99) / 100 - 1];
                        bool gatePassed =
                            median <= medianGateMilliseconds &&
                            p95 <= p95GateMilliseconds &&
                            managedAllocationBytes == 0;
                        string performanceJson = string.Format(
                            CultureInfo.InvariantCulture,
                            "APX_UNITY_PERF_JSON {{\"schema_version\":1," +
                            "\"benchmark\":\"phase1_dual_mesh_100k\"," +
                            "\"initial_particles\":{0},\"split_particles\":{1}," +
                            "\"active_particles\":{2},\"cloth_constraints\":{3}," +
                            "\"render_bindings\":{4},\"render_triangles\":{5}," +
                            "\"cut_event_ms\":{6:F4}," +
                            "\"substeps_per_frame\":2,\"solver_iterations\":2," +
                            "\"collision\":true,\"warmup_frames\":{7}," +
                            "\"sample_frames\":{8}," +
                            "\"timing_scope\":\"step+direct_nativearray_readback+mesh_upload\"," +
                            "\"median_ms\":{9:F4},\"p95_ms\":{10:F4}," +
                            "\"managed_alloc_bytes\":{11}," +
                            "\"median_gate_ms\":{12:F4},\"p95_gate_ms\":{13:F4}," +
                            "\"gate_pass\":{14}}}",
                            initialParticleCount,
                            rows,
                            initialParticleCount + rows,
                            expectedConstraintCount,
                            initialParticleCount,
                            expectedRenderTriangleCount,
                            cutTimer.Elapsed.TotalMilliseconds,
                            warmupFrames,
                            sampleFrames,
                            median,
                            p95,
                            managedAllocationBytes,
                            medianGateMilliseconds,
                            p95GateMilliseconds,
                            gatePassed ? "true" : "false");
                        TestContext.Progress.WriteLine(performanceJson);
                        UnityEngine.Debug.Log(performanceJson);
                        Assert.That(
                            median,
                            Is.LessThanOrEqualTo(medianGateMilliseconds));
                        Assert.That(p95, Is.LessThanOrEqualTo(p95GateMilliseconds));
                        Assert.That(managedAllocationBytes, Is.EqualTo(0L));
                        AssertFinite(renderPositions[0]);
                        AssertFinite(renderPositions[initialParticleCount / 2]);
                        AssertFinite(renderPositions[initialParticleCount - 1]);
                        AssertFinite(renderNormals[0]);
                        AssertFinite(renderNormals[initialParticleCount / 2]);
                        AssertFinite(renderNormals[initialParticleCount - 1]);
                    }
                    finally
                    {
                        UnityEngine.Object.DestroyImmediate(mesh);
                    }
                }
                finally
                {
                    unityPositions.Dispose();
                    unityNormals.Dispose();
                }
            }
        }

        [UnityTest]
        public IEnumerator PhaseOneDemoSceneRunsScriptedCutAndAdvancesPastThirtyTicks()
        {
            EditorSceneManager.OpenScene(
                "Assets/Scenes/PhaseOneDemo.unity",
                OpenSceneMode.Single);
            GameObject editModeCloth = GameObject.Find("PhaseOne_100k_Cloth");
            Assert.That(editModeCloth, Is.Not.Null);
            Assert.That(
                editModeCloth.GetComponent("PhaseOneDemoController"),
                Is.Not.Null);
            Assert.That(
                GameObject.Find("FixedTick_Blade").GetComponent("ApxBladeInteractor"),
                Is.Not.Null);
            Assert.That(
                GameObject.Find("PreciseCut_Preview").GetComponent("ObjCut"),
                Is.Not.Null);

            yield return new EnterPlayMode();
            GameObject cloth = GameObject.Find("PhaseOne_100k_Cloth");
            Component controller = cloth.GetComponent("PhaseOneDemoController");
            Assert.That(controller, Is.Not.Null);
            Behaviour controllerBehaviour = (Behaviour)controller;
            controllerBehaviour.enabled = false;
            MethodInfo fixedUpdate = controller
                .GetType()
                .GetMethod(
                    "FixedUpdate",
                    BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(fixedUpdate, Is.Not.Null);
            for (int update = 0; update < 31; ++update)
            {
                fixedUpdate.Invoke(controller, null);
            }

            ulong finalTick = (ulong)controller
                .GetType()
                .GetProperty("FixedTick")
                .GetValue(controller);
            uint particleCount = (uint)controller
                .GetType()
                .GetProperty("ParticleCount")
                .GetValue(controller);
            ulong stateHash = (ulong)controller
                .GetType()
                .GetProperty("StateHash")
                .GetValue(controller);
            Component previewCut = GameObject
                .Find("PreciseCut_Preview")
                .GetComponent("ObjCut");
            uint cutTriangles = (uint)previewCut
                .GetType()
                .GetField("lastCutTriangleCount")
                .GetValue(previewCut);
            Assert.That(finalTick, Is.GreaterThanOrEqualTo(31U));
            Assert.That(particleCount, Is.EqualTo(100250U));
            Assert.That(stateHash, Is.Not.EqualTo(0U));
            Assert.That(cutTriangles, Is.GreaterThan(0U));
            yield return new ExitPlayMode();
        }

        private static ReplayOutcome RunReplayScene(ApxBackendKind backend)
        {
            PhaseOneClothWorkload workload = PhaseOneClothWorkload.Create(
                8,
                6,
                0.2F,
                1.0e-5F,
                2.0e-5F,
                100.0F);
            PhaseOneReplayTimeline timeline = new PhaseOneReplayTimeline(
                new[]
                {
                    new PhaseOneReplayCommand(
                        0U,
                        1U,
                        workload.CreateVerticalCut(4, 0.01F)),
                });
            PhaseOneReplayCursor cursor = new PhaseOneReplayCursor(timeline);
            using (NativeWorld world = NativeWorld.Create(
                ApxWorldDesc.Create(
                    backend,
                    FixedTimeStep,
                    2U,
                    new ApxVec3(0.0F, -0.25F, 0.0F),
                    0.04F,
                    (uint)workload.Particles.Length)))
            {
                world.AddParticles(workload.Particles);
                world.AddClothDistanceConstraints(workload.ClothConstraints);
                world.SetRenderVertexBindings(workload.RenderBindings);
                world.SetRenderTriangles(workload.TriangleIndices);
                world.Step(FixedTimeStep * 0.5F);

                ApxCutResult cut = default;
                ApxVec3[] scratch = new ApxVec3[workload.Particles.Length * 2];
                ApxVec3[] stepOne = null;
                ulong[] hashes = new ulong[60];
                ulong hash = PhaseOneReplayHash.Begin();
                PhaseOneReplayHash.Append(ref hash, timeline.ContentHash);
                for (ulong tick = 1; tick <= 60; ++tick)
                {
                    while (cursor.TryDequeue(tick, out PhaseOneReplayCommand command))
                    {
                        cut = world.Cut(command.Query);
                        PhaseOneReplayHash.Append(ref hash, command);
                        PhaseOneReplayHash.Append(ref hash, cut);
                    }

                    world.Step(FrameDeltaTime);
                    int count = world.ReadPositionSnapshot(scratch);
                    PhaseOneReplayHash.Append(ref hash, scratch, count);
                    uint[] broken = world.GetBrokenClothDistanceConstraintIds();
                    PhaseOneReplayHash.Append(ref hash, (uint)broken.Length);
                    for (int index = 0; index < broken.Length; ++index)
                    {
                        PhaseOneReplayHash.Append(ref hash, broken[index]);
                    }
                    hashes[checked((int)tick - 1)] = hash;
                    if (tick == 1)
                    {
                        stepOne = CopyPositions(scratch, count);
                    }
                }

                int finalCount = checked((int)world.ParticleCount);
                return new ReplayOutcome
                {
                    FrameHashes = hashes,
                    CutResult = cut,
                    BrokenConstraintIds = world.GetBrokenClothDistanceConstraintIds(),
                    ParticleCount = world.ParticleCount,
                    StepOnePositions = stepOne,
                    FinalPositions = CopyPositions(scratch, finalCount),
                };
            }
        }

        private static ApxVec3[] CopyPositions(ApxVec3[] source, int count)
        {
            ApxVec3[] copy = new ApxVec3[count];
            Array.Copy(source, copy, count);
            return copy;
        }

        private static void CopyRenderData(
            ApxVec3[] positions,
            ApxVec3[] normals,
            NativeArray<Vector3> unityPositions,
            NativeArray<Vector3> unityNormals,
            int count)
        {
            for (int index = 0; index < count; ++index)
            {
                ApxVec3 position = positions[index];
                ApxVec3 normal = normals[index];
                unityPositions[index] = new Vector3(position.X, position.Y, position.Z);
                unityNormals[index] = new Vector3(normal.X, normal.Y, normal.Z);
            }
        }

        private static void AssertCutResultsEqual(ApxCutResult left, ApxCutResult right)
        {
            Assert.That(right.CutId, Is.EqualTo(left.CutId));
            Assert.That(right.CutConstraintCount, Is.EqualTo(left.CutConstraintCount));
            Assert.That(right.SplitParticleCount, Is.EqualTo(left.SplitParticleCount));
            Assert.That(right.FirstSplitParticleId, Is.EqualTo(left.FirstSplitParticleId));
        }

        private static void AssertPositionsNear(
            ApxVec3[] expected,
            ApxVec3[] actual,
            float tolerance)
        {
            Assert.That(actual, Has.Length.EqualTo(expected.Length));
            for (int index = 0; index < expected.Length; ++index)
            {
                Assert.That(
                    actual[index].X,
                    Is.EqualTo(expected[index].X).Within(
                        tolerance + tolerance * Math.Max(
                            Math.Abs(expected[index].X),
                            Math.Abs(actual[index].X))));
                Assert.That(
                    actual[index].Y,
                    Is.EqualTo(expected[index].Y).Within(
                        tolerance + tolerance * Math.Max(
                            Math.Abs(expected[index].Y),
                            Math.Abs(actual[index].Y))));
                Assert.That(
                    actual[index].Z,
                    Is.EqualTo(expected[index].Z).Within(
                        tolerance + tolerance * Math.Max(
                            Math.Abs(expected[index].Z),
                            Math.Abs(actual[index].Z))));
            }
        }

        private static void AssertFinite(ApxVec3 value)
        {
            Assert.That(float.IsNaN(value.X) || float.IsInfinity(value.X), Is.False);
            Assert.That(float.IsNaN(value.Y) || float.IsInfinity(value.Y), Is.False);
            Assert.That(float.IsNaN(value.Z) || float.IsInfinity(value.Z), Is.False);
        }

        private sealed class ReplayOutcome
        {
            public ulong[] FrameHashes;
            public ApxCutResult CutResult;
            public uint[] BrokenConstraintIds;
            public uint ParticleCount;
            public ApxVec3[] StepOnePositions;
            public ApxVec3[] FinalPositions;
        }
    }
}
