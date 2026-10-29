using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tests
{
    /// <summary>
    /// Recording a procedure and replaying it (TODO MED-14). Component tests run in a preview scene, as in
    /// <see cref="InteractionTests"/>; each run builds the scene afresh.
    /// </summary>
    public sealed class ReplayTests
    {
        private const float Dt = 1.0f / 60.0f;
        private const int Steps = 180;
        private const float BallRadius = 0.006f;

        private readonly List<Object> _objects = new List<Object>();
        private readonly List<ApxSoftBody> _bodies = new List<ApxSoftBody>();
        private Scene _scene;

        [TearDown]
        public void TearDown() => Clear();

        /// <summary>
        /// A strip held at one end (10 x 1 x 4 cm, cell 2.5 mm, no gravity) goes through a procedure of 180 steps: a grasper
        /// closes on its free end and lifts it, a ball presses it, a forceps grab pulls it up and lets go, a scalpel stroke
        /// cuts it through, a stitch across the cut is drawn, a cut is made directly and the grasper opens. The recording,
        /// through JSON, is replayed in the scene built afresh: every check of the node positions agrees, and the positions
        /// after the last step are the recorded ones bit for bit. The recording holds the poses of the instruments and the
        /// operations made directly, but no cut of the scalpel and no attachment of the grasper: those follow from the poses.
        /// </summary>
        [Test]
        public void ReplayRepeatsTheRecordedProcedureBitForBit()
        {
            Setup recorded = Build(BallRadius);
            recorded.Recorder.StartRecording();
            bool held = false;
            for (int tick = 0; tick < Steps; tick++)
            {
                Act(recorded, tick);
                Frame(recorded);
                held |= recorded.Grasper.Held != null;
            }
            ApxRecording recording = recorded.Recorder.StopRecording();
            float3[] positions = recorded.Strip.Simulation.Positions.ToArray();
            int pieces = recorded.Strip.Simulation.CountPieces();
            string json = recording.ToJson();
            var kinds = new Dictionary<ApxOperationKind, int>();
            foreach (ApxOperation op in recording.Operations)
            {
                kinds[op.Kind] = kinds.TryGetValue(op.Kind, out int count) ? count + 1 : 1;
            }
            TestContext.WriteLine($"recording: {recording.Steps} steps, {recording.Poses.Count} poses, {recording.Settings.Count} " +
                                  $"settings, {recording.Operations.Count} operations ({string.Join(", ", kinds)}), " +
                                  $"{recording.Checks.Count} checks, {json.Length / 1024} KB of JSON; pieces {pieces}, grasper held {held}");
            Assert.IsTrue(held, "the grasper held the strip");
            Assert.GreaterOrEqual(pieces, 2, "the scalpel cut the strip through");
            Assert.AreEqual(1, kinds[ApxOperationKind.Cut], "only the direct cut is recorded, not the scalpel's");
            Assert.IsFalse(kinds.ContainsKey(ApxOperationKind.EndStroke), "the scalpel's stroke end is not recorded");
            Assert.IsFalse(kinds.ContainsKey(ApxOperationKind.AddAttachment), "the grasper's attachment is not recorded");
            Assert.AreEqual(2, kinds[ApxOperationKind.AddMaterialPoint], "the grasper's material points are not recorded");
            // Operations made by other operations (BeginGrab lets go of the grab first, a cut wakes the body) are not recorded.
            Assert.AreEqual(1, kinds[ApxOperationKind.BeginGrab]);
            Assert.AreEqual(1, kinds[ApxOperationKind.EndGrab]);
            Assert.AreEqual(1, kinds[ApxOperationKind.WakeUp]);
            Assert.AreEqual(1, kinds[ApxOperationKind.ResetSweep]);
            Assert.AreEqual(30, kinds[ApxOperationKind.MoveGrab]);
            Clear();

            Setup replayed = Build(BallRadius);
            replayed.Recorder.Replay(ApxRecording.FromJson(json));
            int frames = 0;
            bool replayHeld = false;
            while (Frame(replayed))
            {
                frames++;
                replayHeld |= replayed.Grasper.Held != null;
            }
            TestContext.WriteLine($"replay: {frames} steps, {replayed.Recorder.ChecksCompared} checks compared, diverged at " +
                                  $"{replayed.Recorder.DivergedTick}");
            Assert.AreEqual(Steps, frames);
            Assert.IsTrue(replayHeld);
            Assert.AreEqual(Steps / 20, replayed.Recorder.ChecksCompared, "checks before steps 20, 40, ..., 160 and after the last");
            Assert.IsFalse(replayed.Recorder.Diverged);
            float3[] replayedPositions = replayed.Strip.Simulation.Positions.ToArray();
            Assert.AreEqual(positions.Length, replayedPositions.Length);
            for (int n = 0; n < positions.Length; n++)
            {
                Assert.IsTrue(math.all(math.asuint(positions[n]) == math.asuint(replayedPositions[n])), $"node {n} differs");
            }
        }

        /// <summary>
        /// The same recording replayed with a ball 0.2 mm larger: the checks agree until the ball reaches the strip (step 40),
        /// and the replay reports the first check after that as the step where it diverged.
        /// </summary>
        [Test]
        public void ReplayInAChangedSceneReportsWhereItDiverged()
        {
            Setup recorded = Build(BallRadius);
            recorded.Recorder.StartRecording();
            for (int tick = 0; tick < Steps; tick++)
            {
                Act(recorded, tick);
                Frame(recorded);
            }
            string json = recorded.Recorder.StopRecording().ToJson();
            Clear();

            Setup replayed = Build(BallRadius + 0.0002f);
            replayed.Recorder.Replay(ApxRecording.FromJson(json));
            while (Frame(replayed))
            {
            }
            TestContext.WriteLine($"diverged at step {replayed.Recorder.DivergedTick}");
            Assert.IsTrue(replayed.Recorder.Diverged);
            Assert.That(replayed.Recorder.DivergedTick, Is.InRange(40, Steps), "the checks agree before the ball touches");
        }

        private sealed class Setup
        {
            public ApxSoftBody Strip;
            public ApxScalpel Scalpel;
            public ApxGrasper Grasper;
            public Transform Ball;
            public ApxRecorder Recorder;
            public int PointA;
            public int PointB;
            public int Stitch;
            public Vector3 GrabStart;
        }

        private Setup Build(float ballRadius)
        {
            var setup = new Setup();
            setup.Strip = CreateBody("Strip", new Vector3(0.1f, 0.01f, 0.04f), new Vector3(0.0f, 0.1f, 0.0f), 40);
            setup.Strip.useGravity = false;
            setup.Strip.pinColliders = new[] { Trigger(new Vector3(-0.05f, 0.1f, 0.0f), new Vector3(0.02f, 0.03f, 0.06f)) };
            setup.Scalpel = InScene(new GameObject("Scalpel")).AddComponent<ApxScalpel>();
            setup.Scalpel.cutting = false;
            setup.Scalpel.transform.position = new Vector3(0.005f, 0.14f, -0.04f);
            setup.Grasper = InScene(new GameObject("Grasper")).AddComponent<ApxGrasper>();
            setup.Grasper.transform.position = new Vector3(0.04f, 0.1f, 0.0f);
            setup.Grasper.gap = 0.014f;
            GameObject ball = InScene(GameObject.CreatePrimitive(PrimitiveType.Sphere));
            ball.name = "Ball";
            ball.transform.localScale = Vector3.one * (2.0f * ballRadius);
            ball.transform.position = new Vector3(-0.025f, 0.125f, 0.0f);
            setup.Ball = ball.transform;
            Physics.SyncTransforms();
            setup.Strip.Initialize();
            setup.Recorder = InScene(new GameObject("Recorder")).AddComponent<ApxRecorder>();
            setup.Recorder.bodies = new[] { setup.Strip };
            setup.Recorder.instruments = new[] { setup.Scalpel.transform, setup.Grasper.transform, setup.Ball };
            setup.Recorder.checkInterval = 20;
            return setup;
        }

        /// <summary>What the user does before step <paramref name="tick"/>: moves the instruments and operates on the strip.</summary>
        private static void Act(Setup s, int tick)
        {
            ApxSoftBody strip = s.Strip;
            if (tick == 0)
            {
                // Points across where the scalpel will cut (x = 5 mm), for the stitch.
                s.PointA = strip.AddMaterialPoint(new Vector3(-0.004f, 0.1f, 0.0f));
                s.PointB = strip.AddMaterialPoint(new Vector3(0.014f, 0.1f, 0.0f));
            }
            // The grasper closes on the free end within 20 steps, lifts it by 2 cm and opens at step 160.
            s.Grasper.gap = tick < 160 ? Mathf.Lerp(0.014f, 0.006f, tick / 20.0f) : 0.014f;
            s.Grasper.transform.position = new Vector3(0.04f, 0.1f + 0.02f * Mathf.Clamp01((tick - 30) / 60.0f), 0.0f);
            // The ball (radius 6 mm) comes down 3 mm into the strip by step 40, stays and goes back up by step 100.
            float press = tick < 70 ? Mathf.Clamp01((tick - 10) / 30.0f) : Mathf.Clamp01((100 - tick) / 30.0f);
            s.Ball.position = new Vector3(-0.025f, 0.125f - 0.017f * press, 0.0f);
            // A forceps grab pulls the strip up by 8 mm and lets go.
            if (tick == 50)
            {
                strip.BeginGrab(new Ray(new Vector3(-0.035f, 0.2f, 0.012f), Vector3.down), 0.004f, out s.GrabStart);
            }
            if (tick > 50 && tick <= 80)
            {
                strip.MoveGrab(s.GrabStart + new Vector3(0.0f, 0.008f * (tick - 50) / 30.0f, 0.0f));
            }
            if (tick == 85)
            {
                strip.EndGrab();
            }
            // The scalpel cuts across the strip at x = 5 mm.
            if (tick == 95)
            {
                s.Scalpel.edgeStart = Vector3.zero;
                s.Scalpel.edgeEnd = new Vector3(0.0f, -0.05f, 0.0f);
                s.Scalpel.cutting = true;
                s.Scalpel.ResetSweep();
            }
            if (tick >= 95 && tick <= 125)
            {
                s.Scalpel.transform.position = new Vector3(0.005f, 0.14f, Mathf.Lerp(-0.04f, 0.04f, (tick - 95) / 30.0f));
            }
            if (tick == 126)
            {
                s.Scalpel.cutting = false;
            }
            if (tick == 130)
            {
                s.Stitch = strip.AddStitch(s.PointA, s.PointB, 0.02f);
            }
            if (tick == 140)
            {
                strip.Cut(new Vector3(-0.03f, 0.12f, -0.02f), new Vector3(-0.03f, 0.09f, -0.02f), new Vector3(-0.03f, 0.12f, -0.01f),
                    new Vector3(-0.03f, 0.09f, -0.01f));
            }
            if (tick == 150)
            {
                strip.SetStitchTarget(s.Stitch, 0.004f);
            }
            if (tick == 155)
            {
                strip.WakeUp();
            }
        }

        /// <summary>One fixed step in the order of FixedUpdate: recorder, instruments, body; false once the replay has ended.</summary>
        private static bool Frame(Setup s)
        {
            s.Recorder.Step();
            if (!s.Recorder.IsRecording && !s.Recorder.IsReplaying)
            {
                return false;
            }
            s.Scalpel.Sweep();
            s.Grasper.Step();
            s.Strip.Simulate(Dt);
            s.Strip.UpdateRender();
            return true;
        }

        private void Clear()
        {
            foreach (ApxSoftBody body in _bodies)
            {
                body.Release();
            }
            _bodies.Clear();
            foreach (Object o in _objects)
            {
                if (o != null)
                {
                    Object.DestroyImmediate(o);
                }
            }
            _objects.Clear();
            if (_scene.IsValid())
            {
                EditorSceneManager.ClosePreviewScene(_scene);
            }
            _scene = default;
        }

        private ApxSoftBody CreateBody(string name, Vector3 size, Vector3 position, int resolution)
        {
            GameObject go = InScene(new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer)));
            go.transform.position = position;
            Mesh mesh = MeshUtil.CreateBox(size, 4);
            _objects.Add(mesh);
            go.GetComponent<MeshFilter>().sharedMesh = mesh;
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = resolution;
            _bodies.Add(body);
            return body;
        }

        private Collider Trigger(Vector3 position, Vector3 size)
        {
            GameObject go = InScene(new GameObject("Pin"));
            go.transform.position = position;
            var box = go.AddComponent<BoxCollider>();
            box.size = size;
            box.isTrigger = true;
            return box;
        }

        /// <summary>Moves a new object into the test's preview scene.</summary>
        private GameObject InScene(GameObject go)
        {
            if (!_scene.IsValid())
            {
                _scene = EditorSceneManager.NewPreviewScene();
            }
            SceneManager.MoveGameObjectToScene(go, _scene);
            _objects.Add(go);
            return go;
        }
    }
}
