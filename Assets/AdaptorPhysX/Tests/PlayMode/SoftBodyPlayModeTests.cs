using System.Collections;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

namespace AdaptorPhysX.Tests
{
    /// <summary>Runs the component path (Start / FixedUpdate / LateUpdate) with a moving scalpel.</summary>
    public sealed class SoftBodyPlayModeTests
    {
        [UnityTest]
        public IEnumerator ScalpelStrokeSplitsBodyOnTheFloor()
        {
            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            floor.transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);

            var bodyObject = new GameObject("Body", typeof(MeshFilter), typeof(MeshRenderer));
            bodyObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.04f, 0.08f), 8);
            bodyObject.transform.position = new Vector3(0.0f, 0.025f, 0.0f);
            var body = bodyObject.AddComponent<ApxSoftBody>();
            body.resolution = 12;
            body.substeps = 8;

            var scalpelObject = new GameObject("Scalpel");
            var scalpel = scalpelObject.AddComponent<ApxScalpel>();
            scalpel.edgeStart = Vector3.zero;
            scalpel.edgeEnd = new Vector3(0.0f, -0.08f, 0.0f);
            scalpel.cutting = false;
            scalpelObject.transform.position = new Vector3(0.001f, 0.07f, -0.08f);

            for (int i = 0; i < 20; i++)
            {
                yield return new WaitForFixedUpdate();
            }
            Assert.IsTrue(body.IsInitialized);
            Assert.AreEqual(1, body.Simulation.CountPieces());

            scalpel.cutting = true;
            for (int i = 0; i <= 30; i++)
            {
                scalpelObject.transform.position = new Vector3(0.001f, 0.07f, Mathf.Lerp(-0.08f, 0.08f, i / 30.0f));
                yield return new WaitForFixedUpdate();
            }
            scalpel.cutting = false;
            for (int i = 0; i < 30; i++)
            {
                yield return null;
            }

            Assert.AreEqual(2, body.Simulation.CountPieces());
            Assert.Greater(body.Surface.CutTriangleCount, 0);
            foreach (Vector3 v in body.Surface.Mesh.vertices)
            {
                Assert.IsFalse(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z));
            }

            Object.Destroy(scalpelObject);
            Object.Destroy(bodyObject);
            Object.Destroy(floor);
            yield return null;
        }

        [UnityTest]
        public IEnumerator RestingBodySleepsAndWakesWhenCut()
        {
            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            floor.transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);

            var bodyObject = new GameObject("Body", typeof(MeshFilter), typeof(MeshRenderer));
            bodyObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.04f, 0.08f), 8);
            bodyObject.transform.position = new Vector3(0.0f, 0.025f, 0.0f);
            var body = bodyObject.AddComponent<ApxSoftBody>();
            body.resolution = 10;
            body.substeps = 8;

            for (int i = 0; i < 300 && !(body.IsInitialized && body.IsSleeping); i++)
            {
                yield return new WaitForFixedUpdate();
            }
            Assert.IsTrue(body.IsSleeping, "a body resting on the floor goes to sleep");
            int steps = body.Simulation.StepCount;
            for (int i = 0; i < 10; i++)
            {
                yield return new WaitForFixedUpdate();
            }
            Assert.AreEqual(steps, body.Simulation.StepCount, "a sleeping body does not step");

            int severed = body.Cut(new Vector3(0.001f, 0.07f, -0.08f), new Vector3(0.001f, -0.01f, -0.08f),
                new Vector3(0.001f, 0.07f, 0.08f), new Vector3(0.001f, -0.01f, 0.08f));
            Assert.Greater(severed, 0);
            Assert.IsFalse(body.IsSleeping);
            for (int i = 0; i < 5; i++)
            {
                yield return new WaitForFixedUpdate();
            }
            Assert.Greater(body.Simulation.StepCount, steps);
            Assert.AreEqual(2, body.Simulation.CountPieces());

            Object.Destroy(bodyObject);
            Object.Destroy(floor);
            yield return null;
        }

        /// <summary>
        /// A scalpel stroke and a forceps grab recorded at three fixed steps per frame, of which the body steps in one
        /// (maxStepsPerFrame 1), and replayed at one fixed step per frame (TODO MED-14): the replay skips the steps the
        /// recording skipped, and the node positions agree with the recording at every check and after the last step.
        /// </summary>
        [UnityTest]
        public IEnumerator RecordingReplaysAtAnotherFrameRate()
        {
            try
            {
                Time.captureDeltaTime = 3.0f * Time.fixedDeltaTime;
                RecordedScene recorded = CreateRecordedScene();
                recorded.Recorder.StartRecording();
                for (int f = 0; f < 10; f++)
                {
                    yield return null;
                }
                recorded.Scalpel.cutting = true;
                for (int f = 0; f <= 20; f++)
                {
                    recorded.Scalpel.transform.position = new Vector3(0.001f, 0.07f, Mathf.Lerp(-0.08f, 0.08f, f / 20.0f));
                    yield return null;
                }
                recorded.Scalpel.cutting = false;
                recorded.Body.BeginGrab(new Ray(new Vector3(-0.03f, 0.2f, 0.0f), Vector3.down), 0.01f, out Vector3 start);
                for (int f = 1; f <= 15; f++)
                {
                    recorded.Body.MoveGrab(start + new Vector3(-0.001f * f, 0.001f * f, 0.0f));
                    yield return null;
                }
                recorded.Body.EndGrab();
                for (int f = 0; f < 10; f++)
                {
                    yield return null;
                }
                ApxRecording recording = recorded.Recorder.StopRecording();
                float3[] positions = recorded.Body.Simulation.Positions.ToArray();
                int pieces = recorded.Body.Simulation.CountPieces();
                recorded.Destroy();
                yield return null;

                Time.captureDeltaTime = Time.fixedDeltaTime;
                RecordedScene replayed = CreateRecordedScene();
                float3[] replayedPositions = null;
                replayed.Recorder.ReplayEnded += r =>
                {
                    replayed.Body.Simulation.CompleteStep();
                    replayedPositions = replayed.Body.Simulation.Positions.ToArray();
                };
                replayed.Recorder.Replay(recording);
                for (int f = 0; f < 3 * recording.Steps && replayed.Recorder.IsReplaying; f++)
                {
                    yield return null;
                }
                Debug.Log($"recording: {recording.Steps} steps, {recording.Skips.Count} skipped by the body, pieces {pieces}; replay: " +
                          $"{replayed.Recorder.ChecksCompared} checks compared, diverged at {replayed.Recorder.DivergedTick}");
                Assert.IsFalse(replayed.Recorder.IsReplaying, "the replay ended");
                Assert.Greater(recording.Skips.Count, recording.Steps / 2, "the body skipped two of three steps");
                Assert.AreEqual(2, pieces);
                Assert.IsFalse(replayed.Recorder.Diverged);
                Assert.Greater(replayed.Recorder.ChecksCompared, 2);
                Assert.IsNotNull(replayedPositions);
                Assert.AreEqual(positions.Length, replayedPositions.Length);
                for (int n = 0; n < positions.Length; n++)
                {
                    Assert.IsTrue(math.all(math.asuint(positions[n]) == math.asuint(replayedPositions[n])), $"node {n} differs");
                }
                replayed.Destroy();
                yield return null;
            }
            finally
            {
                Time.captureDeltaTime = 0.0f;
            }
        }

        private sealed class RecordedScene
        {
            public GameObject Floor;
            public ApxSoftBody Body;
            public ApxScalpel Scalpel;
            public ApxRecorder Recorder;

            public void Destroy()
            {
                Object.Destroy(Floor);
                Object.Destroy(Body.gameObject);
                Object.Destroy(Scalpel.gameObject);
                Object.Destroy(Recorder.gameObject);
            }
        }

        /// <summary>A block on the floor, a scalpel above it and a recorder of both.</summary>
        private static RecordedScene CreateRecordedScene()
        {
            var scene = new RecordedScene { Floor = GameObject.CreatePrimitive(PrimitiveType.Cube) };
            scene.Floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            scene.Floor.transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);
            var bodyObject = new GameObject("Body", typeof(MeshFilter), typeof(MeshRenderer));
            bodyObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.04f, 0.08f), 8);
            bodyObject.transform.position = new Vector3(0.0f, 0.025f, 0.0f);
            scene.Body = bodyObject.AddComponent<ApxSoftBody>();
            scene.Body.resolution = 10;
            scene.Body.substeps = 8;
            scene.Scalpel = new GameObject("Scalpel").AddComponent<ApxScalpel>();
            scene.Scalpel.edgeStart = Vector3.zero;
            scene.Scalpel.edgeEnd = new Vector3(0.0f, -0.08f, 0.0f);
            scene.Scalpel.cutting = false;
            scene.Scalpel.transform.position = new Vector3(0.001f, 0.07f, -0.08f);
            scene.Recorder = new GameObject("Recorder").AddComponent<ApxRecorder>();
            scene.Recorder.bodies = new[] { scene.Body };
            scene.Recorder.instruments = new[] { scene.Scalpel.transform };
            scene.Recorder.checkInterval = 30;
            return scene;
        }
    }
}
