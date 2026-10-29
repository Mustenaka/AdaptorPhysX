using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tests
{
    /// <summary>
    /// Scoring of a procedure from the events of the bodies (TODO MED-13). Component tests run in a preview scene, as in
    /// <see cref="InteractionTests"/>.
    /// </summary>
    public sealed class ScoringTests
    {
        private const float Dt = 1.0f / 60.0f;

        private readonly List<Object> _objects = new List<Object>();
        private readonly List<ApxSoftBody> _bodies = new List<ApxSoftBody>();
        private Scene _scene;

        [TearDown]
        public void TearDown()
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
        }

        /// <summary>
        /// A stroke of ten sweeps cuts a 5 cm long, 15 mm deep incision into the top face of a block (cell 5 mm): when the
        /// stroke ends the scorer reports one incision with the area of its cut surface, its centre on the surface, and its
        /// length and depth less than a cell short (they span the severed lattice edges, which lie on the cell grid).
        /// </summary>
        [Test]
        public void ScorerMeasuresAnIncision()
        {
            ApxSoftBody block = CreateBody("Block", new Vector3(0.1f, 0.04f, 0.06f), 20);
            block.Initialize();
            ApxScorer scorer = CreateScorer(block);
            var made = new List<ApxIncision>();
            scorer.IncisionMade += made.Add;
            // The blade in the plane z = 0.5 mm, from 10 mm above the top face (y = 20 mm) to 15 mm below it.
            for (int i = 0; i < 10; i++)
            {
                float x0 = -0.025f + 0.005f * i, x1 = x0 + 0.005f;
                block.Cut(new Vector3(x0, 0.03f, 0.0005f), new Vector3(x0, 0.005f, 0.0005f), new Vector3(x1, 0.03f, 0.0005f),
                    new Vector3(x1, 0.005f, 0.0005f), 3);
                Step(block, 1);
            }
            Assert.AreEqual(0, made.Count, "the stroke has not ended");
            block.EndStroke(3);
            Step(block, 1);
            Assert.AreEqual(1, made.Count);
            ApxIncision incision = made[0];
            float h = block.Simulation.CellSize;
            TestContext.WriteLine($"incision: {incision.SeveredEdges} edges, length {incision.Length * 1000:0.0} mm, depth " +
                                  $"{incision.Depth * 1000:0.0} mm, area {incision.Area * 1e6f:0} mm^2, centre {incision.Center * 1000} mm, " +
                                  $"from {incision.Start * 1000} to {incision.End * 1000} mm");
            Assert.AreEqual(3, incision.Stroke);
            Assert.That(incision.Length, Is.InRange(0.05f - 1.01f * h, 0.05f + 0.01f * h));
            Assert.That(incision.Depth, Is.InRange(0.015f - 1.01f * h, 0.015f + 0.01f * h));
            Assert.AreEqual(0.05f * 0.015f, incision.Area, 0.2f * 0.05f * 0.015f);
            Assert.AreEqual(0.0f, incision.Center.x, 0.5f * h);
            Assert.AreEqual(0.02f, incision.Center.y, 0.5f * h);
            Assert.AreEqual(0.0005f, incision.Center.z, 0.1f * h);
            Assert.Greater(math.abs(incision.End.x - incision.Start.x), 0.9f * incision.Length, "the incision runs along x");
            ApxScoreReport report = scorer.Report();
            Assert.AreEqual(1, report.Incisions.Count);
            Assert.AreEqual(1, report.Pieces);
            Assert.AreEqual(incision.Area, report.CutArea, 1e-9f);
        }

        /// <summary>A cut through a 8 x 4 x 6 cm block: the cut area is its 4 x 6 cm cross-section.</summary>
        [Test]
        public void CutAreaOfAFullCutIsTheCrossSection()
        {
            ApxSoftBody block = CreateBody("Block", new Vector3(0.08f, 0.04f, 0.06f), 16);
            block.Initialize();
            ApxScorer scorer = CreateScorer(block);
            var made = new List<ApxIncision>();
            scorer.IncisionMade += made.Add;
            block.Cut(new Vector3(0.0005f, 0.5f, -0.2f), new Vector3(0.0005f, -0.5f, -0.2f), new Vector3(0.0005f, 0.5f, 0.2f),
                new Vector3(0.0005f, -0.5f, 0.2f));
            Step(block, 2);
            ApxScoreReport report = scorer.Report();
            TestContext.WriteLine($"cut area {report.CutArea * 1e6f:0} mm^2, pieces {report.Pieces}, incisions {made.Count}");
            Assert.AreEqual(1, made.Count, "a cut of stroke 0 is an incision of its own");
            Assert.AreEqual(2, report.Pieces);
            Assert.AreEqual(0.04f * 0.06f, report.CutArea, 0.05f * 0.04f * 0.06f);
        }

        /// <summary>
        /// A ball pressed 6 mm into a slab held at its ends and lifted again: with a limit below the largest force on the ball
        /// one force event is raised and reported, with its peak and duration; with a limit above it none.
        /// </summary>
        [Test]
        public void ScorerReportsTheTimeAboveTheForceLimit()
        {
            float Press(float limit, out ApxScoreReport report, out int raised)
            {
                ApxSoftBody slab = CreateBody("Slab", new Vector3(0.14f, 0.04f, 0.06f), 14);
                slab.pinColliders = new[]
                {
                    Trigger(new Vector3(-0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                    Trigger(new Vector3(0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                };
                GameObject ball = InScene(GameObject.CreatePrimitive(PrimitiveType.Sphere));
                ball.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
                ball.transform.position = new Vector3(0.0f, 0.04f, 0.0f);
                Physics.SyncTransforms();
                slab.Initialize();
                ApxScorer scorer = CreateScorer(slab);
                scorer.forceLimit = limit;
                scorer.instruments = new[] { ball.GetComponent<Collider>() };
                int count = 0;
                scorer.ForceExceeded += force => count++;
                float peak = 0.0f;
                // The ball (radius 10 mm) from 10 mm above the top face (y = 20 mm) down to 6 mm into it, held, and back.
                for (int f = 0; f < 120; f++)
                {
                    float depth = f < 40 ? f / 40.0f : f < 60 ? 1.0f : math.saturate((100 - f) / 40.0f);
                    ball.transform.position = new Vector3(0.0f, 0.04f - 0.016f * depth, 0.0f);
                    Physics.SyncTransforms();
                    Step(slab, 1);
                    if (slab.TryGetColliderWrench(ball.GetComponent<Collider>(), out ApxWrench wrench))
                    {
                        peak = math.max(peak, math.length(wrench.Force));
                    }
                }
                report = scorer.Report();
                raised = count;
                Object.DestroyImmediate(ball);
                return peak;
            }

            float largest = Press(1000.0f, out ApxScoreReport none, out int raisedNone);
            float limit = 0.5f * largest;
            Press(limit, out ApxScoreReport report, out int raised);
            TestContext.WriteLine($"largest force {largest:0.000} N, limit {limit:0.000} N: {raised} raised, events " +
                                  $"{string.Join(", ", report.ForceEvents.ConvertAll(e => $"{e.Source} from {e.Start:0.000} s for {e.Duration:0.000} s, peak {e.Peak:0.000} N"))}");
            Assert.AreEqual(0, raisedNone);
            Assert.AreEqual(0, none.ForceEvents.Count);
            Assert.AreEqual(1, raised);
            Assert.AreEqual(1, report.ForceEvents.Count);
            ApxForceEvent force = report.ForceEvents[0];
            Assert.AreEqual("Sphere", force.Source);
            Assert.AreEqual(largest, force.Peak, 1e-4f);
            Assert.Greater(force.Duration, 0.3f, "held for 1/3 s");
            Assert.AreEqual(force.Duration, report.OverForceTime, 1e-6f);
        }

        private ApxScorer CreateScorer(ApxSoftBody body)
        {
            // Edit mode calls no OnEnable: the scorer observes the body when told to.
            var scorer = InScene(new GameObject("Scorer")).AddComponent<ApxScorer>();
            scorer.Observe(body);
            return scorer;
        }

        private static void Step(ApxSoftBody body, int frames)
        {
            for (int f = 0; f < frames; f++)
            {
                body.Simulate(Dt);
                body.UpdateRender();
            }
        }

        private ApxSoftBody CreateBody(string name, Vector3 size, int resolution)
        {
            GameObject go = InScene(new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer)));
            Mesh mesh = MeshUtil.CreateBox(size, 4);
            _objects.Add(mesh);
            go.GetComponent<MeshFilter>().sharedMesh = mesh;
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = resolution;
            body.useGravity = false;
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
