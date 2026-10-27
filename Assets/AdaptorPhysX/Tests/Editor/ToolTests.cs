using System.Collections.Generic;
using System.Linq;
using AdaptorPhysX.Tools;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tests
{
    /// <summary>
    /// Interactive tools and the debug view (TODO ENG-5): blade pose, orbit camera math, the scalpel,
    /// forceps and measuring tools driven by scripted pointer input, and the lattice and cut layers.
    /// Bodies live in a preview scene, as in <see cref="InteractionTests"/>.
    /// </summary>
    public sealed class ToolTests
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

        [Test]
        public void BladePoseKeepsTheTipAtItsDepthBelowTheSurface()
        {
            var point = new Vector3(0.1f, 0.05f, -0.2f);
            foreach (float heading in new[] { 0.0f, 37.0f, -120.0f })
            foreach (float tilt in new[] { 0.0f, 25.0f, -40.0f })
            foreach (float rake in new[] { 0.0f, 30.0f, -20.0f })
            {
                var pose = new ApxBladePose { heading = heading, tilt = tilt, rake = rake, depth = 0.012f, clearance = 0.008f };
                pose.GetEdge(point, out Vector3 heel, out Vector3 tip);
                Assert.AreEqual(point.y - 0.012f, tip.y, 1e-6f, $"tip depth at {heading}/{tilt}/{rake}");
                Assert.AreEqual(point.y + 0.008f, heel.y, 1e-6f, $"heel height at {heading}/{tilt}/{rake}");
                Assert.Less(Vector3.Cross(tip - point, heel - point).magnitude, 1e-7f, "the edge passes through the surface point");
            }
            float tan30 = Mathf.Tan(30.0f * Mathf.Deg2Rad);
            // Heading +z: tilt moves the tip to the right (+x), rake moves it ahead (+z).
            new ApxBladePose { tilt = 30.0f, depth = 0.01f }.GetEdge(Vector3.zero, out _, out Vector3 tilted);
            Assert.AreEqual(0.01f * tan30, tilted.x, 1e-6f);
            Assert.AreEqual(0.0f, tilted.z, 1e-6f);
            new ApxBladePose { rake = 30.0f, depth = 0.01f }.GetEdge(Vector3.zero, out _, out Vector3 raked);
            Assert.AreEqual(0.0f, raked.x, 1e-6f);
            Assert.AreEqual(0.01f * tan30, raked.z, 1e-6f);
            // Heading +x: the right of the cutting direction is -z.
            new ApxBladePose { heading = 90.0f, tilt = 30.0f, depth = 0.01f }.GetEdge(Vector3.zero, out _, out Vector3 turned);
            Assert.AreEqual(-0.01f * tan30, turned.z, 1e-6f);
            Assert.AreEqual(0.0f, turned.x, 1e-6f);
        }

        [Test]
        public void OrbitZoomKeepsTheTargetInPlaceAndPanFollowsThePointer()
        {
            var orbit = new ApxOrbit { Pivot = new Vector3(0.1f, 0.03f, -0.05f), Yaw = 30.0f, Pitch = 40.0f, Distance = 0.5f };
            var target = new Vector3(0.14f, 0.0f, 0.02f);
            Vector3 before = Quaternion.Inverse(orbit.Rotation) * (target - orbit.Position).normalized;
            foreach (float factor in new[] { 0.5f, 1.7f })
            {
                ApxOrbit zoomed = orbit;
                zoomed.Zoom(factor, target, 0.01f, 10.0f);
                Vector3 after = Quaternion.Inverse(zoomed.Rotation) * (target - zoomed.Position).normalized;
                Assert.Less((after - before).magnitude, 1e-5f, $"direction to the target after zooming by {factor}");
                Assert.AreEqual(orbit.Distance * factor, zoomed.Distance, 1e-6f);
            }

            const float fov = 60.0f;
            const float height = 720.0f;
            Vector3 p = orbit.Pivot + orbit.Rotation * new Vector3(0.03f, -0.02f, 0.0f);
            Vector2 start = Pixel(orbit, p, fov, height);
            ApxOrbit panned = orbit;
            panned.Pan(new Vector2(40.0f, -25.0f), fov, height);
            Vector2 end = Pixel(panned, p, fov, height);
            Assert.AreEqual(start.x + 40.0f, end.x, 0.01f);
            Assert.AreEqual(start.y - 25.0f, end.y, 0.01f);
        }

        [Test]
        public void ScalpelToolStartsAlongTheStrokeAndCutsWithItsLean()
        {
            ApxSoftBody body = CreateBody(8);
            GameObject go = InScene(new GameObject("Scalpel"));
            var scalpel = go.AddComponent<ApxScalpel>();
            scalpel.cutting = false;
            var tool = go.AddComponent<ApxScalpelTool>();
            tool.scalpel = scalpel;
            tool.blade = new ApxBladePose { tilt = 20.0f, depth = 0.12f, clearance = 0.01f };

            // Pressed beside the body and moved 1 mm: the blade waits for the stroke direction.
            Frame(tool, scalpel, body, Pointer(-0.08f, true, true, false));
            Frame(tool, scalpel, body, Pointer(-0.079f, false, true, false));
            Assert.IsFalse(tool.IsCutting);
            Assert.IsFalse(scalpel.cutting);
            Assert.AreEqual(1, body.Simulation.CountPieces());

            for (int f = 0; f <= 40; f++)
            {
                Frame(tool, scalpel, body, Pointer(Mathf.Lerp(-0.075f, 0.08f, f / 40.0f), false, true, false));
            }
            Assert.IsTrue(tool.IsCutting);
            Assert.AreEqual(90.0f, tool.blade.heading, 0.5f, "heading along the stroke (+x)");
            Vector3 edge = scalpel.transform.TransformPoint(scalpel.edgeEnd) - scalpel.transform.TransformPoint(scalpel.edgeStart);
            Assert.AreEqual(20.0f, Vector3.Angle(edge, Vector3.down), 0.1f, "blade tilted from the vertical");
            Assert.Less(Vector3.Angle(edge, tool.blade.EdgeDirection), 0.1f);
            Assert.Greater(edge.z * -1.0f, 0.0f, "positive tilt leans the tip to the right of +x (-z)");

            Frame(tool, scalpel, body, Pointer(0.08f, false, false, true));
            Assert.IsFalse(scalpel.cutting);
            Assert.Greater(tool.SeveredThisStroke, 0);
            Assert.AreEqual(0.16f - tool.stabilizer, tool.StrokeLength, 0.001f,
                "from the press at x = -0.08 to the string length behind the pointer at x = 0.08");
            for (int f = 0; f < 10; f++)
            {
                Frame(tool, scalpel, body, Pointer(0.2f, false, false, false));
            }
            List<int> pieces = body.Simulation.PieceSizes();
            int total = pieces.Sum();
            Assert.AreEqual(2, pieces.Count(n => n > 0.02f * total), $"pieces [{string.Join(",", pieces)}]");
        }

        [Test]
        public void ScalpelToolStringKeepsHandJitterOutOfTheCut()
        {
            ApxSoftBody body = CreateBody(16);
            GameObject go = InScene(new GameObject("Scalpel"));
            var scalpel = go.AddComponent<ApxScalpel>();
            scalpel.cutting = false;
            var tool = go.AddComponent<ApxScalpelTool>();
            tool.scalpel = scalpel;
            tool.blade = new ApxBladePose { depth = 0.12f, clearance = 0.01f };

            // The pointer moves 3 mm along x per frame and shakes 2 mm to either side every two frames.
            const int frames = 60;
            const float shake = 0.002f;
            for (int f = 0; f <= frames; f++)
            {
                float z = f == 0 ? 0.0f : f % 4 < 2 ? shake : -shake;
                Frame(tool, scalpel, body, Pointer(Mathf.Lerp(-0.08f, 0.1f, f / (float)frames), z, f == 0, true, false));
            }
            Frame(tool, scalpel, body, Pointer(0.1f, 0.0f, false, false, true));
            // Without the string the blade would follow the full shake.
            float lateral = tool.Trail.Max(p => Mathf.Abs(p.z));
            Assert.Less(lateral, 0.5f * shake, "blade path across the stroke");
            for (int f = 0; f < 10; f++)
            {
                Frame(tool, scalpel, body, Pointer(0.2f, 0.0f, false, false, false));
            }
            List<int> pieces = body.Simulation.PieceSizes();
            int total = pieces.Sum();
            Assert.AreEqual(2, pieces.Count(n => n > 0.02f * total), $"pieces [{string.Join(",", pieces)}]");
        }

        [Test]
        public void ForcepsToolLiftsTheGrabAndLetsGo()
        {
            ApxSoftBody body = CreateBody(8);
            var forceps = InScene(new GameObject("Forceps")).AddComponent<ApxForcepsTool>();
            forceps.grabRadius = 0.015f;
            forceps.liftStep = 0.002f;

            forceps.Tick(Pointer(0.0f, true, true, false));
            Assert.AreSame(body, forceps.Grabbed);
            Assert.IsTrue(body.IsGrabbing);
            Vector3 start = forceps.Target;
            float top = body.WorldBounds.max.y;
            ApxToolInput lift = Pointer(0.0f, false, true, false);
            lift.Scroll = 5.0f;
            forceps.Tick(lift);
            Assert.AreEqual(start.y + 0.01f, forceps.Target.y, 1e-5f, "five wheel steps lift 10 mm");
            for (int f = 0; f < 30; f++)
            {
                forceps.Tick(Pointer(0.0f, false, true, false));
                body.Simulate(Dt);
                body.UpdateRender();
            }
            Assert.AreEqual(top + 0.01f, body.WorldBounds.max.y, 0.002f, "the grabbed top follows the lift");

            // Released with Shift: the grab stays until the next click.
            ApxToolInput keep = Pointer(0.0f, false, false, true);
            keep.Shift = true;
            forceps.Tick(keep);
            Assert.IsTrue(body.IsGrabbing);
            forceps.Tick(Pointer(0.0f, true, true, false));
            Assert.IsFalse(body.IsGrabbing);
            Assert.IsNull(forceps.Grabbed);
        }

        [Test]
        public void MeasureToolFollowsItsNodesThroughDeformationAndCuts()
        {
            ApxSoftBody body = CreateBody(8);
            var measure = InScene(new GameObject("Measure")).AddComponent<ApxMeasureTool>();
            measure.Tick(Pointer(-0.03f, true, true, false));
            measure.Tick(Pointer(0.03f, true, true, false));
            Assert.AreEqual(2, measure.PointCount);
            float cell = body.Simulation.CellSize;
            Assert.AreEqual(0.06f, measure.Distance, cell + 1e-4f);
            float before = measure.Distance;

            // Move the half with x > 0 by 10 mm along x: the measurement grows by 10 mm.
            SoftBodySimulation sim = body.Simulation;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.Positions[i].x > 0.0f)
                {
                    sim.Positions[i] += new Unity.Mathematics.float3(0.01f, 0.0f, 0.0f);
                }
            }
            measure.Tick(Pointer(0.3f, false, false, false));
            Assert.AreEqual(before + 0.01f, measure.Distance, 1e-5f);

            // A cut renumbers the nodes: the points move to the nearest real nodes.
            float stretched = measure.Distance;
            int severed = body.Cut(new Vector3(0.0f, 0.2f, -0.1f), new Vector3(0.0f, -0.1f, -0.1f),
                new Vector3(0.0f, 0.2f, 0.1f), new Vector3(0.0f, -0.1f, 0.1f));
            Assert.Greater(severed, 0);
            measure.Tick(Pointer(0.3f, false, false, false));
            Assert.AreEqual(stretched, measure.Distance, 1e-5f, "the nodes under the points kept their positions");
        }

        [Test]
        public void DebugViewDrawsNoLatticeEdgeAcrossACut()
        {
            ApxSoftBody body = CreateBody(8);
            var view = InScene(new GameObject("Debug View")).AddComponent<ApxDebugView>();
            view.layers = ApxDebugLayers.Grid | ApxDebugLayers.Bounds | ApxDebugLayers.Lattice | ApxDebugLayers.Nodes |
                          ApxDebugLayers.Cuts | ApxDebugLayers.Forces | ApxDebugLayers.Blades;
            view.Refresh();
            int uncut = view.LatticeEdges(body).Count / 2;
            Assert.Greater(uncut, 0);
            Assert.AreEqual(0, view.CutCount(body));
            Assert.Greater(view.Lines.Count, 0);

            body.Cut(new Vector3(0.0f, 0.2f, -0.1f), new Vector3(0.0f, -0.1f, -0.1f),
                new Vector3(0.0f, 0.2f, 0.1f), new Vector3(0.0f, -0.1f, 0.1f));
            view.Refresh();
            SoftBodySimulation sim = body.Simulation;
            Assert.AreEqual(2, sim.CountPieces());
            IReadOnlyList<int> edges = view.LatticeEdges(body);
            Assert.Less(edges.Count / 2, uncut);
            for (int k = 0; k < edges.Count; k += 2)
            {
                Assert.AreEqual(sim.PieceId[edges[k]], sim.PieceId[edges[k + 1]], $"edge {edges[k]}-{edges[k + 1]} crosses the cut");
            }
            // The plane x = 0 severs the x edges of the cells it passes through, one per (y, z) grid line.
            var lines = new HashSet<(int, int)>();
            float h = sim.CellSize;
            for (int c = 0; c < sim.CellCount; c++)
            {
                Unity.Mathematics.int3 coord = sim.CellCoord[c];
                float x0 = sim.Grid.Origin.x + coord.x * h;
                if (x0 < 0.0f && x0 + h > 0.0f)
                {
                    lines.Add((coord.y, coord.z));
                    lines.Add((coord.y + 1, coord.z));
                    lines.Add((coord.y, coord.z + 1));
                    lines.Add((coord.y + 1, coord.z + 1));
                }
            }
            Assert.AreEqual(lines.Count, view.CutCount(body));
        }

        private static void Frame(ApxScalpelTool tool, ApxScalpel scalpel, ApxSoftBody body, in ApxToolInput input)
        {
            tool.Tick(input);
            scalpel.Sweep();
            body.Simulate(Dt);
            body.UpdateRender();
        }

        /// <summary>Vertical pointer ray at (x, 0) with the given button state.</summary>
        private static ApxToolInput Pointer(float x, bool down, bool held, bool up) => Pointer(x, 0.0f, down, held, up);

        /// <summary>Vertical pointer ray at (x, z) with the given button state.</summary>
        private static ApxToolInput Pointer(float x, float z, bool down, bool held, bool up) => new ApxToolInput
        {
            Ray = new Ray(new Vector3(x, 0.5f, z), Vector3.down),
            ViewForward = Vector3.down,
            Down = down,
            Held = held,
            Up = up,
            DeltaTime = Dt,
        };

        private static Vector2 Pixel(ApxOrbit orbit, Vector3 p, float fov, float height)
        {
            Vector3 local = Quaternion.Inverse(orbit.Rotation) * (p - orbit.Position);
            float scale = 0.5f * height / Mathf.Tan(0.5f * fov * Mathf.Deg2Rad);
            return new Vector2(local.x, local.y) / local.z * scale;
        }

        /// <summary>A 10 cm cube standing on y = 0 at the origin, without gravity, initialized.</summary>
        private ApxSoftBody CreateBody(int resolution)
        {
            Mesh mesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.1f, 0.1f), 4);
            _objects.Add(mesh);
            GameObject go = InScene(new GameObject("Body", typeof(MeshFilter), typeof(MeshRenderer)));
            go.transform.position = new Vector3(0.0f, 0.05f, 0.0f);
            go.GetComponent<MeshFilter>().sharedMesh = mesh;
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = resolution;
            body.useGravity = false;
            _bodies.Add(body);
            body.Initialize();
            return body;
        }

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
