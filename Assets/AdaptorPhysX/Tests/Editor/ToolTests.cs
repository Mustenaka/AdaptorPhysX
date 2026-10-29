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
    /// forceps, measuring and grasper tools driven by scripted pointer input, and the lattice and cut layers.
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

        /// <summary>
        /// The grasper tool on a soft sheet (16 x 1 x 8 cm, cell 3.3 mm, E = 20 kPa) held at its ends by clamps and sagging under
        /// its weight (TODO MED-10): a press on the near edge brings the grasper in along the view with a jaw above and one
        /// below the sheet and closes it; the grasper takes the edge, the wheel lifts it by 20 mm, turning the grasper by 45
        /// degrees about its jaws turns the held tissue with it, and a release with Shift keeps the hold until the next click,
        /// which opens the jaws, lets go and draws the grasper back.
        /// </summary>
        [Test]
        public void GrasperToolTakesAnEdgeLiftsAndTurnsIt()
        {
            ApxSoftBody sheet = CreateSheet();
            ApxGrasperTool tool = CreateGrasperTool();
            ApxGrasper grasper = tool.grasper;
            for (int f = 0; f < 60; f++)
            {
                GrasperFrame(tool, sheet, default);
            }
            // The pointer stays where it was pressed.
            ApxToolInput press = SheetEdgePointer(sheet);
            ApxToolInput Pointer(bool down, bool held, bool up)
            {
                ApxToolInput input = press;
                input.Down = down;
                input.Held = held;
                input.Up = up;
                return input;
            }
            Assert.IsTrue(tool.Plan(press.Ray, press.ViewForward, out Vector3 planned, out Quaternion rotation, out float open,
                out bool fits, out ApxSoftBody planBody));
            Assert.AreSame(sheet, planBody);
            Assert.IsTrue(fits);
            Assert.Greater(Vector3.Dot(rotation * Vector3.forward, Vector3.forward), 0.99f, "the grasper comes in along the view");
            Assert.Greater(Vector3.Dot(rotation * Vector3.up, Vector3.up), 0.99f, "a jaw above and one below");
            // Points across the jaws (x) a little inside the edge, between the jaws: they turn with the held tissue.
            Vector3 tip = planned + rotation * Vector3.forward * (0.5f * grasper.jawSize.z);
            int left = sheet.AddMaterialPoint(new Vector3(tip.x - 0.0025f, planned.y, tip.z - 0.005f));
            int right = sheet.AddMaterialPoint(new Vector3(tip.x + 0.0025f, planned.y, tip.z - 0.005f));
            Assert.GreaterOrEqual(left, 0);
            Assert.GreaterOrEqual(right, 0);
            float start = sheet.MaterialPointPosition(left).y;

            GrasperFrame(tool, sheet, Pointer(true, true, false));
            Assert.AreEqual(ApxGrasperTool.Motion.Approaching, tool.State);
            Assert.IsTrue(grasper.gameObject.activeSelf);
            for (int f = 0; f < 40; f++)
            {
                GrasperFrame(tool, sheet, Pointer(false, true, false));
            }
            TestContext.WriteLine($"grasped: open {open * 1000:0.0} mm, held {grasper.HeldPoints} points, squeeze {grasper.Squeeze:0.000} N, " +
                                  $"grip {grasper.GripCapacity:0.000} N");
            Assert.AreEqual(ApxGrasperTool.Motion.Holding, tool.State);
            Assert.AreSame(sheet, grasper.Held, "the grasper took the edge");
            Assert.Greater(grasper.HeldPoints, 0);

            // Lift by ten wheel steps of 2 mm, one every six frames, and hold.
            for (int f = 0; f < 90; f++)
            {
                ApxToolInput lift = Pointer(false, true, false);
                lift.Scroll = f < 60 && f % 6 == 0 ? 1.0f : 0.0f;
                GrasperFrame(tool, sheet, lift);
            }
            float lifted = sheet.MaterialPointPosition(left).y - start;
            // Turn by 45 degrees about the jaws in half a second, and hold for one.
            for (int f = 0; f < 90; f++)
            {
                tool.Turn(0.0f, 0.0f, f < 30 ? 1.5f : 0.0f);
                GrasperFrame(tool, sheet, Pointer(false, true, false));
            }
            Vector3 across = (sheet.MaterialPointPosition(right) - sheet.MaterialPointPosition(left)).normalized;
            grasper.TryGetWrench(out ApxWrench wrench);
            TestContext.WriteLine($"lifted {lifted * 1000:0.0} mm, turned {tool.Roll:0} deg: across the jaws {across}, held {grasper.HeldPoints} " +
                                  $"points, squeeze {grasper.Squeeze:0.000} N, grip {grasper.GripCapacity:0.000} N, force on the grasper " +
                                  $"{wrench.Force} N, max speed {sheet.Simulation.MaxSpeed:0.000} m/s");
            Assert.AreEqual(0.02f, lifted, 0.003f, "the held edge follows the lift");
            Assert.AreSame(sheet, grasper.Held);
            Assert.IsFalse(grasper.Slipped);
            // Turned by 45 degrees about +z (the jaws), +x turns halfway to +y.
            Assert.Greater(Vector3.Dot(across, new Vector3(1.0f, 1.0f, 0.0f).normalized), 0.95f, $"the held tissue turned with the grasper: {across}");

            // Released with Shift: the hold stays until the next click.
            ApxToolInput keep = Pointer(false, false, true);
            keep.Shift = true;
            GrasperFrame(tool, sheet, keep);
            for (int f = 0; f < 10; f++)
            {
                GrasperFrame(tool, sheet, default);
            }
            Assert.AreEqual(ApxGrasperTool.Motion.Holding, tool.State);
            Assert.AreSame(sheet, grasper.Held);
            GrasperFrame(tool, sheet, Pointer(true, true, false));
            Assert.AreEqual(ApxGrasperTool.Motion.Opening, tool.State);
            for (int f = 0; f < 60; f++)
            {
                GrasperFrame(tool, sheet, default);
            }
            float back = sheet.MaterialPointPosition(left).y - start;
            TestContext.WriteLine($"released: the edge is {back * 1000:0.0} mm above where it was taken");
            Assert.AreEqual(ApxGrasperTool.Motion.Idle, tool.State);
            Assert.IsFalse(grasper.gameObject.activeSelf, "the grasper is parked");
            Assert.IsNull(grasper.Held);
            Assert.Less(back, 0.5f * lifted, "the released edge sags back");
        }

        /// <summary>The grasper tool does not take tissue thicker than its jaws open: a 10 cm cube.</summary>
        [Test]
        public void GrasperToolDoesNotTakeTissueThickerThanItsJaws()
        {
            ApxSoftBody body = CreateBody(8);
            ApxGrasperTool tool = CreateGrasperTool();
            var press = new ApxToolInput
            {
                Ray = new Ray(new Vector3(0.0f, 0.2f, -0.3f), new Vector3(0.0f, -0.1f, 0.3f).normalized),
                ViewForward = new Vector3(0.0f, -0.1f, 0.3f).normalized,
                Down = true,
                Held = true,
                DeltaTime = Dt,
            };
            Assert.IsTrue(tool.Plan(press.Ray, press.ViewForward, out _, out _, out float open, out bool fits, out ApxSoftBody picked));
            Assert.AreSame(body, picked);
            Assert.IsFalse(fits, $"open gap {open * 1000:0} mm");
            tool.Tick(press);
            Assert.AreEqual(ApxGrasperTool.Motion.Idle, tool.State);
            Assert.IsFalse(tool.grasper.gameObject.activeSelf);
        }

        /// <summary>One frame of the grasper tool: its input, then a fixed step of the tool, the grasper and the body.</summary>
        private static void GrasperFrame(ApxGrasperTool tool, ApxSoftBody body, in ApxToolInput input)
        {
            tool.Tick(input);
            tool.Advance(Dt);
            if (tool.grasper.gameObject.activeSelf)
            {
                tool.grasper.Step();
            }
            Physics.SyncTransforms();
            body.Simulate(Dt);
            body.UpdateRender();
        }

        /// <summary>A press from in front of and above the sheet on its near edge, at x = 0.</summary>
        private static ApxToolInput SheetEdgePointer(ApxSoftBody sheet)
        {
            SoftBodySimulation sim = sheet.Simulation;
            int top = sim.PickNode(new Vector3(0.0f, 0.3f, -0.03f), Vector3.down, sim.CellSize);
            int edge = sim.PickNode(new Vector3(0.0f, sim.Positions[top].y - 0.003f, -0.3f), Vector3.forward, sim.CellSize);
            Vector3 origin = new Vector3(0.0f, 0.2f, -0.3f);
            Vector3 direction = ((Vector3)sim.Positions[edge] - origin).normalized;
            return new ApxToolInput
            {
                Ray = new Ray(origin, direction),
                ViewForward = direction,
                Down = true,
                Held = true,
                DeltaTime = Dt,
            };
        }

        /// <summary>A soft sheet (16 x 1 x 8 cm) centred 7 cm above y = 0, held at its ends by clamps, with gravity, initialized.</summary>
        private ApxSoftBody CreateSheet()
        {
            Mesh mesh = MeshUtil.CreateBox(new Vector3(0.16f, 0.01f, 0.08f), 4);
            _objects.Add(mesh);
            GameObject go = InScene(new GameObject("Sheet", typeof(MeshFilter), typeof(MeshRenderer)));
            go.transform.position = new Vector3(0.0f, 0.07f, 0.0f);
            go.GetComponent<MeshFilter>().sharedMesh = mesh;
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = 48;
            body.youngsModulus = 2.0e4f;
            body.pinColliders = new[] { Clamp(-0.08f), Clamp(0.08f) };
            _bodies.Add(body);
            Physics.SyncTransforms();
            body.Initialize();
            return body;
        }

        private Collider Clamp(float x)
        {
            GameObject go = InScene(new GameObject("Clamp"));
            go.transform.position = new Vector3(x, 0.07f, 0.0f);
            var box = go.AddComponent<BoxCollider>();
            box.size = new Vector3(0.025f, 0.03f, 0.1f);
            box.isTrigger = true;
            return box;
        }

        /// <summary>A grasper tool and its grasper, parked.</summary>
        private ApxGrasperTool CreateGrasperTool()
        {
            var tool = InScene(new GameObject("Tools")).AddComponent<ApxGrasperTool>();
            tool.grasper = InScene(new GameObject("Grasper")).AddComponent<ApxGrasper>();
            tool.grasper.gameObject.SetActive(false);
            return tool;
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
