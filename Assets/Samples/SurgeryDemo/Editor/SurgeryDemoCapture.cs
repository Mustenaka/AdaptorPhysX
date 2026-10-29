using System;
using System.Collections.Generic;
using System.IO;
using AdaptorPhysX.Tools;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Runs the standard scenarios (ROADMAP S1-S7) on the demo scene in edit mode with a
    /// scripted scalpel and forceps, and writes a PNG per stage. The run is recorded and then
    /// replayed in the scene opened again (TODO MED-14). Batch use:
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.SurgeryDemoCapture.CaptureBatch -apxCaptureDir &lt;dir&gt;
    /// </summary>
    public static class SurgeryDemoCapture
    {
        private const string ScenePath = "Assets/Samples/SurgeryDemo/SurgeryDemo.unity";
        private const float Dt = 1.0f / 60.0f;

        private static Camera _camera;
        private static ApxScalpel _scalpel;
        private static ApxRecorder _recorder;
        private static ApxDebugView _debugView;
        private static string _directory;
        private static readonly List<StitchLine> _stitchLines = new List<StitchLine>();

        [MenuItem("AdaptorPhysX/Capture Standard Scenarios")]
        public static void CaptureMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }
            SceneSetup[] setup = EditorSceneManager.GetSceneManagerSetup();
            try
            {
                Capture(Path.Combine(Directory.GetCurrentDirectory(), "Captures"));
            }
            finally
            {
                // The capture moves the camera and replaces meshes and materials in memory: reload from disk.
                if (setup.Length > 0 && Array.TrueForAll(setup, s => !string.IsNullOrEmpty(s.path)))
                {
                    EditorSceneManager.RestoreSceneManagerSetup(setup);
                }
                else
                {
                    EditorSceneManager.OpenScene(ScenePath);
                }
            }
        }

        [MenuItem("AdaptorPhysX/Capture Standard Scenarios", true)]
        private static bool CanCapture() => !EditorApplication.isPlayingOrWillChangePlaymode;

        public static void CaptureBatch()
        {
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, "-apxCaptureDir");
            string directory = index >= 0 && index + 1 < args.Length
                ? args[index + 1]
                : Path.Combine(Directory.GetCurrentDirectory(), "Captures");
            int code = 0;
            try
            {
                Capture(directory);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                code = 1;
            }
            EditorApplication.Exit(code);
        }

        private static void Capture(string directory)
        {
            _directory = directory;
            Directory.CreateDirectory(directory);
            if (!File.Exists(ScenePath))
            {
                SurgeryDemoSceneBuilder.Build();
            }
            EditorSceneManager.OpenScene(ScenePath);
            _stitchLines.Clear();
            try
            {
                RunScenarios(out ApxRecording recording, out string score);
                Replay(recording, score);
            }
            finally
            {
                // Edit-mode bodies get no OnDestroy: release their native memory even when a stage throws.
                foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
                {
                    body.Release();
                }
            }
        }

        private static void RunScenarios(out ApxRecording recording, out string score)
        {
            PrepareScene();
            _camera = Camera.main;
            // The standard shots show the surfaces only; the debug view is turned on for the D shots.
            _debugView = UnityEngine.Object.FindObjectOfType<ApxDebugView>();
            if (_debugView != null)
            {
                _debugView.enabled = false;
            }
            ApxSoftBody block = Find("Tissue Block");
            ApxSoftBody flap = Find("Skin Flap");
            ApxSoftBody liver = Find("Liver Lobe");
            ApxSoftBody pad = Find("Tissue Pad");
            // TODO MED-13: a scorer observes the bodies through all scenarios; its report is written next to the shots.
            ApxScorer scorer = CreateScorer();
            // TODO MED-14: the instruments and operations are recorded from the first step on.
            _recorder = CreateRecorder();
            _recorder.StartRecording();
            scorer.IncisionMade += incision => Debug.Log(
                $"[ApxCapture] incision: {incision.Body} stroke {incision.Stroke}, {incision.SeveredEdges} edges, length " +
                $"{incision.Length * 1000:0.0} mm, depth {incision.Depth * 1000:0.0} mm, area {incision.Area * 1e6f:0} mm^2, " +
                $"centre {incision.Center * 1000} mm");
            scorer.ForceExceeded += force => Debug.Log(
                $"[ApxCapture] force above {ForceLimit} N: {force.Body} {force.Source} at {force.Start:0.00} s, {force.Peak:0.00} N");

            Run(45);
            Log("settled");
            Shot("00_overview", new Vector3(0.0f, 0.32f, -0.42f), new Vector3(0.0f, 0.03f, 0.0f));

            // S1: full cut through the block along z, then pull the left half away.
            Stroke(new Vector3(-0.2f, 0.08f, -0.09f), new Vector3(-0.2f, 0.08f, 0.09f), Vector3.zero,
                new Vector3(0.0f, -0.09f, 0.0f), 40);
            Run(20);
            Log("S1 cut");
            Shot("S1a_block_cut", new Vector3(-0.2f, 0.2f, -0.2f), new Vector3(-0.2f, 0.03f, 0.0f));
            Drag(block, new Vector3(-0.24f, 0.053f, 0.0f), new Vector3(-0.27f, 0.075f, -0.01f), 40, 0.015f);
            Log("S1 pulled");
            Shot("S1b_block_pulled", new Vector3(-0.2f, 0.16f, -0.2f), new Vector3(-0.21f, 0.035f, 0.0f));
            Release(block);
            Run(40);

            // S3: cross cut, then re-cut one quarter.
            Stroke(new Vector3(-0.34f, 0.08f, 0.0f), new Vector3(-0.105f, 0.08f, 0.0f), Vector3.zero,
                new Vector3(0.0f, -0.09f, 0.0f), 40);
            Stroke(new Vector3(-0.17f, 0.08f, -0.004f), new Vector3(-0.17f, 0.08f, 0.07f), Vector3.zero,
                new Vector3(0.0f, -0.09f, 0.0f), 30);
            Run(30);
            Log("S3 cross + re-cut");
            Shot("S3_block_cross_recut", new Vector3(-0.2f, 0.26f, -0.1f), new Vector3(-0.2f, 0.03f, 0.0f));
            // The pieces still touch and hide the cuts: move the three back pieces away, one grab at a time.
            Drag(block, new Vector3(-0.155f, 0.053f, 0.025f), new Vector3(-0.13f, 0.08f, 0.055f), 40, 0.01f);
            Release(block);
            Run(30);
            Drag(block, new Vector3(-0.185f, 0.053f, 0.025f), new Vector3(-0.185f, 0.08f, 0.055f), 40, 0.01f);
            Release(block);
            Run(30);
            Drag(block, new Vector3(-0.26f, 0.053f, 0.025f), new Vector3(-0.26f, 0.08f, 0.055f), 40, 0.01f);
            Release(block);
            Run(40);
            Log("S3 pieces apart");
            Shot("S3b_block_pieces_apart", new Vector3(-0.2f, 0.26f, -0.1f), new Vector3(-0.2f, 0.03f, 0.01f));
            // D1: lattice coloured by piece and the gaps of the severed edges, surfaces hidden.
            DebugShot("D1_block_lattice_cuts", ApxDebugLayers.Grid | ApxDebugLayers.Lattice | ApxDebugLayers.Cuts, true,
                new Vector3(-0.2f, 0.2f, -0.16f), new Vector3(-0.2f, 0.02f, 0.01f));

            // S2: partial incision into the pre-tensioned flap, about half its thickness deep.
            float flapTop = SurfaceHeight(flap, new Vector3(0.0f, 0.3f, 0.0f));
            Stroke(new Vector3(0.0f, flapTop + 0.01f, -0.045f), new Vector3(0.0f, flapTop + 0.01f, 0.045f), Vector3.zero,
                new Vector3(0.0f, -0.019f, 0.0f), 40);
            Run(60);
            Log("S2 partial incision");
            Shot("S2_flap_incision", new Vector3(0.0f, 0.2f, -0.13f), new Vector3(0.0f, 0.06f, 0.0f));
            Shot("S2_flap_incision_close", new Vector3(0.0f, 0.13f, -0.025f), new Vector3(0.0f, 0.065f, 0.0f));

            // S4: forceps lift one incision edge.
            Drag(flap, new Vector3(0.006f, 0.068f, 0.0f), new Vector3(0.022f, 0.09f, 0.0f), 40, 0.008f);
            Log("S4 retraction");
            Shot("S4_flap_retraction", new Vector3(-0.06f, 0.16f, -0.13f), new Vector3(0.005f, 0.065f, 0.0f));
            Shot("S4_flap_retraction_close", new Vector3(-0.04f, 0.11f, -0.05f), new Vector3(0.005f, 0.068f, 0.0f));
            Release(flap);
            Run(30);

            // S5: wedge resection: a V-shaped pair of tilted strokes closed by two transverse end cuts.
            Stroke(new Vector3(0.22f, 0.0f, -0.035f), new Vector3(0.22f, 0.0f, 0.035f), new Vector3(-0.028f, 0.08f, 0.0f),
                new Vector3(0.008f, 0.022f, 0.0f), 30);
            Stroke(new Vector3(0.22f, 0.0f, -0.035f), new Vector3(0.22f, 0.0f, 0.035f), new Vector3(0.028f, 0.08f, 0.0f),
                new Vector3(-0.008f, 0.022f, 0.0f), 30);
            Stroke(new Vector3(0.18f, 0.0f, -0.033f), new Vector3(0.26f, 0.0f, -0.033f), new Vector3(0.0f, 0.08f, 0.0f),
                new Vector3(0.0f, 0.025f, 0.0f), 20);
            Stroke(new Vector3(0.18f, 0.0f, 0.033f), new Vector3(0.26f, 0.0f, 0.033f), new Vector3(0.0f, 0.08f, 0.0f),
                new Vector3(0.0f, 0.025f, 0.0f), 20);
            Run(20);
            Log("S5 wedge cuts");
            Shot("S5a_liver_cuts", new Vector3(0.22f, 0.16f, -0.12f), new Vector3(0.22f, 0.05f, 0.0f));
            Drag(liver, new Vector3(0.22f, 0.07f, 0.0f), new Vector3(0.22f, 0.12f, -0.02f), 50, 0.012f);
            Log("S5 wedge lifted");
            Shot("S5b_liver_wedge_lifted", new Vector3(0.3f, 0.2f, -0.18f), new Vector3(0.22f, 0.06f, 0.0f));
            Shot("S5c_liver_wedge_side", new Vector3(0.22f, 0.09f, -0.2f), new Vector3(0.22f, 0.06f, 0.0f));

            // S6: short, shallow incision into the solid tissue pad. Skin tension opens it; the tissue
            // below the incision stays solid. Then the forceps retract one edge to show the layers.
            float padTop = SurfaceHeight(pad, new Vector3(0.0f, 0.3f, -0.21f));
            // For S7 (below): material points across the incision placed before it is cut, so each keeps its side. Three
            // stitches 6 mm apart join points 3 mm either side of it and 1 mm below the rendered skin (the topmost node
            // lies up to a cell above it); the points at the surface above them hold the drawn threads, and pairs on the
            // two walls just below the surface measure the opening.
            float skin = SkinHeight(pad, 0.0f, -0.21f);
            var stitchA = new int[3];
            var stitchB = new int[3];
            var threadA = new int[3];
            var threadB = new int[3];
            for (int i = 0; i < 3; i++)
            {
                float x = 0.006f * (i - 1);
                stitchA[i] = pad.AddMaterialPoint(new Vector3(x, skin - 0.001f, -0.213f));
                stitchB[i] = pad.AddMaterialPoint(new Vector3(x, skin - 0.001f, -0.207f));
                threadA[i] = pad.AddMaterialPoint(new Vector3(x, skin - 0.0002f, -0.213f));
                threadB[i] = pad.AddMaterialPoint(new Vector3(x, skin - 0.0002f, -0.207f));
            }
            var walls = new List<Vector2Int>();
            for (int i = -5; i <= 5; i++)
            {
                walls.Add(new Vector2Int(pad.AddMaterialPoint(new Vector3(0.002f * i, skin - 0.0002f, -0.2102f)),
                    pad.AddMaterialPoint(new Vector3(0.002f * i, skin - 0.0002f, -0.2098f))));
            }
            float[] wallRest = walls.ConvertAll(w => Vector3.Distance(pad.MaterialPointPosition(w.x), pad.MaterialPointPosition(w.y))).ToArray();
            Stroke(new Vector3(-0.012f, padTop + 0.01f, -0.21f), new Vector3(0.012f, padTop + 0.01f, -0.21f), Vector3.zero,
                new Vector3(0.0f, -0.018f, 0.0f), 30);
            Run(60);
            Log("S6 small incision");
            Shot("S6a_pad_incision", new Vector3(0.0f, padTop + 0.06f, -0.27f), new Vector3(0.0f, padTop, -0.21f));
            Drag(pad, new Vector3(0.0f, padTop, -0.2145f), new Vector3(0.0f, padTop + 0.006f, -0.222f), 40, 0.005f);
            Log("S6 retraction");
            Shot("S6b_pad_retracted", new Vector3(0.0f, padTop + 0.05f, -0.255f), new Vector3(0.0f, padTop - 0.004f, -0.21f));
            Release(pad);

            // D3: a leaning blade held at the pad surface (heading +x, tilt 30, rake 20 degrees, 10 mm deep).
            var pose = new ApxBladePose { heading = 90.0f, tilt = 30.0f, rake = 20.0f, depth = 0.01f, clearance = 0.01f };
            var bladePoint = new Vector3(0.025f, padTop, -0.205f);
            pose.GetEdge(bladePoint, out Vector3 heel, out Vector3 tip);
            _scalpel.cutting = false;
            _scalpel.transform.SetPositionAndRotation(bladePoint, pose.Rotation);
            _scalpel.edgeStart = _scalpel.transform.InverseTransformPoint(heel);
            _scalpel.edgeEnd = _scalpel.transform.InverseTransformPoint(tip);
            DebugShot("D3_blade_pose", ApxDebugLayers.Grid | ApxDebugLayers.Blades, false,
                new Vector3(0.025f, padTop + 0.045f, -0.27f), new Vector3(0.025f, padTop - 0.004f, -0.205f));

            Shot("99_overview", new Vector3(0.0f, 0.32f, -0.42f), new Vector3(0.0f, 0.03f, 0.0f));

            // S7: once the retracted edge is back, the three stitches across the S6 incision are drawn at 1 cm/s to the
            // length of the skin between their points without stress; the incision closes. The threads are drawn over the skin.
            Run(60);
            float opened = Opening(pad, walls, wallRest);
            var stitches = new int[3];
            for (int i = 0; i < 3; i++)
            {
                stitches[i] = pad.AddStitch(stitchA[i], stitchB[i], 0.01f);
                _stitchLines.Add(StitchLine.Create(pad, threadA[i], threadB[i]));
            }
            Run(150);
            Log("S7 sutured");
            Debug.Log($"[ApxCapture] S7: opening at the surface {opened * 1000:0.00} -> {Opening(pad, walls, wallRest) * 1000:0.00} mm, " +
                      $"tensions {string.Join(" / ", Array.ConvertAll(stitches, t => pad.StitchTension(t).ToString("0.000")))} N");
            Shot("S7a_pad_sutured", new Vector3(0.0f, padTop + 0.06f, -0.27f), new Vector3(0.0f, padTop, -0.21f));
            Shot("S7b_pad_sutured_close", new Vector3(0.0f, skin + 0.03f, -0.218f), new Vector3(0.0f, skin, -0.21f));

            recording = _recorder.StopRecording();
            File.WriteAllText(Path.Combine(_directory, "recording.json"), recording.ToJson());
            ApxScoreReport report = scorer.Report();
            score = report.ToJson();
            File.WriteAllText(Path.Combine(_directory, "score.json"), score);
            Debug.Log($"[ApxCapture] score: {report.Incisions.Count} incisions, cut area {report.CutArea * 1e6f:0} mm^2, " +
                      $"{report.Pieces} pieces, {report.ForceEvents.Count} times above {ForceLimit} N for {report.OverForceTime:0.00} s, " +
                      $"peak {report.PeakForce:0.00} N");
        }

        /// <summary>Force on an instrument above which the scorer reports it, N.</summary>
        private const float ForceLimit = 1.0f;

        /// <summary>
        /// Opens the scene again and replays the recording of the scenarios (TODO MED-14): the bodies' node positions agree
        /// with the recording at its checks (every 60 steps and after the last), and the score with the recorded score.
        /// </summary>
        private static void Replay(ApxRecording recording, string score)
        {
            foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
            {
                body.Release();
            }
            EditorSceneManager.OpenScene(ScenePath);
            // The threads drawn in S7 belong to the recorded run.
            _stitchLines.Clear();
            var timer = System.Diagnostics.Stopwatch.StartNew();
            PrepareScene();
            ApxScorer scorer = CreateScorer();
            _recorder = CreateRecorder();
            _recorder.Replay(recording);
            int steps = 0;
            while (true)
            {
                _recorder.Step();
                if (!_recorder.IsReplaying)
                {
                    break;
                }
                StepScene();
                steps++;
            }
            string replayed = scorer.Report().ToJson();
            string result = $"[ApxCapture] replay: {steps} steps of {recording.Bodies.Count} bodies in {timer.Elapsed.TotalSeconds:0.0} s, " +
                            $"{recording.Operations.Count} operations, {recording.Poses.Count} scalpel poses; " +
                            $"{_recorder.ChecksCompared} positions hashes {(_recorder.Diverged ? $"differ from step {_recorder.DivergedTick} on" : "as recorded")}, " +
                            $"score {(replayed == score ? "as recorded" : "differs")}";
            if (_recorder.Diverged || replayed != score || steps != recording.Steps)
            {
                Debug.LogError(result);
                File.WriteAllText(Path.Combine(_directory, "score-replay.json"), replayed);
            }
            else
            {
                Debug.Log(result);
            }
        }

        /// <summary>Applies the demo shapes and initializes the bodies of the opened scene.</summary>
        private static void PrepareScene()
        {
            foreach (ApxDemoShape shape in UnityEngine.Object.FindObjectsOfType<ApxDemoShape>())
            {
                shape.Apply();
            }
            foreach (ApxSoftBody body in UnityEngine.Object.FindObjectsOfType<ApxSoftBody>())
            {
                body.Initialize();
            }
            Physics.SyncTransforms();
            _scalpel = UnityEngine.Object.FindObjectOfType<ApxScalpel>();
        }

        private static ApxScorer CreateScorer()
        {
            var scorer = new GameObject("Scorer").AddComponent<ApxScorer>();
            scorer.forceLimit = ForceLimit;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                scorer.Observe(body);
            }
            return scorer;
        }

        /// <summary>A recorder of the bodies in the order they step and of the scalpel, the only moving instrument.</summary>
        private static ApxRecorder CreateRecorder()
        {
            var recorder = new GameObject("Recorder").AddComponent<ApxRecorder>();
            recorder.bodies = ApxSoftBody.Active.ToArray();
            recorder.instruments = new[] { _scalpel.transform };
            return recorder;
        }

        /// <summary>
        /// Largest opening of an incision: the distance of each pair of wall points less the one before the cut; negative where
        /// the walls press into each other everywhere.
        /// </summary>
        private static float Opening(ApxSoftBody body, List<Vector2Int> walls, float[] rest)
        {
            float opening = float.MinValue;
            for (int i = 0; i < walls.Count; i++)
            {
                opening = Mathf.Max(opening,
                    Vector3.Distance(body.MaterialPointPosition(walls[i].x), body.MaterialPointPosition(walls[i].y)) - rest[i]);
            }
            return opening;
        }

        /// <summary>
        /// A stitch drawn as a thread over the skin: a thin tube from one material point at the surface in an arc over the
        /// incision to the other.
        /// </summary>
        private sealed class StitchLine
        {
            private const int Segments = 8;
            private const int Sides = 6;
            /// <summary>Height of the arc over the incision and radius of the thread, m.</summary>
            private const float Lift = 0.0012f;
            private const float Radius = 0.00025f;

            private Mesh _mesh;
            private ApxSoftBody _body;
            private int _a, _b;
            private readonly Vector3[] _vertices = new Vector3[(Segments + 1) * Sides];

            public static StitchLine Create(ApxSoftBody body, int a, int b)
            {
                var thread = new GameObject("Stitch", typeof(MeshFilter), typeof(MeshRenderer));
                thread.GetComponent<MeshRenderer>().sharedMaterial =
                    new Material(Shader.Find("Unlit/Color")) { color = new Color(0.25f, 0.12f, 0.45f) };
                var stitch = new StitchLine { _mesh = new Mesh(), _body = body, _a = a, _b = b };
                var triangles = new int[Segments * Sides * 6];
                for (int i = 0, t = 0; i < Segments; i++)
                for (int k = 0; k < Sides; k++)
                {
                    int p = i * Sides + k, q = i * Sides + (k + 1) % Sides;
                    triangles[t++] = p;
                    triangles[t++] = p + Sides;
                    triangles[t++] = q;
                    triangles[t++] = q;
                    triangles[t++] = p + Sides;
                    triangles[t++] = q + Sides;
                }
                stitch.Update();
                stitch._mesh.triangles = triangles;
                thread.GetComponent<MeshFilter>().sharedMesh = stitch._mesh;
                return stitch;
            }

            public void Update()
            {
                Vector3 a = _body.MaterialPointPosition(_a), b = _body.MaterialPointPosition(_b);
                Vector3 side = Vector3.Cross(b - a, Vector3.up).normalized;
                for (int i = 0; i <= Segments; i++)
                {
                    float t = i / (float)Segments;
                    Vector3 center = Vector3.Lerp(a, b, t) + Vector3.up * (Lift * Mathf.Sin(Mathf.PI * t));
                    Vector3 tangent = (b - a) + Vector3.up * (Lift * Mathf.PI * Mathf.Cos(Mathf.PI * t));
                    Vector3 normal = Vector3.Cross(side, tangent).normalized;
                    for (int k = 0; k < Sides; k++)
                    {
                        float angle = 2.0f * Mathf.PI * k / Sides;
                        _vertices[i * Sides + k] = center + Radius * (Mathf.Cos(angle) * side + Mathf.Sin(angle) * normal);
                    }
                }
                _mesh.vertices = _vertices;
                _mesh.RecalculateBounds();
            }
        }

        private static void DebugShot(string name, ApxDebugLayers layers, bool hideSurfaces, Vector3 position, Vector3 target)
        {
            if (_debugView == null)
            {
                return;
            }
            _debugView.layers = layers;
            _debugView.hideSurfaces = hideSurfaces;
            _debugView.xray = false;
            _debugView.nodeColoring = ApxNodeColoring.Piece;
            _debugView.enabled = true;
            // The grid fades with the camera distance: place the camera before collecting the lines.
            _camera.transform.SetPositionAndRotation(position, Quaternion.LookRotation(target - position));
            _debugView.Refresh();
            Shot(name, position, target);
            _debugView.enabled = false;
        }

        /// <summary>Height of the topmost real node below <paramref name="above"/>.</summary>
        private static float SurfaceHeight(ApxSoftBody body, Vector3 above)
        {
            SoftBodySimulation sim = body.Simulation;
            int node = sim.PickNode(above, Vector3.down, sim.CellSize);
            return node >= 0 ? sim.Positions[node].y : above.y;
        }

        /// <summary>Height of the rendered skin at (<paramref name="x"/>, <paramref name="z"/>): its highest vertex within 2 mm.</summary>
        private static float SkinHeight(ApxSoftBody body, float x, float z)
        {
            float height = float.MinValue;
            Matrix4x4 toWorld = body.transform.localToWorldMatrix;
            foreach (Vector3 v in body.Surface.Mesh.vertices)
            {
                Vector3 p = toWorld.MultiplyPoint3x4(v);
                if (Mathf.Abs(p.x - x) < 0.002f && Mathf.Abs(p.z - z) < 0.002f)
                {
                    height = Mathf.Max(height, p.y);
                }
            }
            return height;
        }

        private static ApxSoftBody Find(string name) => GameObject.Find(name).GetComponent<ApxSoftBody>();

        private static void Run(int frames)
        {
            for (int f = 0; f < frames; f++)
            {
                Frame();
            }
        }

        private static void Frame()
        {
            _recorder.Step();
            StepScene();
        }

        /// <summary>A fixed step after the recorder's: the scalpel, the bodies and the rendered meshes.</summary>
        private static void StepScene()
        {
            _scalpel.Sweep();
            foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
            {
                body.Simulate(Dt);
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                body.UpdateRender();
            }
            foreach (StitchLine line in _stitchLines)
            {
                line.Update();
            }
        }

        /// <summary>Moves a blade with local edge (edgeStart, edgeEnd) from <paramref name="from"/> to <paramref name="to"/>.</summary>
        private static void Stroke(Vector3 from, Vector3 to, Vector3 edgeStart, Vector3 edgeEnd, int frames)
        {
            _scalpel.edgeStart = edgeStart;
            _scalpel.edgeEnd = edgeEnd;
            _scalpel.transform.SetPositionAndRotation(from, Quaternion.identity);
            _scalpel.cutting = true;
            _scalpel.ResetSweep();
            int severed = 0;
            for (int f = 0; f <= frames; f++)
            {
                _scalpel.transform.position = Vector3.Lerp(from, to, f / (float)frames);
                Frame();
                severed += _scalpel.LastSeveredEdges;
            }
            _scalpel.cutting = false;
            Debug.Log($"[ApxCapture] stroke {from} -> {to}: severed {severed} edges");
        }

        private static void Drag(ApxSoftBody body, Vector3 grabPoint, Vector3 target, int frames, float radius)
        {
            var ray = new Ray(grabPoint + Vector3.up * 0.2f, Vector3.down);
            if (!body.BeginGrab(ray, radius, out Vector3 start))
            {
                Debug.LogWarning($"[ApxCapture] grab on {body.name} missed");
                return;
            }
            int grabbed = 0;
            for (int i = 0; i < body.Simulation.NodeCount; i++)
            {
                grabbed += body.Simulation.KinematicId[i] == SoftBodySimulation.GrabFrame ? 1 : 0;
            }
            Debug.Log($"[ApxCapture] grab on {body.name} at {start}: {grabbed} nodes");
            for (int f = 1; f <= frames; f++)
            {
                body.MoveGrab(Vector3.Lerp(start, target, f / (float)frames));
                Frame();
            }
        }

        private static void Release(ApxSoftBody body) => body.EndGrab();

        private static void Log(string stage)
        {
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = body.Simulation;
                Debug.Log($"[ApxCapture] {stage} | {body.name}: pieces [{string.Join(",", sim.PieceSizes())}] cells {sim.CellCount} " +
                          $"copies {sim.CopyCount} nodes {sim.NodeCount} cutTris {body.Surface.CutTriangleCount} " +
                          $"step {sim.LastStepMilliseconds:0.00} ms topo {sim.LastTopologyMilliseconds:0.00} ms");
            }
        }

        private static void Shot(string name, Vector3 position, Vector3 target)
        {
            _camera.transform.SetPositionAndRotation(position, Quaternion.LookRotation(target - position));
            var rt = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32) { antiAliasing = 4 };
            _camera.targetTexture = rt;
            _camera.Render();
            RenderTexture.active = rt;
            var texture = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            texture.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            texture.Apply();
            File.WriteAllBytes(Path.Combine(_directory, name + ".png"), texture.EncodeToPNG());
            RenderTexture.active = null;
            _camera.targetTexture = null;
            UnityEngine.Object.DestroyImmediate(texture);
            UnityEngine.Object.DestroyImmediate(rt);
        }
    }
}
