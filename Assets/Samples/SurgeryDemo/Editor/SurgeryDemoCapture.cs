using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Runs the standard scenarios (ROADMAP S1-S5) on the demo scene in edit mode with a
    /// scripted scalpel and forceps, and writes a PNG per stage. Batch use:
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.SurgeryDemoCapture.CaptureBatch -apxCaptureDir &lt;dir&gt;
    /// </summary>
    public static class SurgeryDemoCapture
    {
        private const string ScenePath = "Assets/Samples/SurgeryDemo/SurgeryDemo.unity";
        private const float Dt = 1.0f / 60.0f;

        private static Camera _camera;
        private static ApxScalpel _scalpel;
        private static string _directory;

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
            try
            {
                RunScenarios();
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

        private static void RunScenarios()
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
            _camera = Camera.main;
            _scalpel = UnityEngine.Object.FindObjectOfType<ApxScalpel>();
            ApxSoftBody block = Find("Tissue Block");
            ApxSoftBody flap = Find("Skin Flap");
            ApxSoftBody liver = Find("Liver Lobe");
            ApxSoftBody pad = Find("Tissue Pad");

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
            Stroke(new Vector3(-0.012f, padTop + 0.01f, -0.21f), new Vector3(0.012f, padTop + 0.01f, -0.21f), Vector3.zero,
                new Vector3(0.0f, -0.018f, 0.0f), 30);
            Run(60);
            Log("S6 small incision");
            Shot("S6a_pad_incision", new Vector3(0.0f, padTop + 0.06f, -0.27f), new Vector3(0.0f, padTop, -0.21f));
            Drag(pad, new Vector3(0.0f, padTop, -0.2145f), new Vector3(0.0f, padTop + 0.006f, -0.222f), 40, 0.005f);
            Log("S6 retraction");
            Shot("S6b_pad_retracted", new Vector3(0.0f, padTop + 0.05f, -0.255f), new Vector3(0.0f, padTop - 0.004f, -0.21f));
            Release(pad);
            Shot("99_overview", new Vector3(0.0f, 0.32f, -0.42f), new Vector3(0.0f, 0.03f, 0.0f));
        }

        /// <summary>Height of the topmost real node below <paramref name="above"/>.</summary>
        private static float SurfaceHeight(ApxSoftBody body, Vector3 above)
        {
            SoftBodySimulation sim = body.Simulation;
            int node = sim.PickNode(above, Vector3.down, sim.CellSize);
            return node >= 0 ? sim.Positions[node].y : above.y;
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
            _scalpel.Sweep();
            foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
            {
                body.Simulate(Dt);
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                body.UpdateRender();
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
