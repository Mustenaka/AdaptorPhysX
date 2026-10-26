using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Runs the interaction scenarios (ROADMAP S8-S11) on the interaction demo scene in edit mode,
    /// with PhysX stepped by script, and writes a PNG per stage. Batch use:
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.InteractionDemoCapture.CaptureBatch -apxCaptureDir &lt;dir&gt;
    /// </summary>
    public static class InteractionDemoCapture
    {
        private const float Dt = 1.0f / 60.0f;

        private static Camera _camera;
        private static string _directory;
        private static ApxDemoPresser[] _pressers;

        [MenuItem("AdaptorPhysX/Capture Interaction Scenarios")]
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
                if (setup.Length > 0 && Array.TrueForAll(setup, s => !string.IsNullOrEmpty(s.path)))
                {
                    EditorSceneManager.RestoreSceneManagerSetup(setup);
                }
                else
                {
                    EditorSceneManager.OpenScene(InteractionDemoSceneBuilder.ScenePath);
                }
            }
        }

        [MenuItem("AdaptorPhysX/Capture Interaction Scenarios", true)]
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
            if (!File.Exists(InteractionDemoSceneBuilder.ScenePath))
            {
                InteractionDemoSceneBuilder.Build();
            }
            EditorSceneManager.OpenScene(InteractionDemoSceneBuilder.ScenePath);
            SimulationMode mode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            try
            {
                RunScenarios();
            }
            finally
            {
                Physics.simulationMode = mode;
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
            Physics.SyncTransforms();
            foreach (ApxSoftBody body in UnityEngine.Object.FindObjectsOfType<ApxSoftBody>())
            {
                body.Initialize();
            }
            _camera = Camera.main;
            _pressers = UnityEngine.Object.FindObjectsOfType<ApxDemoPresser>();
            Shot("I00_overview", new Vector3(0.0f, 0.36f, -0.5f), new Vector3(0.0f, 0.02f, 0.06f));

            // 1.33 s: the lobe and the ball have landed.
            Run(80);
            Log("S8 S9 settled");
            Shot("S8_bodies_stacked", new Vector3(-0.33f, 0.13f, -0.2f), new Vector3(-0.33f, 0.045f, 0.0f));
            Shot("S9_ball_on_tissue", new Vector3(-0.11f, 0.12f, -0.17f), new Vector3(-0.11f, 0.055f, 0.0f));

            // 2.5 s: the ring and the putty press hold at full depth; the sticky paddle is lifting.
            Run(70);
            Log("S10 S11 pressing");
            Shot("S10a_ring_pressed", new Vector3(0.11f, 0.08f, -0.12f), new Vector3(0.11f, 0.035f, 0.0f));
            Shot("S11a_putty_pressed", new Vector3(0.33f, 0.09f, -0.13f), new Vector3(0.33f, 0.025f, 0.0f));
            Shot("S11c_paddle_lifting", new Vector3(0.33f, 0.07f, 0.12f), new Vector3(0.33f, 0.035f, 0.22f));

            // 4 s: the ring is back up, the putty press lifted away.
            Run(90);
            Log("released");
            Shot("S10b_ring_lifted", new Vector3(0.11f, 0.15f, -0.12f), new Vector3(0.11f, 0.03f, 0.0f));
            Shot("S11b_putty_dent", new Vector3(0.33f, 0.14f, -0.13f), new Vector3(0.33f, 0.025f, 0.0f));
            Shot("I99_overview", new Vector3(0.0f, 0.36f, -0.5f), new Vector3(0.0f, 0.02f, 0.06f));
        }

        private static void Run(int frames)
        {
            for (int f = 0; f < frames; f++)
            {
                foreach (ApxDemoPresser presser in _pressers)
                {
                    presser.Advance(Dt);
                }
                Physics.SyncTransforms();
                foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
                {
                    body.Simulate(Dt);
                }
                foreach (ApxSoftBody body in ApxSoftBody.Active)
                {
                    body.UpdateRender();
                }
                Physics.Simulate(Dt);
            }
        }

        private static void Log(string stage)
        {
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = body.Simulation;
                Bounds bounds = body.WorldBounds;
                Debug.Log($"[ApxCapture] {stage} | {body.name}: bounds {bounds.min.y * 1000:0.0}..{bounds.max.y * 1000:0.0} mm " +
                          $"step {sim.LastStepMilliseconds:0.00} ms sleeping {body.IsSleeping}");
            }
            GameObject ball = GameObject.Find("Steel Ball");
            if (ball != null)
            {
                Debug.Log($"[ApxCapture] {stage} | Steel Ball y {ball.transform.position.y * 1000:0.0} mm");
            }
            // Surface under the ring's tube and in the middle of the putty: the indentations.
            Debug.Log($"[ApxCapture] {stage} | Pressed Block surface under the ring {SurfaceHeight("Pressed Block", new Vector3(0.135f, 0.2f, 0.0f)) * 1000:0.0} mm, " +
                      $"at the edge {SurfaceHeight("Pressed Block", new Vector3(0.155f, 0.2f, 0.04f)) * 1000:0.0} mm; " +
                      $"Putty middle {SurfaceHeight("Putty", new Vector3(0.33f, 0.2f, 0.0f)) * 1000:0.0} mm, " +
                      $"edge {SurfaceHeight("Putty", new Vector3(0.37f, 0.2f, 0.03f)) * 1000:0.0} mm");
        }

        /// <summary>Height of the topmost real node below <paramref name="above"/>.</summary>
        private static float SurfaceHeight(string body, Vector3 above)
        {
            SoftBodySimulation sim = GameObject.Find(body).GetComponent<ApxSoftBody>().Simulation;
            int node = sim.PickNode(above, Vector3.down, sim.CellSize);
            return node >= 0 ? sim.Positions[node].y : float.NaN;
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
