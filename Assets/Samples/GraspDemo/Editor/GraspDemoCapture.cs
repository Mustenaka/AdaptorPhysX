using System;
using System.IO;
using AdaptorPhysX.Tools;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Runs the grasp scenario (ROADMAP S13, TODO MED-10) on the grasp demo scene in edit mode: the grasper tool, driven by a
    /// scripted pointer, takes the near edge of the sheet, lifts it and turns it about its jaws, then lets go. Writes a PNG
    /// per stage; the run is recorded and replayed in the scene opened again (TODO MED-14). Batch use:
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.GraspDemoCapture.CaptureBatch -apxCaptureDir &lt;dir&gt;
    /// </summary>
    public static class GraspDemoCapture
    {
        private const float Dt = 1.0f / 60.0f;

        private static Camera _camera;
        private static string _directory;
        private static ApxGrasperTool _tool;
        private static ApxRecorder _recorder;

        [MenuItem("AdaptorPhysX/Capture Grasp Scenario")]
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
                    EditorSceneManager.OpenScene(GraspDemoSceneBuilder.ScenePath);
                }
            }
        }

        [MenuItem("AdaptorPhysX/Capture Grasp Scenario", true)]
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
            if (!File.Exists(GraspDemoSceneBuilder.ScenePath))
            {
                GraspDemoSceneBuilder.Build();
            }
            EditorSceneManager.OpenScene(GraspDemoSceneBuilder.ScenePath);
            try
            {
                ApxRecording recording = RunScenario();
                Replay(recording);
            }
            finally
            {
                foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
                {
                    body.Release();
                }
            }
        }

        private static ApxRecording RunScenario()
        {
            PrepareScene();
            _camera = Camera.main;
            var debugView = UnityEngine.Object.FindObjectOfType<ApxDebugView>();
            if (debugView != null)
            {
                debugView.enabled = false;
            }
            ApxSoftBody sheet = ApxSoftBody.Active[0];
            ApxGrasper grasper = _tool.grasper;
            _recorder.StartRecording();
            Run(60, default);
            Shot("S13a_sheet", new Vector3(0.1f, 0.13f, -0.2f), new Vector3(0.0f, 0.045f, 0.0f));

            // The pointer from the camera's side, on the middle of the near edge, stays where it was pressed.
            ApxToolInput press = EdgePointer(sheet, new Vector3(0.0f, 0.2f, -0.3f));
            ApxToolInput Pointer(bool down, bool held, bool up)
            {
                ApxToolInput input = press;
                input.Down = down;
                input.Held = held;
                input.Up = up;
                return input;
            }
            Run(1, Pointer(true, true, false));
            Run(40, Pointer(false, true, false));
            Log("S13 grasped", grasper, sheet);
            // Lifted by ten wheel steps of 2 mm, one every six frames.
            for (int f = 0; f < 90; f++)
            {
                ApxToolInput lift = Pointer(false, true, false);
                lift.Scroll = f < 60 && f % 6 == 0 ? 1.0f : 0.0f;
                Run(1, lift);
            }
            Log("S13 lifted", grasper, sheet);
            Shot("S13b_sheet_lifted", new Vector3(0.08f, 0.11f, -0.16f), new Vector3(0.0f, 0.055f, -0.03f));
            // Turned by 45 degrees about the jaws in half a second, and held for one.
            for (int f = 0; f < 90; f++)
            {
                _tool.Turn(0.0f, 0.0f, f < 30 ? 1.5f : 0.0f);
                Run(1, Pointer(false, true, false));
            }
            Log("S13 turned", grasper, sheet);
            Shot("S13c_sheet_turned", new Vector3(0.08f, 0.11f, -0.16f), new Vector3(0.0f, 0.055f, -0.03f));
            Shot("S13d_sheet_turned_close", new Vector3(0.05f, 0.09f, -0.13f), new Vector3(0.0f, 0.06f, -0.04f));
            // Released: the jaws open, let go and draw back.
            Run(1, Pointer(false, false, true));
            Run(90, default);
            Log("S13 released", grasper, sheet);
            Shot("S13e_sheet_released", new Vector3(0.08f, 0.11f, -0.16f), new Vector3(0.0f, 0.055f, -0.03f));
            ApxRecording recording = _recorder.StopRecording();
            File.WriteAllText(Path.Combine(_directory, "grasp-recording.json"), recording.ToJson());
            return recording;
        }

        /// <summary>
        /// Opens the scene again and replays the recording (TODO MED-14): the grasper takes the sheet again from its recorded
        /// poses and gaps, and the node positions agree with the recording at its checks.
        /// </summary>
        private static void Replay(ApxRecording recording)
        {
            foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
            {
                body.Release();
            }
            EditorSceneManager.OpenScene(GraspDemoSceneBuilder.ScenePath);
            PrepareScene();
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
            string result = $"[ApxCapture] replay: {steps} steps, {recording.Operations.Count} operations, {recording.Poses.Count} grasper " +
                            $"poses, {recording.Settings.Count} grasper settings; {_recorder.ChecksCompared} positions hashes " +
                            (_recorder.Diverged ? $"differ from step {_recorder.DivergedTick} on" : "as recorded");
            if (_recorder.Diverged || steps != recording.Steps)
            {
                Debug.LogError(result);
            }
            else
            {
                Debug.Log(result);
            }
        }

        /// <summary>Applies the demo shapes, initializes the bodies and finds the grasper tool; a recorder of the bodies and the grasper.</summary>
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
            _tool = UnityEngine.Object.FindObjectOfType<ApxGrasperTool>();
            _recorder = new GameObject("Recorder").AddComponent<ApxRecorder>();
            _recorder.bodies = ApxSoftBody.Active.ToArray();
            _recorder.instruments = new[] { _tool.grasper.transform };
        }

        /// <summary>A press on the near edge of the sheet at x = 0 from <paramref name="origin"/>.</summary>
        private static ApxToolInput EdgePointer(ApxSoftBody sheet, Vector3 origin)
        {
            SoftBodySimulation sim = sheet.Simulation;
            int top = sim.PickNode(new Vector3(0.0f, 0.3f, -0.03f), Vector3.down, sim.CellSize);
            int edge = sim.PickNode(new Vector3(0.0f, sim.Positions[top].y - 0.003f, -0.3f), Vector3.forward, sim.CellSize);
            Vector3 direction = ((Vector3)sim.Positions[edge] - origin).normalized;
            return new ApxToolInput { Ray = new Ray(origin, direction), ViewForward = direction, DeltaTime = Dt };
        }

        /// <summary>Frames with the same pointer input: the tool reads it, then a fixed step of the tool, the recorder and the scene.</summary>
        private static void Run(int frames, in ApxToolInput input)
        {
            for (int f = 0; f < frames; f++)
            {
                _tool.Tick(input);
                _tool.Advance(Dt);
                _recorder.Step();
                StepScene();
            }
        }

        /// <summary>A fixed step after the recorder's: the grasper, the bodies and the rendered meshes.</summary>
        private static void StepScene()
        {
            if (_tool.grasper.gameObject.activeSelf)
            {
                _tool.grasper.Step();
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
            {
                body.Simulate(Dt);
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                body.UpdateRender();
            }
        }

        private static void Log(string stage, ApxGrasper grasper, ApxSoftBody sheet)
        {
            grasper.TryGetWrench(out ApxWrench wrench);
            Debug.Log($"[ApxCapture] {stage}: {_tool.State}, held {grasper.HeldPoints} points, squeeze {grasper.Squeeze:0.000} N, " +
                      $"grip {grasper.GripCapacity:0.000} N, force on the grasper {wrench.Force} N, grasper at {grasper.transform.position * 1000} mm, " +
                      $"turned {_tool.Roll:0} deg; sheet top {sheet.WorldBounds.max.y * 1000:0.0} mm, max speed {sheet.Simulation.MaxSpeed:0.000} m/s");
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
