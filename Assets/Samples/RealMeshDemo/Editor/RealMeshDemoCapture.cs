using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Runs the real mesh scenarios (S12) of docs/ROADMAP.md in edit mode and writes screenshots:
    /// the three models at rest, full cuts through Fertility and the dragon with one part pulled away, and the
    /// bunny's ears lifted with the forceps. Batch use:
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.RealMeshDemoCapture.CaptureBatch
    ///   -apxCaptureDir &lt;dir&gt;
    /// </summary>
    public static class RealMeshDemoCapture
    {
        private const float Dt = 1.0f / 60.0f;

        private static Camera _camera;
        private static ApxScalpel _scalpel;
        private static string _directory;

        [MenuItem("AdaptorPhysX/Capture Real Mesh Scenarios")]
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
                    EditorSceneManager.OpenScene(RealMeshDemoSceneBuilder.ScenePath);
                }
            }
        }

        [MenuItem("AdaptorPhysX/Capture Real Mesh Scenarios", true)]
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
            if (!File.Exists(RealMeshDemoSceneBuilder.ScenePath))
            {
                RealMeshDemoSceneBuilder.Build();
            }
            EditorSceneManager.OpenScene(RealMeshDemoSceneBuilder.ScenePath);
            try
            {
                RunScenarios();
            }
            finally
            {
                foreach (ApxSoftBody body in ApxSoftBody.Active.ToArray())
                {
                    body.Release();
                }
            }
        }

        private static void RunScenarios()
        {
            foreach (ApxSoftBody body in UnityEngine.Object.FindObjectsOfType<ApxSoftBody>())
            {
                body.Initialize();
                ApxSoftBody.InitializeTimings t = body.LastInitializeTimings;
                Debug.Log($"[ApxCapture] {body.name}: initialized in {t.Total:0} ms (prepare {t.Prepare:0}, voxelize {t.Voxelize:0}, " +
                          $"surface {t.Surface:0}), cells {body.Simulation.CellCount}, render vertices {body.Surface.Mesh.vertexCount}");
            }
            Physics.SyncTransforms();
            _camera = Camera.main;
            _scalpel = UnityEngine.Object.FindObjectOfType<ApxScalpel>();
            var debugView = UnityEngine.Object.FindObjectOfType<Tools.ApxDebugView>();
            if (debugView != null)
            {
                debugView.enabled = false;
            }
            ApxSoftBody bunny = Find("Scanned Bunny");
            ApxSoftBody fertility = Find("Fertility");
            ApxSoftBody dragon = Find("Dragon");

            Run(90);
            Log("settled");
            Shot("S12a_real_meshes", new Vector3(0.0f, 0.3f, -0.55f), new Vector3(0.0f, 0.06f, 0.0f));

            // Fertility: a vertical blade through the middle along z, then the -x half is pulled away.
            Bounds f = fertility.WorldBounds;
            Stroke(new Vector3(f.center.x, f.max.y + 0.02f, f.min.z - 0.02f), new Vector3(f.center.x, f.max.y + 0.02f, f.max.z + 0.02f),
                Vector3.zero, new Vector3(0.0f, -(f.size.y + 0.04f), 0.0f), 60);
            Run(20);
            Log("S12b Fertility cut");
            Vector3 grab = new Vector3(f.center.x - 0.25f * f.extents.x, f.max.y + 0.05f, f.center.z);
            Drag(fertility, grab, new Vector3(grab.x - 0.04f, SurfaceHeight(fertility, grab) + 0.03f, grab.z), 50, 0.012f);
            Log("S12b Fertility pulled");
            Shot("S12b_fertility_cut", new Vector3(f.center.x - 0.06f, 0.2f, -0.24f), new Vector3(f.center.x, 0.06f, 0.0f));
            Release(fertility);
            Run(30);

            // Dragon: a vertical blade across the body at a third of its length, then the +x part is pulled away.
            Bounds d = dragon.WorldBounds;
            float x = d.center.x + 0.15f * d.size.x;
            Stroke(new Vector3(x, d.max.y + 0.02f, d.min.z - 0.02f), new Vector3(x, d.max.y + 0.02f, d.max.z + 0.02f),
                Vector3.zero, new Vector3(0.0f, -(d.size.y + 0.04f), 0.0f), 60);
            Run(20);
            Log("S12c dragon cut");
            Vector3 part = new Vector3(x + 0.04f, d.max.y + 0.05f, d.center.z);
            Drag(dragon, part, new Vector3(part.x + 0.04f, SurfaceHeight(dragon, part) + 0.03f, part.z), 50, 0.012f);
            Log("S12c dragon pulled");
            Shot("S12c_dragon_cut", new Vector3(x - 0.03f, 0.2f, d.center.z - 0.24f), new Vector3(x + 0.02f, 0.04f, d.center.z));
            Release(dragon);
            Run(30);

            // Scanned bunny (upright): the forceps lift the tip of the ears by 3 cm.
            Bounds b = bunny.WorldBounds;
            int node = HighestNode(bunny);
            Vector3 ear = bunny.Simulation.Positions[node];
            Drag(bunny, ear, ear + new Vector3(0.0f, 0.03f, 0.0f), 50, 0.008f);
            Log("S12d bunny lifted");
            Shot("S12d_bunny_lifted", new Vector3(b.center.x + 0.05f, 0.18f, -0.32f), new Vector3(b.center.x, 0.08f, 0.0f));
            Release(bunny);
            Run(30);

            Shot("S12e_real_meshes_after", new Vector3(0.0f, 0.3f, -0.55f), new Vector3(0.0f, 0.06f, 0.0f));
        }

        private static int HighestNode(ApxSoftBody body)
        {
            SoftBodySimulation sim = body.Simulation;
            int best = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.NodeReal[i] != 0 && sim.Positions[i].y > sim.Positions[best].y)
                {
                    best = i;
                }
            }
            return best;
        }

        /// <summary>Height of the topmost real node below <paramref name="above"/>.</summary>
        private static float SurfaceHeight(ApxSoftBody body, Vector3 above)
        {
            SoftBodySimulation sim = body.Simulation;
            int node = sim.PickNode(new Vector3(above.x, above.y + 0.2f, above.z), Vector3.down, sim.CellSize);
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
                Bounds bounds = body.WorldBounds;
                Debug.Log($"[ApxCapture] {stage} | {body.name}: pieces [{string.Join(",", sim.PieceSizes())}] cells {sim.CellCount} " +
                          $"bounds {bounds.min.ToString("F3")} {bounds.max.ToString("F3")} max speed {sim.MaxSpeed:0.000} " +
                          $"step {sim.LastStepMilliseconds:0.00} ms render {body.Surface.LastTimings.Total:0.00} ms");
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
