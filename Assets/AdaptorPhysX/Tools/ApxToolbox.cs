using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Interactive test bench: switches between the tools (keys 1 to 9), passes them the pointer,
    /// keeps the camera from zooming while a tool uses the wheel, toggles the debug layers, pauses
    /// and steps the simulation, and shows a panel with the tool state, the bindings and the bodies.
    /// Reads the input manager (legacy input); without it the tools only run from scripts.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class ApxToolbox : MonoBehaviour, IApxDebugDrawer
    {
        public ApxTool[] tools = new ApxTool[0];
        public int activeTool;
        public ApxCameraRig cameraRig;
        public ApxDebugView debugView;
        public bool showPanel = true;
        public bool showHelp = true;
        [Tooltip("R reloads the active scene.")]
        public bool allowReload = true;

        public ApxTool ActiveTool => activeTool >= 0 && activeTool < tools.Length ? tools[activeTool] : null;
        public bool Paused { get; private set; }

        private static readonly (KeyCode Key, ApxDebugLayers Layer, string Name)[] LayerKeys =
        {
            (KeyCode.F2, ApxDebugLayers.Grid, "Grid"),
            (KeyCode.F3, ApxDebugLayers.Lattice, "Lattice"),
            (KeyCode.F4, ApxDebugLayers.Nodes, "Nodes"),
            (KeyCode.F5, ApxDebugLayers.Cuts, "Cuts"),
            (KeyCode.F6, ApxDebugLayers.Forces, "Forces"),
            (KeyCode.F7, ApxDebugLayers.Blades, "Blades"),
            (KeyCode.F8, ApxDebugLayers.Bounds, "Bounds"),
        };

        private const string CameraHelp =
            "RMB / Alt+LMB drag: orbit   MMB / Shift+RMB drag: pan\n" +
            "wheel: zoom to pointer   F: frame   Home: start view\n" +
            "keypad 1 / 3 / 7: front / right / top   5: start angle";

        private const string GeneralHelp =
            "1-3: tool   P: pause   N: step   R: reload   Tab: panel   F1: help\n" +
            "F2-F8: layers   F9: x-ray   F10: hide surfaces";

        private bool _toolPress;
        private int _stepFrame = -1;
        private Rect _panelRect;
        private Vector2 _scroll;
        private float _statsTime;
        private readonly List<string> _stats = new List<string>();

        private void Start()
        {
            for (int i = 0; i < tools.Length; i++)
            {
                if (tools[i] != null)
                {
                    tools[i].SetActive(i == activeTool);
                }
            }
        }

        private void OnEnable() => ApxDebugView.Drawers.Add(this);

        private void OnDisable()
        {
            ApxDebugView.Drawers.Remove(this);
            if (Paused)
            {
                SetPaused(false);
            }
        }

        private void Update()
        {
            if (_stepFrame >= 0 && Time.frameCount > _stepFrame)
            {
                Time.timeScale = 0.0f;
                _stepFrame = -1;
            }
            Camera camera = cameraRig != null ? cameraRig.Camera : Camera.main;
            if (camera == null)
            {
                return;
            }
            ApxToolInput input = ReadInput(camera);
            HandleKeys(input);
            ApxTool tool = ActiveTool;
            if (tool != null)
            {
                tool.HandleKeys(input.DeltaTime, input.Shift);
                tool.Tick(input);
            }
            if (cameraRig != null)
            {
                cameraRig.blockedRect = showPanel ? _panelRect : default;
                cameraRig.blockWheel = input.Ctrl || (tool != null && tool.WantsWheel(input));
            }
        }

        private ApxToolInput ReadInput(Camera camera)
        {
            var input = new ApxToolInput { DeltaTime = Time.unscaledDeltaTime, ViewForward = camera.transform.forward };
#if ENABLE_LEGACY_INPUT_MANAGER
            Vector2 pointer = Input.mousePosition;
            input.Position = pointer;
            input.Ray = camera.ScreenPointToRay(pointer);
            bool alt = Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt);
            input.Shift = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            input.Ctrl = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
            bool overPanel = showPanel && _panelRect.Contains(new Vector2(pointer.x, Screen.height - pointer.y));
            input.Down = Input.GetMouseButtonDown(0) && !alt && !overPanel;
            _toolPress |= input.Down;
            input.Held = _toolPress && Input.GetMouseButton(0);
            input.Up = _toolPress && Input.GetMouseButtonUp(0);
            if (!Input.GetMouseButton(0))
            {
                _toolPress = false;
            }
            input.Scroll = Input.mouseScrollDelta.y;
#endif
            return input;
        }

        private void HandleKeys(in ApxToolInput input)
        {
#if ENABLE_LEGACY_INPUT_MANAGER
            for (int i = 0; i < tools.Length && i < 9; i++)
            {
                if (Input.GetKeyDown(KeyCode.Alpha1 + i))
                {
                    Select(i);
                }
            }
            if (Input.GetKeyDown(KeyCode.F1)) showHelp = !showHelp;
            if (Input.GetKeyDown(KeyCode.Tab)) showPanel = !showPanel;
            if (Input.GetKeyDown(KeyCode.P)) SetPaused(!Paused);
            if (Input.GetKeyDown(KeyCode.N)) Step();
            if (allowReload && Input.GetKeyDown(KeyCode.R) && !input.Held)
            {
                SetPaused(false);
                Scene scene = SceneManager.GetActiveScene();
                SceneManager.LoadScene(scene.buildIndex >= 0 ? scene.buildIndex : 0);
            }
            if (debugView != null)
            {
                foreach (var (key, layer, _) in LayerKeys)
                {
                    if (Input.GetKeyDown(key))
                    {
                        debugView.Toggle(layer);
                    }
                }
                if (Input.GetKeyDown(KeyCode.F9)) debugView.xray = !debugView.xray;
                if (Input.GetKeyDown(KeyCode.F10)) debugView.hideSurfaces = !debugView.hideSurfaces;
            }
#endif
        }

        public void Select(int index)
        {
            if (index < 0 || index >= tools.Length || index == activeTool)
            {
                return;
            }
            ActiveTool?.SetActive(false);
            activeTool = index;
            ActiveTool?.SetActive(true);
        }

        public void SetPaused(bool paused)
        {
            Paused = paused;
            _stepFrame = -1;
            Time.timeScale = paused ? 0.0f : 1.0f;
        }

        /// <summary>While paused, runs the fixed steps of one frame (one step at the usual frame rate).</summary>
        public void Step()
        {
            if (!Paused)
            {
                return;
            }
            Time.timeScale = 1.0f;
            _stepFrame = Time.frameCount;
        }

        public void DrawDebug(ApxDebugView view)
        {
            foreach (ApxTool tool in tools)
            {
                if (tool != null && (tool.IsActive || tool is ApxMeasureTool))
                {
                    tool.Draw(view);
                }
            }
        }

        private void OnGUI()
        {
            if (!showPanel)
            {
                GUI.Label(new Rect(10, 10, 200, 22), "Tab: panel");
                return;
            }
            float width = Mathf.Min(420.0f, Screen.width - 20.0f);
            GUILayout.BeginArea(new Rect(10, 10, width, Screen.height - 20));
            GUILayout.BeginVertical(GUI.skin.box);
            var names = new string[tools.Length];
            for (int i = 0; i < tools.Length; i++)
            {
                names[i] = tools[i] != null ? $"{i + 1} {tools[i].DisplayName}" : "-";
            }
            int selected = GUILayout.Toolbar(activeTool, names);
            if (selected != activeTool)
            {
                Select(selected);
            }
            ApxTool tool = ActiveTool;
            if (tool != null)
            {
                GUILayout.Label(tool.Status);
            }
            if (showHelp)
            {
                if (tool != null)
                {
                    GUILayout.Label(tool.Help);
                }
                GUILayout.Label(CameraHelp);
                GUILayout.Label(GeneralHelp);
            }
            DrawViewButtons();
            DrawDebugControls();
            DrawSimulationControls();
            DrawBodies();
            GUILayout.EndVertical();
            if (Event.current.type == EventType.Repaint)
            {
                Rect r = GUILayoutUtility.GetLastRect();
                _panelRect = new Rect(r.x + 10.0f, r.y + 10.0f, r.width, r.height);
            }
            GUILayout.EndArea();
        }

        private void DrawViewButtons()
        {
            if (cameraRig == null)
            {
                return;
            }
            GUILayout.BeginHorizontal();
            GUILayout.Label("View", GUILayout.Width(52));
            if (GUILayout.Button("Front")) cameraRig.SetAngles(0.0f, 0.0f);
            if (GUILayout.Button("Right")) cameraRig.SetAngles(-90.0f, 0.0f);
            if (GUILayout.Button("Top")) cameraRig.SetAngles(0.0f, 89.0f);
            if (GUILayout.Button("Start")) cameraRig.ResetView();
            if (GUILayout.Button("All")) cameraRig.FrameAll();
            GUILayout.EndHorizontal();
        }

        private void DrawDebugControls()
        {
            if (debugView == null)
            {
                return;
            }
            GUILayout.BeginHorizontal();
            for (int i = 0; i < LayerKeys.Length; i++)
            {
                var (_, layer, name) = LayerKeys[i];
                bool on = GUILayout.Toggle(debugView.Has(layer), name);
                if (on != debugView.Has(layer))
                {
                    debugView.Toggle(layer);
                }
                if (i == 3)
                {
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                }
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            debugView.xray = GUILayout.Toggle(debugView.xray, "X-ray");
            debugView.hideSurfaces = GUILayout.Toggle(debugView.hideSurfaces, "Hide surfaces");
            GUILayout.EndHorizontal();
            if (debugView.Has(ApxDebugLayers.Lattice) || debugView.Has(ApxDebugLayers.Nodes))
            {
                debugView.nodeColoring = (ApxNodeColoring)GUILayout.Toolbar((int)debugView.nodeColoring,
                    new[] { "Pieces", "Pins/grab", "Speed", "Plain" });
            }
        }

        private void DrawSimulationControls()
        {
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(Paused ? "Resume (P)" : "Pause (P)"))
            {
                SetPaused(!Paused);
            }
            GUI.enabled = Paused;
            if (GUILayout.Button("Step (N)"))
            {
                Step();
            }
            GUI.enabled = true;
            GUILayout.EndHorizontal();
        }

        private void DrawBodies()
        {
            if (Time.unscaledTime >= _statsTime)
            {
                _statsTime = Time.unscaledTime + 0.5f;
                _stats.Clear();
                foreach (ApxSoftBody body in ApxSoftBody.Active)
                {
                    SoftBodySimulation sim = body.Simulation;
                    if (sim == null)
                    {
                        continue;
                    }
                    var line = new StringBuilder();
                    line.Append($"{body.name}: {sim.CountPieces()} pieces, {sim.CellCount} cells, {sim.CopyCount} copies, ");
                    line.Append($"{sim.NodeCount} nodes, step {sim.LastStepMilliseconds:0.0} ms");
                    if (body.IsSleeping)
                    {
                        line.Append(", sleeping");
                    }
                    _stats.Add(line.ToString());
                }
            }
            _scroll = GUILayout.BeginScrollView(_scroll, GUILayout.MaxHeight(110));
            foreach (string line in _stats)
            {
                GUILayout.Label(line);
            }
            GUILayout.EndScrollView();
        }

        /// <summary>
        /// Adds the camera rig (to <paramref name="camera"/>), the scalpel, forceps and measuring tools,
        /// the debug view and a toolbox to a new object "Apx Tools".
        /// </summary>
        public static ApxToolbox CreateDefault(Camera camera, ApxScalpel scalpel, ApxOrbit view)
        {
            var rig = camera.GetComponent<ApxCameraRig>();
            if (rig == null)
            {
                rig = camera.gameObject.AddComponent<ApxCameraRig>();
            }
            rig.view = view;
            var go = new GameObject("Apx Tools");
            var toolbox = go.AddComponent<ApxToolbox>();
            var scalpelTool = go.AddComponent<ApxScalpelTool>();
            scalpelTool.scalpel = scalpel;
            scalpel.cutting = false;
            toolbox.tools = new ApxTool[] { scalpelTool, go.AddComponent<ApxForcepsTool>(), go.AddComponent<ApxMeasureTool>() };
            toolbox.cameraRig = rig;
            toolbox.debugView = go.AddComponent<ApxDebugView>();
            return toolbox;
        }
    }
}
