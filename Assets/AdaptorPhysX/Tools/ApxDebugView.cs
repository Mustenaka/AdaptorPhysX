using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace AdaptorPhysX.Tools
{
    /// <summary>What <see cref="ApxDebugView"/> draws.</summary>
    [System.Flags]
    public enum ApxDebugLayers
    {
        None = 0,
        /// <summary>Reference grid (1 cm, every tenth line brighter) with the world axes.</summary>
        Grid = 1 << 0,
        /// <summary>Node bounds of each body: orange awake, grey sleeping.</summary>
        Bounds = 1 << 1,
        /// <summary>Uncut lattice edges between real corners, coloured per node.</summary>
        Lattice = 1 << 2,
        /// <summary>Real nodes, coloured per node.</summary>
        Nodes = 1 << 3,
        /// <summary>Severed edges: the crossing point on each side and, up to three cells, the gap between them.</summary>
        Cuts = 1 << 4,
        /// <summary>Forces on colliders (red), the grab (yellow) and pin regions (blue).</summary>
        Forces = 1 << 5,
        /// <summary>Scalpel edges and the quad swept in the last step.</summary>
        Blades = 1 << 6,
    }

    /// <summary>Node colours of the lattice and node layers.</summary>
    public enum ApxNodeColoring
    {
        /// <summary>One colour per material piece.</summary>
        Piece,
        /// <summary>Free nodes light grey, grabbed yellow, pinned blue.</summary>
        Frame,
        /// <summary>Blue (still) to red (<see cref="ApxDebugView.speedRange"/> and faster).</summary>
        Speed,
        Uniform,
    }

    /// <summary>Adds lines to the debug view each frame (tools, camera rig).</summary>
    public interface IApxDebugDrawer
    {
        void DrawDebug(ApxDebugView view);
    }

    /// <summary>
    /// Reference lines for inspecting soft bodies in the Game and Scene views, in the manner of
    /// Obi's particle and blueprint visualisations: grid, lattice, nodes, cuts, forces and blades
    /// (<see cref="layers"/>), optionally on top of the surfaces (<see cref="xray"/>) or with the
    /// surfaces hidden. Drawn with GL after each camera renders (built-in render pipeline).
    /// </summary>
    [ExecuteAlways]
    [DefaultExecutionOrder(1000)]
    public sealed class ApxDebugView : MonoBehaviour
    {
        public ApxDebugLayers layers = ApxDebugLayers.Grid | ApxDebugLayers.Forces | ApxDebugLayers.Blades;
        public ApxNodeColoring nodeColoring = ApxNodeColoring.Piece;
        [Tooltip("Draw the lattice, nodes and cuts on top of the surfaces.")]
        public bool xray;
        [Tooltip("Hide the rendered soft-body surfaces, e.g. to look at the lattice.")]
        public bool hideSurfaces;
        [Tooltip("Node speed shown in red, m/s.")]
        [Min(1e-4f)] public float speedRange = 0.05f;
        [Tooltip("Arrow length per newton, metres.")]
        [Min(0.0f)] public float forceScale = 0.05f;
        [Tooltip("Longest force arrow, metres.")]
        [Min(0.0f)] public float maxArrow = 0.15f;
        [Tooltip("Write the force magnitudes next to the arrows (Game view, Play mode).")]
        public bool forceLabels = true;

        [Header("Grid")]
        public Vector3 gridCenter = new Vector3(0.0f, 0.0005f, 0.0f);
        [Min(0.01f)] public float gridSize = 1.0f;
        [Min(0.001f)] public float gridSpacing = 0.01f;
        [Min(1)] public int gridMajor = 10;

        /// <summary>Drawers asked for lines every frame (tools, camera rig).</summary>
        public static readonly List<IApxDebugDrawer> Drawers = new List<IApxDebugDrawer>();

        /// <summary>Lines hidden by the surfaces in front of them.</summary>
        public ApxDebugLines Lines { get; } = new ApxDebugLines();
        /// <summary>Lines drawn on top of everything.</summary>
        public ApxDebugLines Overlay { get; } = new ApxDebugLines();

        private sealed class BodyView
        {
            public int Topology = -1;
            public Mesh Lattice;
            /// <summary>Node pairs of the uncut lattice edges between real corners.</summary>
            public int[] Edges = System.Array.Empty<int>();
            /// <summary>Severed edges, each once: (cell, edge).</summary>
            public int2[] Cuts = System.Array.Empty<int2>();
            public Color32[] Colors = System.Array.Empty<Color32>();
            public bool LatticeDrawn;
        }

        private readonly Dictionary<ApxSoftBody, BodyView> _views = new Dictionary<ApxSoftBody, BodyView>();
        private readonly List<ApxSoftBody> _stale = new List<ApxSoftBody>();
        private readonly List<KeyValuePair<Vector3, string>> _labels = new List<KeyValuePair<Vector3, string>>();
        private readonly HashSet<Renderer> _hidden = new HashSet<Renderer>();
        private Mesh _linesMesh;
        private Mesh _overlayMesh;
        private Material _depthMaterial;
        private Material _topMaterial;

        public bool Has(ApxDebugLayers layer) => (layers & layer) != 0;

        public void Toggle(ApxDebugLayers layer) => layers ^= layer;

        private void OnEnable()
        {
            _linesMesh = new Mesh { name = "Apx Debug Lines", hideFlags = HideFlags.HideAndDontSave };
            _overlayMesh = new Mesh { name = "Apx Debug Overlay", hideFlags = HideFlags.HideAndDontSave };
            _depthMaterial = ApxDebugLines.CreateMaterial(false);
            _topMaterial = ApxDebugLines.CreateMaterial(true);
        }

        private void OnDisable()
        {
            RestoreSurfaces();
            foreach (BodyView v in _views.Values)
            {
                DestroyImmediate(v.Lattice);
            }
            _views.Clear();
            DestroyImmediate(_linesMesh);
            DestroyImmediate(_overlayMesh);
            DestroyImmediate(_depthMaterial);
            DestroyImmediate(_topMaterial);
        }

        private void LateUpdate() => Refresh();

        /// <summary>Collects this frame's lines (called in LateUpdate; call it directly in edit-mode scripts).</summary>
        public void Refresh()
        {
            if (_linesMesh == null)
            {
                return;
            }
            Lines.Clear();
            Overlay.Clear();
            _labels.Clear();
            if (Has(ApxDebugLayers.Grid))
            {
                DrawGrid();
            }
            ApxDebugLines bodyLines = xray ? Overlay : Lines;
            _stale.Clear();
            _stale.AddRange(_views.Keys);
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = body.Simulation;
                if (sim == null)
                {
                    continue;
                }
                _stale.Remove(body);
                sim.CompleteStep();
                BodyView v = ViewOf(body);
                if (v.Topology != sim.TopologyVersion)
                {
                    Rebuild(v, sim);
                }
                if (Has(ApxDebugLayers.Bounds))
                {
                    Lines.WireBox(body.WorldBounds, body.IsSleeping ? new Color(0.6f, 0.6f, 0.6f, 0.6f) : new Color(1.0f, 0.6f, 0.1f, 0.8f));
                }
                bool lattice = Has(ApxDebugLayers.Lattice);
                bool nodes = Has(ApxDebugLayers.Nodes);
                if (lattice || nodes)
                {
                    Colorize(v, sim, body.pinColliders.Length);
                }
                v.LatticeDrawn = lattice;
                if (lattice)
                {
                    UploadLattice(v, sim);
                }
                if (nodes)
                {
                    DrawNodes(bodyLines, v, sim);
                }
                if (Has(ApxDebugLayers.Cuts))
                {
                    DrawCuts(bodyLines, v, sim);
                }
                if (Has(ApxDebugLayers.Forces))
                {
                    DrawForces(body);
                }
            }
            foreach (ApxSoftBody body in _stale)
            {
                DestroyImmediate(_views[body].Lattice);
                _views.Remove(body);
            }
            if (Has(ApxDebugLayers.Blades))
            {
                DrawBlades();
            }
            foreach (IApxDebugDrawer drawer in Drawers)
            {
                drawer.DrawDebug(this);
            }
            UpdateSurfaces();
            Lines.Upload(_linesMesh);
            Overlay.Upload(_overlayMesh);
        }

        /// <summary>Node pairs of the lattice edges drawn for <paramref name="body"/> (empty before the first refresh).</summary>
        public IReadOnlyList<int> LatticeEdges(ApxSoftBody body) =>
            _views.TryGetValue(body, out BodyView v) ? v.Edges : System.Array.Empty<int>();

        /// <summary>Severed lattice edges of <paramref name="body"/>, each counted once.</summary>
        public int CutCount(ApxSoftBody body) => _views.TryGetValue(body, out BodyView v) ? v.Cuts.Length : 0;

        private BodyView ViewOf(ApxSoftBody body)
        {
            if (!_views.TryGetValue(body, out BodyView v))
            {
                v = new BodyView { Lattice = new Mesh { name = "Apx Debug Lattice", hideFlags = HideFlags.HideAndDontSave } };
                _views.Add(body, v);
            }
            return v;
        }

        private static void Rebuild(BodyView v, SoftBodySimulation sim)
        {
            var edges = new List<int>();
            var seen = new HashSet<long>();
            int[] corners = Lattice.EdgeCorners;
            long nodes = sim.NodeCount;
            for (int q = 0; q < sim.CopyCount; q++)
            {
                CellCopy copy = sim.Copies[q];
                int severed = sim.Severed[copy.Cell];
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    int a = corners[2 * e];
                    int b = corners[2 * e + 1];
                    if (((severed >> e) & 1) != 0 || ((copy.Mask >> a) & 1) == 0 || ((copy.Mask >> b) & 1) == 0)
                    {
                        continue;
                    }
                    int na = sim.CopyNodes[q * 8 + a];
                    int nb = sim.CopyNodes[q * 8 + b];
                    if (seen.Add(math.min(na, nb) * nodes + math.max(na, nb)))
                    {
                        edges.Add(na);
                        edges.Add(nb);
                    }
                }
            }
            v.Edges = edges.ToArray();

            var cuts = new List<int2>();
            seen.Clear();
            int3 dims = sim.Grid.Dims + 1;
            long points = (long)dims.x * dims.y * dims.z;
            for (int c = 0; c < sim.CellCount; c++)
            {
                int severed = sim.Severed[c];
                if (severed == 0)
                {
                    continue;
                }
                int3 coord = sim.CellCoord[c];
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if (((severed >> e) & 1) == 0)
                    {
                        continue;
                    }
                    int3 pa = coord + Lattice.CornerOffset(corners[2 * e]);
                    int3 pb = coord + Lattice.CornerOffset(corners[2 * e + 1]);
                    long ga = pa.x + (long)dims.x * (pa.y + (long)dims.y * pa.z);
                    long gb = pb.x + (long)dims.x * (pb.y + (long)dims.y * pb.z);
                    if (seen.Add(ga * points + gb))
                    {
                        cuts.Add(new int2(c, e));
                    }
                }
            }
            v.Cuts = cuts.ToArray();

            v.Lattice.Clear();
            v.Lattice.indexFormat = sim.NodeCount > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            v.Lattice.SetVertices(sim.Positions);
            v.Colors = new Color32[sim.NodeCount];
            v.Lattice.SetColors(v.Colors);
            v.Lattice.SetIndices(v.Edges, MeshTopology.Lines, 0, false);
            v.Lattice.bounds = new Bounds(Vector3.zero, Vector3.one * 1e4f);
            v.Topology = sim.TopologyVersion;
        }

        private void Colorize(BodyView v, SoftBodySimulation sim, int pins)
        {
            for (int i = 0; i < sim.NodeCount; i++)
            {
                Color c;
                switch (nodeColoring)
                {
                    case ApxNodeColoring.Piece:
                        int piece = sim.PieceId[i];
                        c = piece < 0 ? Color.gray : Color.HSVToRGB(Mathf.Repeat(0.13f + piece * 0.618034f, 1.0f), 0.75f, 1.0f);
                        break;
                    case ApxNodeColoring.Frame:
                        int frame = sim.KinematicId[i];
                        c = frame < 0 ? new Color(0.85f, 0.85f, 0.85f)
                            : frame == SoftBodySimulation.GrabFrame || frame > pins ? new Color(1.0f, 0.9f, 0.1f)
                            : Color.HSVToRGB(Mathf.Repeat(0.55f + 0.07f * frame, 1.0f), 0.8f, 1.0f);
                        break;
                    case ApxNodeColoring.Speed:
                        float s = Mathf.Clamp01(math.length(sim.Velocities[i]) / speedRange);
                        c = Color.HSVToRGB(0.66f * (1.0f - s), 0.9f, 1.0f);
                        break;
                    default:
                        c = new Color(0.55f, 0.9f, 1.0f);
                        break;
                }
                c.a = 0.75f;
                v.Colors[i] = c;
            }
        }

        private static void UploadLattice(BodyView v, SoftBodySimulation sim)
        {
            v.Lattice.SetVertices(sim.Positions);
            v.Lattice.SetColors(v.Colors);
        }

        private static void DrawNodes(ApxDebugLines lines, BodyView v, SoftBodySimulation sim)
        {
            float size = 0.2f * sim.CellSize;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.NodeReal[i] != 0)
                {
                    lines.Cross(sim.Positions[i], size, v.Colors[i]);
                }
            }
        }

        private static void DrawCuts(ApxDebugLines lines, BodyView v, SoftBodySimulation sim)
        {
            int[] corners = Lattice.EdgeCorners;
            float size = 0.25f * sim.CellSize;
            var gap = new Color(1.0f, 0.15f, 0.1f, 0.9f);
            var side = new Color(1.0f, 0.6f, 0.1f, 0.9f);
            var crack = new Color(1.0f, 0.2f, 1.0f, 0.9f);
            foreach (int2 cut in v.Cuts)
            {
                int c = cut.x;
                int e = cut.y;
                int a = corners[2 * e];
                int b = corners[2 * e + 1];
                int2 range = sim.CellCopyRange[c];
                int qa = -1;
                int qb = -1;
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = sim.Copies[q].Mask;
                    if (qa < 0 && ((mask >> a) & 1) != 0)
                    {
                        qa = q;
                    }
                    if (qb < 0 && ((mask >> b) & 1) != 0)
                    {
                        qb = q;
                    }
                }
                if (qa < 0 || qb < 0)
                {
                    continue;
                }
                float t = sim.EdgeT[c * Lattice.EdgeCount + e];
                Vector3 xa = math.lerp(sim.Positions[sim.CopyNodes[qa * 8 + a]], sim.Positions[sim.CopyNodes[qa * 8 + b]], t);
                if (qa == qb)
                {
                    // Severed, but the corners stay connected around it: the end of a partial cut.
                    lines.Cross(xa, size, crack);
                    continue;
                }
                Vector3 xb = math.lerp(sim.Positions[sim.CopyNodes[qb * 8 + a]], sim.Positions[sim.CopyNodes[qb * 8 + b]], t);
                // The opening of the incision; pieces moved apart would only add long lines across the view.
                if ((xb - xa).sqrMagnitude < 9.0f * sim.CellSize * sim.CellSize)
                {
                    lines.Line(xa, xb, gap);
                }
                lines.Cross(xa, size, side);
                lines.Cross(xb, size, side);
            }
        }

        private void DrawForces(ApxSoftBody body)
        {
            foreach (KeyValuePair<Collider, ApxWrench> pair in body.ColliderWrenches)
            {
                DrawWrench(pair.Value, new Color(1.0f, 0.25f, 0.2f));
            }
            foreach (int handle in body.GrabHandles)
            {
                if (body.TryGetGrabWrench(handle, out ApxWrench grab))
                {
                    DrawWrench(grab, new Color(1.0f, 0.9f, 0.1f));
                }
            }
            for (int p = 0; p < body.pinColliders.Length; p++)
            {
                if (body.TryGetPinWrench(p, out ApxWrench pin))
                {
                    DrawWrench(pin, new Color(0.3f, 0.6f, 1.0f));
                }
            }
        }

        private void DrawWrench(ApxWrench wrench, Color color)
        {
            Vector3 force = wrench.Force;
            float magnitude = force.magnitude;
            Vector3 from = wrench.ContactCentroid;
            Overlay.Cross(from, 0.004f, color);
            if (magnitude < 1e-6f)
            {
                return;
            }
            Vector3 to = from + force / magnitude * Mathf.Min(magnitude * forceScale, maxArrow);
            Overlay.Arrow(from, to, color);
            _labels.Add(new KeyValuePair<Vector3, string>(to, magnitude < 0.1f ? $"{magnitude * 1000.0f:0.0} mN" : $"{magnitude:0.000} N"));
        }

        private void DrawBlades()
        {
            foreach (ApxScalpel scalpel in FindObjectsOfType<ApxScalpel>())
            {
                Transform t = scalpel.transform;
                Color color = scalpel.cutting ? new Color(1.0f, 0.2f, 0.15f) : new Color(0.2f, 0.9f, 1.0f);
                ApxBladePose.Draw(Overlay, t.TransformPoint(scalpel.edgeStart), t.TransformPoint(scalpel.edgeEnd), t.forward, color);
                if (scalpel.HasSweep)
                {
                    scalpel.GetLastSweep(out Vector3 s0, out Vector3 e0, out Vector3 s1, out Vector3 e1);
                    Overlay.Quad(s0, e0, e1, s1, new Color(1.0f, 0.55f, 0.1f, 0.8f));
                }
            }
        }

        private void DrawGrid()
        {
            int half = Mathf.Max(1, Mathf.RoundToInt(0.5f * gridSize / gridSpacing));
            float extent = half * gridSpacing;
            // Minor lines fade out with the viewing distance (20 to 60 spacings), where they would merge
            // into a bright sheet.
            float fade = 1.0f;
            Camera camera = Camera.main;
            if (camera != null)
            {
                Transform t = camera.transform;
                float height = t.position.y - gridCenter.y;
                float distance = t.forward.y < -1e-3f ? height / -t.forward.y : Mathf.Abs(height);
                fade = Mathf.Clamp01(1.5f - distance / (40.0f * gridSpacing));
            }
            var minor = new Color(1.0f, 1.0f, 1.0f, 0.07f * fade);
            var major = new Color(1.0f, 1.0f, 1.0f, 0.22f);
            for (int i = -half; i <= half; i++)
            {
                float x = i * gridSpacing;
                Color c = i % gridMajor == 0 ? major : minor;
                if (c.a <= 0.0f && i != 0)
                {
                    continue;
                }
                Lines.Line(gridCenter + new Vector3(x, 0.0f, -extent), gridCenter + new Vector3(x, 0.0f, extent),
                    i == 0 ? new Color(0.3f, 0.5f, 1.0f, 0.8f) : c);
                Lines.Line(gridCenter + new Vector3(-extent, 0.0f, x), gridCenter + new Vector3(extent, 0.0f, x),
                    i == 0 ? new Color(1.0f, 0.3f, 0.3f, 0.8f) : c);
            }
            Lines.Line(gridCenter, gridCenter + Vector3.up * (10.0f * gridSpacing), new Color(0.3f, 1.0f, 0.3f, 0.8f));
        }

        private void UpdateSurfaces()
        {
            if (!hideSurfaces)
            {
                RestoreSurfaces();
                return;
            }
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                if (body.TryGetComponent(out MeshRenderer renderer) && renderer.enabled)
                {
                    renderer.enabled = false;
                    _hidden.Add(renderer);
                }
            }
        }

        private void RestoreSurfaces()
        {
            foreach (Renderer renderer in _hidden)
            {
                if (renderer != null)
                {
                    renderer.enabled = true;
                }
            }
            _hidden.Clear();
        }

        private void OnRenderObject()
        {
            if (_depthMaterial == null)
            {
                return;
            }
            Material bodyMaterial = xray ? _topMaterial : _depthMaterial;
            foreach (BodyView v in _views.Values)
            {
                if (v.LatticeDrawn && v.Edges.Length > 0)
                {
                    bodyMaterial.SetPass(0);
                    Graphics.DrawMeshNow(v.Lattice, Matrix4x4.identity);
                }
            }
            if (_linesMesh.vertexCount > 0)
            {
                _depthMaterial.SetPass(0);
                Graphics.DrawMeshNow(_linesMesh, Matrix4x4.identity);
            }
            if (_overlayMesh.vertexCount > 0)
            {
                _topMaterial.SetPass(0);
                Graphics.DrawMeshNow(_overlayMesh, Matrix4x4.identity);
            }
        }

        private void OnGUI()
        {
            if (!forceLabels || !Has(ApxDebugLayers.Forces) || !Application.isPlaying || Camera.main == null)
            {
                return;
            }
            Camera camera = Camera.main;
            foreach (KeyValuePair<Vector3, string> label in _labels)
            {
                Vector3 p = camera.WorldToScreenPoint(label.Key);
                if (p.z > 0.0f)
                {
                    GUI.Label(new Rect(p.x + 4.0f, Screen.height - p.y - 10.0f, 120.0f, 20.0f), label.Value);
                }
            }
        }
    }
}
