using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Distance between two points of the tissue. Each click picks the node under the pointer; the
    /// points stay on their nodes as the tissue moves (e.g. to follow an incision opening), and are
    /// moved to the nearest real node of the same body after a cut renumbers the nodes.
    /// </summary>
    public sealed class ApxMeasureTool : ApxTool
    {
        private struct Anchor
        {
            public ApxSoftBody Body;
            public int Node;
            public int Topology;
            public Vector3 Position;
        }

        public override string DisplayName => "Measure";

        public override string Help =>
            "LMB: set point A, then B (a third click starts over)\n" +
            "Backspace: clear";

        public override string Status
        {
            get
            {
                if (PointCount < 2)
                {
                    return PointCount == 0 ? "click the first point" : "click the second point";
                }
                Vector3 d = _b.Position - _a.Position;
                return $"A-B {Distance * 1000.0f:0.0} mm   (dx {d.x * 1000.0f:0.0}, dy {d.y * 1000.0f:0.0}, dz {d.z * 1000.0f:0.0} mm)";
            }
        }

        public int PointCount { get; private set; }
        public float Distance => PointCount == 2 ? Vector3.Distance(_a.Position, _b.Position) : 0.0f;
        public Vector3 PointA => _a.Position;
        public Vector3 PointB => _b.Position;

        private Anchor _a;
        private Anchor _b;
        private bool _hover;
        private Vector3 _hoverPoint;

        public override void Tick(in ApxToolInput input)
        {
            if (input.Down && ApxToolUtil.PickSurface(input.Ray, out Vector3 point, out ApxSoftBody body, out int node))
            {
                if (PointCount == 2)
                {
                    PointCount = 0;
                }
                var anchor = new Anchor { Body = body, Node = node, Topology = body.Simulation.TopologyVersion, Position = point };
                if (PointCount == 0)
                {
                    _a = anchor;
                }
                else
                {
                    _b = anchor;
                }
                PointCount++;
            }
            Follow(ref _a);
            Follow(ref _b);
            _hover = ApxToolUtil.PickSurface(input.Ray, out _hoverPoint, out _, out _);
        }

        public override void HandleKeys(float deltaTime, bool shift)
        {
            if (KeyDown(KeyCode.Backspace))
            {
                PointCount = 0;
            }
        }

        protected override void OnDeactivate() => _hover = false;

        private static void Follow(ref Anchor anchor)
        {
            SoftBodySimulation sim = anchor.Body != null ? anchor.Body.Simulation : null;
            if (sim == null)
            {
                return;
            }
            if (anchor.Topology != sim.TopologyVersion || anchor.Node >= sim.NodeCount)
            {
                float best = float.MaxValue;
                float3 last = anchor.Position;
                for (int i = 0; i < sim.NodeCount; i++)
                {
                    float d = math.distancesq(sim.Positions[i], last);
                    if (sim.NodeReal[i] != 0 && d < best)
                    {
                        best = d;
                        anchor.Node = i;
                    }
                }
                anchor.Topology = sim.TopologyVersion;
            }
            anchor.Position = sim.Positions[anchor.Node];
        }

        public override void Draw(ApxDebugView view)
        {
            // Also while another tool is active: the points keep following the tissue.
            Follow(ref _a);
            Follow(ref _b);
            var color = new Color(0.4f, 1.0f, 0.4f, 0.95f);
            if (PointCount >= 1)
            {
                view.Overlay.Cross(_a.Position, 0.005f, color);
            }
            if (PointCount == 2)
            {
                view.Overlay.Cross(_b.Position, 0.005f, color);
                view.Overlay.Line(_a.Position, _b.Position, color);
            }
            if (_hover)
            {
                view.Overlay.Cross(_hoverPoint, 0.003f, new Color(color.r, color.g, color.b, 0.5f));
            }
        }
    }
}
