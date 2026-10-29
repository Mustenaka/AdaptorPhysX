using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace AdaptorPhysX.Tools
{
    /// <summary>
    /// Coloured line segments collected for one frame and drawn as a single line mesh
    /// (see <see cref="ApxDebugView"/>). World space.
    /// </summary>
    public sealed class ApxDebugLines
    {
        private readonly List<Vector3> _vertices = new List<Vector3>();
        private readonly List<Color32> _colors = new List<Color32>();
        private readonly List<int> _indices = new List<int>();

        public int Count => _vertices.Count / 2;

        public void Clear()
        {
            _vertices.Clear();
            _colors.Clear();
        }

        public void Line(Vector3 a, Vector3 b, Color color)
        {
            Color32 c = color;
            _vertices.Add(a);
            _vertices.Add(b);
            _colors.Add(c);
            _colors.Add(c);
        }

        public void Polyline(IReadOnlyList<Vector3> points, Color color)
        {
            for (int i = 1; i < points.Count; i++)
            {
                Line(points[i - 1], points[i], color);
            }
        }

        /// <summary>Three axis-aligned strokes of length <paramref name="size"/> through <paramref name="p"/>.</summary>
        public void Cross(Vector3 p, float size, Color color)
        {
            float h = 0.5f * size;
            Line(p - new Vector3(h, 0.0f, 0.0f), p + new Vector3(h, 0.0f, 0.0f), color);
            Line(p - new Vector3(0.0f, h, 0.0f), p + new Vector3(0.0f, h, 0.0f), color);
            Line(p - new Vector3(0.0f, 0.0f, h), p + new Vector3(0.0f, 0.0f, h), color);
        }

        /// <summary>Line from <paramref name="from"/> to <paramref name="to"/> with a four-stroke head.</summary>
        public void Arrow(Vector3 from, Vector3 to, Color color)
        {
            Vector3 d = to - from;
            float length = d.magnitude;
            if (length < 1e-6f)
            {
                return;
            }
            Line(from, to, color);
            Vector3 dir = d / length;
            Vector3 side = Vector3.Cross(dir, Mathf.Abs(dir.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
            Vector3 up = Vector3.Cross(side, dir);
            float head = Mathf.Min(0.25f * length, 0.01f);
            Vector3 back = to - dir * head;
            Line(to, back + side * 0.4f * head, color);
            Line(to, back - side * 0.4f * head, color);
            Line(to, back + up * 0.4f * head, color);
            Line(to, back - up * 0.4f * head, color);
        }

        public void Circle(Vector3 center, Vector3 normal, float radius, Color color, int segments = 32)
        {
            normal = normal.normalized;
            Vector3 u = Vector3.Cross(normal, Mathf.Abs(normal.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
            Vector3 v = Vector3.Cross(normal, u);
            Vector3 previous = center + u * radius;
            for (int i = 1; i <= segments; i++)
            {
                float a = 2.0f * Mathf.PI * i / segments;
                Vector3 p = center + (u * Mathf.Cos(a) + v * Mathf.Sin(a)) * radius;
                Line(previous, p, color);
                previous = p;
            }
        }

        public void WireSphere(Vector3 center, float radius, Color color, int segments = 32)
        {
            Circle(center, Vector3.up, radius, color, segments);
            Circle(center, Vector3.right, radius, color, segments);
            Circle(center, Vector3.forward, radius, color, segments);
        }

        public void WireBox(Bounds bounds, Color color) => WireBox(bounds.center, Quaternion.identity, bounds.extents, color);

        public void WireBox(Vector3 center, Quaternion rotation, Vector3 halfExtents, Color color)
        {
            var corners = new Vector3[8];
            for (int c = 0; c < 8; c++)
            {
                var offset = new Vector3((c & 1) == 0 ? -1.0f : 1.0f, (c & 2) == 0 ? -1.0f : 1.0f, (c & 4) == 0 ? -1.0f : 1.0f);
                corners[c] = center + rotation * Vector3.Scale(offset, halfExtents);
            }
            for (int c = 0; c < 8; c++)
            {
                for (int axis = 1; axis <= 4; axis <<= 1)
                {
                    if ((c & axis) == 0)
                    {
                        Line(corners[c], corners[c | axis], color);
                    }
                }
            }
        }

        /// <summary>Outline of the quadrilateral a, b, c, d.</summary>
        public void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color color)
        {
            Line(a, b, color);
            Line(b, c, color);
            Line(c, d, color);
            Line(d, a, color);
        }

        /// <summary>Copies the lines into <paramref name="mesh"/> (line topology).</summary>
        public void Upload(Mesh mesh)
        {
            mesh.Clear();
            int count = _vertices.Count;
            if (count == 0)
            {
                return;
            }
            mesh.indexFormat = count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            mesh.SetVertices(_vertices);
            mesh.SetColors(_colors);
            while (_indices.Count < count)
            {
                _indices.Add(_indices.Count);
            }
            mesh.SetIndices(_indices, 0, count, MeshTopology.Lines, 0, false);
            mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 1e4f);
        }

        /// <summary>Unlit vertex-coloured line material; <paramref name="onTop"/> ignores depth.</summary>
        public static Material CreateMaterial(bool onTop)
        {
            var material = new Material(Shader.Find("Hidden/Internal-Colored")) { hideFlags = HideFlags.HideAndDontSave };
            material.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
            material.SetInt("_Cull", (int)CullMode.Off);
            material.SetInt("_ZWrite", 0);
            material.SetInt("_ZTest", (int)(onTop ? CompareFunction.Always : CompareFunction.LessEqual));
            return material;
        }
    }
}
