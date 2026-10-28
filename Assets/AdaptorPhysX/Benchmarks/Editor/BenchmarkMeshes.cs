using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace AdaptorPhysX.Benchmarks.Editor
{
    /// <summary>
    /// Closed test meshes for the benchmark. A mesh is read from <c>Benchmarks/Meshes/&lt;name&gt;.obj</c>;
    /// without the file, "box" and "liver" are generated. Every mesh is centred and scaled so that its
    /// longest axis has the requested size.
    /// </summary>
    public static class BenchmarkMeshes
    {
        public const string MeshDirectory = "Benchmarks/Meshes";

        /// <summary>Loads or generates <paramref name="name"/>; returns null when there is neither a file nor a generator.</summary>
        public static Mesh Load(string name, float size, MeshInfo info)
        {
            var timer = System.Diagnostics.Stopwatch.StartNew();
            string path = Path.Combine(MeshDirectory, name + ".obj");
            Vector3[] positions;
            int[] triangles;
            if (File.Exists(path))
            {
                ReadObj(path, out positions, out triangles);
                info.source = path.Replace('\\', '/');
            }
            else if (name == "box")
            {
                Mesh box = MeshUtil.CreateBox(new Vector3(0.15f, 0.06f, 0.1f), 12);
                positions = box.vertices;
                triangles = box.triangles;
                UnityEngine.Object.DestroyImmediate(box);
                info.source = "procedural";
            }
            else if (name == "liver")
            {
                CreateLiver(out positions, out triangles);
                info.source = "procedural";
            }
            else
            {
                return null;
            }

            Bounds bounds = Normalize(positions, size);
            MeshUtil.CountOpenEdges(Array.ConvertAll(positions, p => (float3)p), triangles,
                out info.boundaryEdges, out info.nonManifoldEdges);
            info.vertices = positions.Length;
            info.triangles = triangles.Length / 3;
            info.sizeM = new[] { Math.Round(bounds.size.x, 4), Math.Round(bounds.size.y, 4), Math.Round(bounds.size.z, 4) };
            var mesh = new Mesh { name = "Benchmark " + name, indexFormat = IndexFormat.UInt32 };
            mesh.vertices = positions;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            info.loadMs = Stats.Round(timer.Elapsed.TotalMilliseconds);
            return mesh;
        }

        /// <summary>Vertex positions and faces of a Wavefront OBJ; polygons are fanned, negative indices resolved.</summary>
        public static void ReadObj(string path, out Vector3[] positions, out int[] triangles)
        {
            var points = new List<Vector3>();
            var indices = new List<int>();
            var corners = new List<int>(8);
            foreach (string line in File.ReadLines(path))
            {
                if (line.Length < 2 || line[1] != ' ')
                {
                    continue;
                }
                if (line[0] == 'v')
                {
                    string[] p = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    points.Add(new Vector3(ParseFloat(p[1]), ParseFloat(p[2]), ParseFloat(p[3])));
                }
                else if (line[0] == 'f')
                {
                    string[] p = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                    corners.Clear();
                    for (int i = 1; i < p.Length; i++)
                    {
                        int slash = p[i].IndexOf('/');
                        int index = int.Parse(slash < 0 ? p[i] : p[i].Substring(0, slash), CultureInfo.InvariantCulture);
                        corners.Add(index > 0 ? index - 1 : points.Count + index);
                    }
                    for (int i = 1; i + 1 < corners.Count; i++)
                    {
                        indices.Add(corners[0]);
                        indices.Add(corners[i]);
                        indices.Add(corners[i + 1]);
                    }
                }
            }
            positions = points.ToArray();
            triangles = indices.ToArray();
        }

        /// <summary>
        /// Liver-like closed surface: a unit sphere mapped by (x, y, z) -> (x, a(x, sign y) y, b(x) z) with
        /// positive a and b, which keeps the surface free of self-intersections. Thick right lobe (+x),
        /// thin left lobe, convex top, flat underside and a shallow groove between the lobes on top.
        /// </summary>
        public static void CreateLiver(out Vector3[] positions, out int[] triangles, int rings = 48, int sectors = 96)
        {
            var points = new List<Vector3> { LiverPoint(Vector3.up) };
            for (int r = 1; r < rings; r++)
            {
                float phi = Mathf.PI * r / rings;
                for (int s = 0; s < sectors; s++)
                {
                    float theta = 2.0f * Mathf.PI * s / sectors;
                    points.Add(LiverPoint(new Vector3(Mathf.Sin(phi) * Mathf.Cos(theta), Mathf.Cos(phi),
                        Mathf.Sin(phi) * Mathf.Sin(theta))));
                }
            }
            points.Add(LiverPoint(Vector3.down));
            int south = points.Count - 1;
            int Ring(int r, int s) => 1 + (r - 1) * sectors + s % sectors;

            var indices = new List<int>();
            for (int s = 0; s < sectors; s++)
            {
                indices.AddRange(new[] { 0, Ring(1, s + 1), Ring(1, s) });
                for (int r = 1; r < rings - 1; r++)
                {
                    int i0 = Ring(r, s), i1 = Ring(r, s + 1), i2 = Ring(r + 1, s), i3 = Ring(r + 1, s + 1);
                    indices.AddRange(new[] { i0, i1, i2, i1, i3, i2 });
                }
                indices.AddRange(new[] { Ring(rings - 1, s), Ring(rings - 1, s + 1), south });
            }
            positions = points.ToArray();
            triangles = indices.ToArray();
        }

        private static Vector3 LiverPoint(Vector3 d)
        {
            float right = Mathf.SmoothStep(0.0f, 1.0f, Mathf.InverseLerp(-0.9f, 0.6f, d.x));
            float thickness = Mathf.Lerp(0.22f, 0.75f, right);
            float g = (d.x + 0.15f) / 0.12f;
            float groove = 1.0f - 0.3f * Mathf.Exp(-g * g);
            float y = d.y >= 0.0f ? d.y * thickness * groove : d.y * thickness * 0.45f;
            return new Vector3(d.x, y, d.z * Mathf.Lerp(0.55f, 0.8f, right));
        }

        /// <summary>Centres the positions and scales the longest axis to <paramref name="size"/>; returns the new bounds.</summary>
        private static Bounds Normalize(Vector3[] positions, float size)
        {
            var bounds = new Bounds(positions[0], Vector3.zero);
            foreach (Vector3 p in positions)
            {
                bounds.Encapsulate(p);
            }
            float scale = size / Mathf.Max(bounds.size.x, Mathf.Max(bounds.size.y, bounds.size.z));
            for (int i = 0; i < positions.Length; i++)
            {
                positions[i] = (positions[i] - bounds.center) * scale;
            }
            return new Bounds(Vector3.zero, bounds.size * scale);
        }

        private static float ParseFloat(string s) => float.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
    }
}
