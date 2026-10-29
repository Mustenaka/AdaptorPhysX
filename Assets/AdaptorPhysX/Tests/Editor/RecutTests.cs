using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tests
{
    /// <summary>
    /// A second stroke beside or along an incision that has opened (TODO BUG-15): the hammock of the interaction
    /// demo, 16 x 2 x 7 cm, resolution 28 (5.7 mm cells), clamped at both ends and cut through 3 cm from one clamp,
    /// so the two parts sag apart before the second stroke. Bodies live in a preview scene, as in InteractionTests.
    /// </summary>
    public sealed class RecutTests
    {
        private const float Dt = 1.0f / 60.0f;
        /// <summary>The first stroke, across the band 3 cm from the right clamp.</summary>
        private const float CutX = 0.05f;
        private const float BandTop = 0.07f;

        private Scene _scene;
        private ApxSoftBody _band;
        private ApxScalpel _scalpel;

        [SetUp]
        public void SetUp()
        {
            _scene = EditorSceneManager.NewPreviewScene();

            var bandObject = InScene(new GameObject("Hammock", typeof(MeshFilter), typeof(MeshRenderer)));
            bandObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.16f, 0.02f, 0.07f), 12);
            bandObject.transform.position = new Vector3(0.0f, 0.06f, 0.0f);
            _band = bandObject.AddComponent<ApxSoftBody>();
            _band.resolution = 28;
            _band.youngsModulus = 8.0e4f;
            _band.pinColliders = new[] { Clamp(-0.08f), Clamp(0.08f) };
            // The demo's table, 5 cm below the band: cut-off parts come to rest on it.
            GameObject table = InScene(GameObject.CreatePrimitive(PrimitiveType.Cube));
            table.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            table.transform.localScale = new Vector3(0.6f, 0.1f, 0.4f);

            _scalpel = InScene(new GameObject("Scalpel")).AddComponent<ApxScalpel>();
            _scalpel.cutting = false;

            Physics.SyncTransforms();
            _band.Initialize();
        }

        [TearDown]
        public void TearDown()
        {
            _band.Release();
            EditorSceneManager.ClosePreviewScene(_scene);
        }

        private GameObject InScene(GameObject go)
        {
            SceneManager.MoveGameObjectToScene(go, _scene);
            return go;
        }

        /// <summary>
        /// The second stroke one cell beside the first with the blade leaning towards it, along it with the hand
        /// wavering, back along it at a slant, and one cell beside it upright, which cuts a strip off; offsets from the
        /// first stroke and the waver to either side in cells, back: from the far side of the band to the near one,
        /// pieces: pieces of at least SurfaceMesh.MinVisiblePieceNodes nodes afterwards. The lean stands in for the turn
        /// of the cut parts under the demo's steel ball. Before the fixes, the leaning stroke left skin stretched across
        /// the incision (3 triangles longer than two cells) and flaps of the wall that swung apart (rays through the
        /// incision crossing the surface an odd number of times 0.2 % to 3.5 %, TODO BUG-16), and the wavering and
        /// slanted ones shaved 8 and 6 fragments of 1–5 nodes off the walls (3.0 % and 2.8 %, TODO BUG-15).
        /// </summary>
        [TestCase(1.0f, 1.0f, 0.0f, 15.0f, false, 2, TestName = "SecondStrokeBesideAnIncisionLeaningTowardsIt")]
        [TestCase(0.0f, 0.0f, 0.4f, 0.0f, false, 2, TestName = "SecondStrokeWaveringAlongAnIncision")]
        [TestCase(0.3f, -0.3f, 0.3f, 0.0f, true, 2, TestName = "SecondStrokeBackAlongAnIncisionAtASlant")]
        [TestCase(1.0f, 1.0f, 0.0f, 0.0f, false, 3, TestName = "SecondStrokeOneCellBesideAnIncisionCutsAStripOff")]
        public void SecondStrokeLeavesTheIncisionWhole(float startOffset, float endOffset, float waver, float lean, bool back,
            int pieces)
        {
            SoftBodySimulation sim = _band.Simulation;
            float h = sim.CellSize;
            CutThrough();
            List<int> before = sim.PieceSizes();
            Assert.AreEqual(0, LongSkinTriangles(h), "first stroke: skin triangles longer than two cells");
            float oddBefore = OddRays();

            var start = new Vector3(CutX + startOffset * h, 0.0f, back ? 0.05f : -0.05f);
            var end = new Vector3(CutX + endOffset * h, 0.0f, back ? -0.05f : 0.05f);
            Stroke(start, end, waver * h, lean);
            Run(40);
            List<int> after = sim.PieceSizes();
            float oddAfter = OddRays();
            TestContext.WriteLine($"pieces [{string.Join(",", after.OrderByDescending(n => n))}], odd rays {oddBefore:P1} -> {oddAfter:P1}");
            Assert.AreEqual(0, LongSkinTriangles(h), "skin triangles longer than two cells");
            Assert.AreEqual(pieces, after.Count(n => n >= SurfaceMesh.MinVisiblePieceNodes), $"pieces [{string.Join(",", after)}]");
            Assert.AreEqual(before.Count(n => n < SurfaceMesh.MinVisiblePieceNodes), after.Count(n => n < SurfaceMesh.MinVisiblePieceNodes),
                $"fragments of fewer than {SurfaceMesh.MinVisiblePieceNodes} nodes: pieces [{string.Join(",", after)}]");
            Assert.Less(oddAfter - oddBefore, 0.02f, "rays through the incision crossing the surface an odd number of times");
        }

        /// <summary>
        /// A second stroke from the incision into the part beside it, upright and at 45 degrees to the incision, that
        /// stops inside the part: the partial cut stays. The end of the stroke restores only edges cut alongside the
        /// earlier cut, and these cuts run across it.
        /// </summary>
        [TestCase(0.0f, TestName = "StrokeFromAnIncisionIntoAPartKeepsItsCut")]
        [TestCase(45.0f, TestName = "SlantedStrokeFromAnIncisionIntoAPartKeepsItsCut")]
        public void StrokeFromAnIncisionIntoAPartKeepsItsCut(float angle)
        {
            SoftBodySimulation sim = _band.Simulation;
            CutThrough();
            int before = SeveredEdges();
            // From the gap of the incision 2 cm into the part between it and the right clamp, in the middle of the band.
            var direction = new Vector3(Mathf.Cos(angle * Mathf.Deg2Rad), 0.0f, Mathf.Sin(angle * Mathf.Deg2Rad));
            Vector3 start = new Vector3(CutX, 0.0f, 0.0f) - 0.008f * direction;
            Stroke(start, start + 0.028f * direction, 0.0f, 0.0f);
            int atEnd = SeveredEdges();
            Run(40);
            Assert.Greater(atEnd, before, "the second stroke cuts");
            Assert.AreEqual(atEnd, SeveredEdges(), "severed edges after the end of the stroke");
            Assert.AreEqual(2, sim.PieceSizes().Count(n => n >= SurfaceMesh.MinVisiblePieceNodes), "pieces");
        }

        /// <summary>The first stroke: across the band at <see cref="CutX"/>, through it, after the band has settled.</summary>
        private void CutThrough()
        {
            Run(60);
            Stroke(new Vector3(CutX, 0.0f, -0.05f), new Vector3(CutX, 0.0f, 0.05f), 0.0f, 0.0f);
            Run(40);
            List<int> pieces = _band.Simulation.PieceSizes();
            Assert.AreEqual(2, pieces.Count(n => n >= SurfaceMesh.MinVisiblePieceNodes), $"first stroke: pieces [{string.Join(",", pieces)}]");
        }

        /// <summary>Severed edges summed over the cells (an edge counts once for every cell sharing it).</summary>
        private int SeveredEdges()
        {
            int count = 0;
            foreach (ushort mask in _band.Simulation.Severed)
            {
                count += math.countbits((uint)mask);
            }
            return count;
        }

        /// <summary>
        /// A blade from 10 mm above the tissue under it to 30 mm below (through the 20 mm band), as ApxScalpelTool
        /// holds it, moved 60 frames from <paramref name="from"/> to <paramref name="to"/> and <paramref name="waver"/>
        /// to either side; the tip leans <paramref name="lean"/> degrees towards -x.
        /// </summary>
        private void Stroke(Vector3 from, Vector3 to, float waver, float lean)
        {
            SoftBodySimulation sim = _band.Simulation;
            Vector3 side = Vector3.Cross(Vector3.up, (to - from).normalized);
            float slope = Mathf.Tan(lean * Mathf.Deg2Rad);
            _scalpel.edgeStart = new Vector3(0.01f * slope, 0.01f, 0.0f);
            _scalpel.edgeEnd = new Vector3(-0.03f * slope, -0.03f, 0.0f);
            _scalpel.ResetSweep();
            _scalpel.cutting = true;
            float height = BandTop;
            const int frames = 60;
            for (int f = 0; f <= frames; f++)
            {
                Vector3 p = Vector3.Lerp(from, to, f / (float)frames) + side * (waver * Mathf.Sin(0.9f * f));
                int node = sim.PickNode(new Vector3(p.x, 0.3f, p.z), Vector3.down, sim.CellSize);
                height = node >= 0 ? sim.Positions[node].y : height;
                _scalpel.transform.position = new Vector3(p.x, height, p.z);
                Frame();
            }
            _scalpel.cutting = false;
        }

        private int LongSkinTriangles(float h)
        {
            Vector3[] vertices = _band.Surface.Mesh.vertices;
            int[] skin = _band.Surface.Mesh.GetTriangles(0);
            int stretched = 0;
            for (int t = 0; t < skin.Length; t += 3)
            {
                float longest = 0.0f;
                for (int i = 0; i < 3; i++)
                {
                    longest = Mathf.Max(longest, Vector3.Distance(vertices[skin[t + i]], vertices[skin[t + (i + 1) % 3]]));
                }
                stretched += longest > 2.0f * h ? 1 : 0;
            }
            return stretched;
        }

        /// <summary>
        /// Fraction of random lines through the incision (3 cm wide, the full height and width of the band) that
        /// cross the rendered surface an odd number of times. A closed surface is crossed an even number of times;
        /// holes and flaps make it odd. Skin and cut faces meet along seams without shared vertices, so some lines
        /// are odd on an intact surface too.
        /// </summary>
        private float OddRays()
        {
            Mesh mesh = _band.Surface.Mesh;
            Matrix4x4 toWorld = _band.transform.localToWorldMatrix;
            Vector3[] vertices = mesh.vertices.Select(v => toWorld.MultiplyPoint3x4(v)).ToArray();
            int[] triangles = mesh.GetTriangles(0).Concat(mesh.GetTriangles(1)).ToArray();
            Bounds bounds = _band.WorldBounds;
            var random = new System.Random(7);
            const int rays = 600;
            int odd = 0;
            for (int r = 0; r < rays; r++)
            {
                var target = new Vector3(
                    Mathf.Lerp(CutX - 0.015f, CutX + 0.015f, (float)random.NextDouble()),
                    Mathf.Lerp(bounds.min.y, bounds.max.y, (float)random.NextDouble()),
                    Mathf.Lerp(bounds.min.z + 0.003f, bounds.max.z - 0.003f, (float)random.NextDouble()));
                Vector3 direction = new Vector3((float)random.NextDouble() * 2.0f - 1.0f,
                    (float)random.NextDouble() * 2.0f - 1.0f, (float)random.NextDouble() * 2.0f - 1.0f).normalized;
                Vector3 origin = target - direction;
                int hits = 0;
                for (int t = 0; t < triangles.Length; t += 3)
                {
                    hits += RayHits(origin, direction, vertices[triangles[t]], vertices[triangles[t + 1]], vertices[triangles[t + 2]]) ? 1 : 0;
                }
                odd += hits & 1;
            }
            return odd / (float)rays;
        }

        private static bool RayHits(Vector3 origin, Vector3 direction, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 e1 = b - a, e2 = c - a;
            Vector3 p = Vector3.Cross(direction, e2);
            float det = Vector3.Dot(e1, p);
            if (Mathf.Abs(det) < 1e-12f)
            {
                return false;
            }
            float inverse = 1.0f / det;
            Vector3 s = origin - a;
            float u = Vector3.Dot(s, p) * inverse;
            if (u < 0.0f || u > 1.0f)
            {
                return false;
            }
            Vector3 q = Vector3.Cross(s, e1);
            float v = Vector3.Dot(direction, q) * inverse;
            return v >= 0.0f && u + v <= 1.0f && Vector3.Dot(e2, q) * inverse > 0.0f;
        }

        private Collider Clamp(float x)
        {
            GameObject clamp = InScene(GameObject.CreatePrimitive(PrimitiveType.Cube));
            clamp.transform.position = new Vector3(x, 0.06f, 0.0f);
            clamp.transform.localScale = new Vector3(0.02f, 0.03f, 0.09f);
            var collider = clamp.GetComponent<BoxCollider>();
            collider.isTrigger = true;
            return collider;
        }

        private void Run(int frames)
        {
            for (int f = 0; f < frames; f++)
            {
                Frame();
            }
        }

        private void Frame()
        {
            _scalpel.Sweep();
            _band.Simulate(Dt);
            _band.UpdateRender();
        }
    }
}
