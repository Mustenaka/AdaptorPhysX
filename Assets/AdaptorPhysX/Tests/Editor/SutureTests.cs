using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX.Tests
{
    /// <summary>Simplified sutures (TODO SUT-5): stitches between material points and the cut-face contact.</summary>
    public sealed class SutureTests
    {
        private const float Dt = 1.0f / 60.0f;
        private const int Substeps = 16;
        /// <summary>Plane of the incision of <see cref="Incise"/>, and its top face.</summary>
        private const float IncisionZ = 0.0005f;
        private const float Surface = 0.015f;

        private readonly List<SoftBodySimulation> _sims = new List<SoftBodySimulation>();

        [TearDown]
        public void TearDown()
        {
            foreach (SoftBodySimulation sim in _sims)
            {
                sim.Dispose();
            }
            _sims.Clear();
        }

        /// <summary>
        /// The small incision of S6 (2 cm long and 6 mm deep in an 8 x 3 x 6 cm block with a pre-strained skin layer, held in
        /// a tray, cell 3.3 mm) gapes under the skin tension; three stitches 6 mm apart, between points 3 mm either side of
        /// it and 1 mm deep, drawn at 1 cm/s to the length of the skin between them without stress, close it at the surface,
        /// and the walls do not pass through each other. The distance of the points before the cut is longer: the skin
        /// between a point and the wall contracts by its pre-strain, and the incision stayed open by 0.85 mm in a trial.
        /// </summary>
        [Test]
        public void StitchesCloseTheIncisionAgainstTheSkinTension()
        {
            SoftBodySimulation sim = Suture(true, out float open, out float closed, out float crossing);
            float h = sim.CellSize;
            Assert.Greater(open, 0.3f * h, "skin tension did not open the incision");
            Assert.Less(closed, 0.05f * h, "the stitches did not close the incision");
            Assert.Less(crossing, 0.05f * h, "the walls pass through each other");
            Assert.Less(SpeedNearIncision(sim), 0.005f, "the stitched tissue does not come to rest");
            Assert.AreEqual(1, sim.CountPieces());
        }

        /// <summary>
        /// The stitched incision of <see cref="StitchesCloseTheIncisionAgainstTheSkinTension"/> without the cut-face contact:
        /// the walls pressed together by the stitches pass through each other.
        /// </summary>
        [Test]
        public void CutFaceContactKeepsTheStitchedWallsFromPassingThroughEachOther()
        {
            SoftBodySimulation with = Suture(true, out _, out _, out float crossingWith);
            Suture(false, out _, out _, out float crossingWithout);
            float h = with.CellSize;
            Assert.Less(crossingWith, 0.05f * h);
            Assert.Greater(crossingWithout, 0.2f * h, "without the cut-face contact the walls pass through each other");
        }

        /// <summary>
        /// Stitches the incision of <see cref="Incise"/> with stitches 1 mm deep drawn at 1 cm/s for 2 s;
        /// <paramref name="open"/> and <paramref name="closed"/> are the largest opening at the surface before and after,
        /// <paramref name="crossing"/> how far the walls have passed through each other.
        /// </summary>
        private SoftBodySimulation Suture(bool cutFaceContact, out float open, out float closed, out float crossing)
        {
            SoftBodySimulation sim = Incise(cutFaceContact, 0.001f, out int[] a, out int[] b, out List<int2> walls);
            float h = sim.CellSize;
            open = SkinGap(sim, walls);
            var stitches = new int[a.Length];
            var before = new float[a.Length];
            for (int i = 0; i < a.Length; i++)
            {
                before[i] = Distance(sim, a[i], b[i]);
                stitches[i] = sim.AddStitch(a[i], b[i], 0.01f);
            }
            StepFrames(sim, 120);
            AssertFinite(sim);
            closed = SkinGap(sim, walls);
            crossing = MaxCrossing(sim);
            TestContext.WriteLine($"cut-face contact {cutFaceContact} ({sim.CutFacePairCount} pairs): opening at the surface " +
                                  $"{open * 1000:0.00} -> {closed * 1000:0.00} mm, walls crossed by at most {crossing * 1000:0.000} mm, " +
                                  $"speed near the incision {SpeedNearIncision(sim):0.0000} m/s");
            Assert.IsFalse(sim.StitchesDrawing);
            for (int i = 0; i < a.Length; i++)
            {
                float natural = sim.StitchNaturalLength(stitches[i]);
                float rest = math.distance(sim.MaterialPointRestPosition(a[i]), sim.MaterialPointRestPosition(b[i]));
                float distance = Distance(sim, a[i], b[i]);
                TestContext.WriteLine($"stitch {i}: {before[i] * 1000:0.00} -> {distance * 1000:0.00} mm (natural length " +
                                      $"{natural * 1000:0.00} mm, before the cut {rest * 1000:0.00} mm), tension " +
                                      $"{sim.StitchTension(stitches[i]):0.0000} N");
                Assert.AreEqual(natural, sim.StitchLength(stitches[i]), 1e-6f);
                // Where the walls meet before the stitch reaches its length, the cut-face contact holds the points apart.
                Assert.LessOrEqual(distance, natural + 0.05f * h);
                Assert.Less(natural, rest - 0.1f * h, "the skin between the points is pre-strained");
                Assert.Greater(before[i], rest, "the incision did not gape at the stitch");
                Assert.Greater(sim.StitchTension(stitches[i]), 0.0f, "the stitch does not hold the skin tension");
            }
            return sim;
        }

        /// <summary>
        /// The walls of an incision get the cut-face contact once its stroke ends: while the stroke is in progress the blade
        /// lies between them.
        /// </summary>
        [Test]
        public void CutFaceContactStartsWhenTheStrokeEnds()
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(new float3(0.1f), parameters, 8);
            Assert.Greater(sim.Cut(new float3(0.003f, 0.5f, -0.2f), new float3(0.003f, -0.5f, -0.2f), new float3(0.003f, 0.5f, 0.0f),
                new float3(0.003f, -0.5f, 0.0f), 1), 0);
            Assert.AreEqual(1, sim.CountPieces());
            Assert.AreEqual(0, sim.CutFacePairCount, "the stroke is in progress");
            sim.EndStroke(1);
            Assert.Greater(sim.CutFacePairCount, 0);
        }

        /// <summary>
        /// A 10 cm cube hangs from its top layer; nine stitches across the plane of a horizontal cut (points a quarter cell
        /// above and below it, not drawn) hold the part below once the cut goes through: their tensions add up to its weight.
        /// </summary>
        [Test]
        public void StitchesCarryThePartBelowACut()
        {
            SoftBodySimulation sim = CreateBox(new float3(0.1f), SoftBodyParameters.Default, 8);
            float h = sim.CellSize;
            var top = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.RestPositions[i].y > 0.05f - 0.1f * h)
                {
                    top.Add(i);
                }
            }
            sim.Attach(top, 1, float4x4.identity);
            float y = 0.003f;
            var stitches = new List<int>();
            for (int x = -1; x <= 1; x++)
            for (int z = -1; z <= 1; z++)
            {
                var p = new float3(0.03f * x, y, 0.03f * z);
                int above = sim.AddMaterialPoint(p + new float3(0.0f, 0.25f * h, 0.0f));
                int below = sim.AddMaterialPoint(p - new float3(0.0f, 0.25f * h, 0.0f));
                stitches.Add(sim.AddStitch(above, below, 0.0f));
            }
            Assert.Greater(sim.Cut(new float3(-0.2f, y, -0.2f), new float3(0.2f, y, -0.2f), new float3(-0.2f, y, 0.2f),
                new float3(0.2f, y, 0.2f)), 0);
            Assert.AreEqual(2, sim.CountPieces());
            Assert.AreEqual(0, sim.CutFacePairCount, "separated pieces have the piece contact instead");
            StepFrames(sim, 180);
            AssertFinite(sim);

            // Weight of the lower part: the nodes of the copies whose real corners are in it.
            int lower = -1;
            var owner = new int[sim.NodeCount];
            for (int q = 0; q < sim.CopyCount; q++)
            {
                int b = q * Lattice.CornerCount;
                int piece = sim.PieceId[sim.CopyNodes[b + math.tzcnt((int)sim.Copies[q].Mask)]];
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    owner[sim.CopyNodes[b + k]] = piece;
                }
            }
            for (int i = 0; i < sim.NodeCount; i++)
            {
                lower = sim.NodeReal[i] != 0 && sim.RestPositions[i].y < y - 0.5f * h ? sim.PieceId[i] : lower;
            }
            float weight = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                weight += owner[i] == lower && sim.InverseMass[i] > 0.0f ? 9.81f / sim.InverseMass[i] : 0.0f;
            }
            float tension = 0.0f;
            foreach (int stitch in stitches)
            {
                tension += sim.StitchTension(stitch);
            }
            TestContext.WriteLine($"tensions {tension:0.0000} N, lower part {weight:0.0000} N, whole {sim.TotalMass * 9.81f:0.0000} N, " +
                                  $"max speed {sim.MaxSpeed:0.0000} m/s");
            Assert.Less(weight, 0.8f * sim.TotalMass * 9.81f, "the cut separates a part");
            Assert.AreEqual(weight, tension, 0.02f * weight);
        }

        /// <summary>
        /// The S6 block with the incision (in the plane z = <see cref="IncisionZ"/>, x from -1 to 1 cm, 6 mm deep), opened by
        /// the skin tension for 1 s; <paramref name="a"/> and <paramref name="b"/> are material points 3 mm either side of the
        /// incision and <paramref name="depth"/> below the surface at x = -6, 0 and 6 mm, <paramref name="walls"/> pairs of
        /// points on the two walls 0.2 mm below the surface, every millimetre from x = -8 to 8 mm, all placed before the cut.
        /// </summary>
        private SoftBodySimulation Incise(bool selfCollision, float depth, out int[] a, out int[] b, out List<int2> walls)
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            parameters.SkinThickness = 0.004f;
            parameters.SkinStiffness = 20.0f;
            parameters.SkinPreStrain = 0.2f;
            parameters.SelfCollision = selfCollision;
            SoftBodySimulation sim = CreateBox(new float3(0.08f, 0.03f, 0.06f), parameters, 24);
            // Held in a tray: bottom and side faces fixed, so the skin tension stays in the skin.
            float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
            for (int i = 0; i < sim.NodeCount; i++)
            {
                lo = math.min(lo, sim.Positions[i]);
                hi = math.max(hi, sim.Positions[i]);
            }
            var tray = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 x = sim.Positions[i];
                if (x.y < lo.y + 1e-4f || x.x < lo.x + 1e-4f || x.x > hi.x - 1e-4f || x.z < lo.z + 1e-4f ||
                    x.z > hi.z - 1e-4f)
                {
                    tray.Add(i);
                }
            }
            sim.Attach(tray, 1, float4x4.identity);
            StepFrames(sim, 30);
            a = new int[3];
            b = new int[3];
            for (int i = 0; i < 3; i++)
            {
                float x = 0.006f * (i - 1);
                a[i] = sim.AddMaterialPoint(new float3(x, Surface - depth, IncisionZ - 0.003f));
                b[i] = sim.AddMaterialPoint(new float3(x, Surface - depth, IncisionZ + 0.003f));
                Assert.GreaterOrEqual(a[i], 0);
                Assert.GreaterOrEqual(b[i], 0);
            }
            walls = new List<int2>();
            for (int i = -8; i <= 8; i++)
            {
                walls.Add(new int2(sim.AddMaterialPoint(new float3(0.001f * i, Surface - 0.0002f, IncisionZ - 0.0002f)),
                    sim.AddMaterialPoint(new float3(0.001f * i, Surface - 0.0002f, IncisionZ + 0.0002f))));
            }
            sim.Cut(new float3(-0.01f, Surface + 0.01f, IncisionZ), new float3(-0.01f, Surface - 0.006f, IncisionZ),
                new float3(0.01f, Surface + 0.01f, IncisionZ), new float3(0.01f, Surface - 0.006f, IncisionZ));
            Assert.AreEqual(1, sim.CountPieces());
            Assert.Greater(sim.CutFacePairCount, 0);
            StepFrames(sim, 60);
            return sim;
        }

        /// <summary>Largest opening of the incision at the surface: the distance of the wall points less their rest distance.</summary>
        private static float SkinGap(SoftBodySimulation sim, List<int2> walls)
        {
            float gap = 0.0f;
            foreach (int2 w in walls)
            {
                float rest = math.distance(sim.MaterialPointRestPosition(w.x), sim.MaterialPointRestPosition(w.y));
                gap = math.max(gap, Distance(sim, w.x, w.y) - rest);
            }
            return gap;
        }

        /// <summary>Largest speed of a node within 1 cm of the middle of the incision.</summary>
        private static float SpeedNearIncision(SoftBodySimulation sim)
        {
            float speed = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (math.distance(sim.RestPositions[i], new float3(0.0f, Surface, IncisionZ)) < 0.01f)
                {
                    speed = math.max(speed, math.length(sim.Velocities[i]));
                }
            }
            return speed;
        }

        private static float Distance(SoftBodySimulation sim, int a, int b) =>
            math.distance(sim.MaterialPointPosition(a), sim.MaterialPointPosition(b));

        /// <summary>
        /// How far the walls of the incision of <see cref="Incise"/> have passed through each other: at every grid point with
        /// nodes of both sides, how far the node of the copy across the cut (virtual there) is behind the node with material,
        /// along z.
        /// </summary>
        private static float MaxCrossing(SoftBodySimulation sim)
        {
            float crossing = 0.0f;
            foreach (List<int> group in GridPoints(sim))
            {
                int owner = group.FindIndex(n => sim.NodeReal[n] != 0);
                if (owner < 0)
                {
                    continue;
                }
                float3 x = sim.Positions[group[owner]];
                // Towards the other side of the cut.
                float side = sim.RestPositions[group[owner]].z < IncisionZ ? 1.0f : -1.0f;
                foreach (int n in group)
                {
                    if (sim.NodeReal[n] == 0)
                    {
                        crossing = math.max(crossing, -side * (sim.Positions[n].z - x.z));
                    }
                }
            }
            return crossing;
        }

        private static IEnumerable<List<int>> GridPoints(SoftBodySimulation sim)
        {
            var byRest = new Dictionary<float3, List<int>>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (!byRest.TryGetValue(sim.RestPositions[i], out List<int> list))
                {
                    byRest[sim.RestPositions[i]] = list = new List<int>();
                }
                list.Add(i);
            }
            foreach (List<int> group in byRest.Values)
            {
                if (group.Count > 1)
                {
                    yield return group;
                }
            }
        }

        private SoftBodySimulation CreateBox(float3 size, SoftBodyParameters parameters, int resolution)
        {
            Mesh mesh = MeshUtil.CreateBox(size, 8);
            var p = new List<float3>();
            foreach (Vector3 v in mesh.vertices) p.Add(v);
            var n = new List<float3>();
            foreach (Vector3 v in mesh.normals) n.Add(v);
            var uv = new List<Vector2>(mesh.uv);
            var triangles = new List<int>(mesh.triangles);
            Object.DestroyImmediate(mesh);
            float cell = math.cmax(size) / resolution;
            MeshUtil.Subdivide(p, n, uv, triangles, 0.5f * cell);
            VoxelGrid grid = Voxelizer.Voxelize(p.ToArray(), triangles.ToArray(), cell);
            var sim = new SoftBodySimulation(grid, parameters, float4x4.identity);
            _sims.Add(sim);
            return sim;
        }

        private static void StepFrames(SoftBodySimulation sim, int frames)
        {
            using var shapes = new NativeArray<ColliderShape>(0, Allocator.Persistent);
            for (int f = 0; f < frames; f++)
            {
                sim.Step(Dt, Substeps, shapes);
            }
        }

        private static void AssertFinite(SoftBodySimulation sim)
        {
            for (int i = 0; i < sim.NodeCount; i++)
            {
                Assert.IsTrue(math.all(math.isfinite(sim.Positions[i])), $"node {i} is not finite");
            }
        }
    }
}
