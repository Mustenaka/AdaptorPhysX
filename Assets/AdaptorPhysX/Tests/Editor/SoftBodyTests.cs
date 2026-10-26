using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX.Tests
{
    public sealed class SoftBodyTests
    {
        private const float Size = 0.1f;
        private const int Resolution = 8;
        private const float Dt = 1.0f / 60.0f;
        private const int Substeps = 16;

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

        [Test]
        public void TetrahedraFillTheCellAndComponentsSplitCleanly()
        {
            Lattice.BuildRestTets(1.0f, out _, out float[] volumes);
            for (int parity = 0; parity < 2; parity++)
            {
                float sum = 0.0f;
                for (int t = 0; t < Lattice.TetsPerCell; t++)
                {
                    sum += volumes[parity * Lattice.TetsPerCell + t];
                }
                Assert.AreEqual(1.0f, sum, 1e-6f);
            }

            Lattice.BuildComponentTable(out byte[] counts, out byte[] masks);
            Assert.AreEqual(1, counts[0]);
            Assert.AreEqual(0xFF, masks[0]);
            const int xEdges = 0x00F;
            Assert.AreEqual(2, counts[xEdges]);
            Assert.AreEqual(0x55, masks[xEdges * 8]);
            Assert.AreEqual(0xAA, masks[xEdges * 8 + 1]);
        }

        [Test]
        public void RestStateIsStationaryWithoutGravity()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            float3[] start = sim.Positions.ToArray();
            StepFrames(sim, 60);
            Assert.Less(MaxDisplacement(sim, start), 1e-4f);
        }

        [Test]
        public void RotatedBodyStaysAtRestWithoutGravity()
        {
            // The tetrahedron rotations start at the body's rotation. Started at identity, the
            // warm-started update needs several substeps for a quarter turn and never leaves
            // identity for a half turn, and the elastic constraints deform the body meanwhile.
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            foreach (float yaw in new[] { 90.0f, 180.0f })
            {
                SoftBodySimulation sim = CreateBox(new float3(Size), parameters, Resolution, yaw);
                float3[] start = sim.Positions.ToArray();
                StepFrames(sim, 30);
                Assert.Less(MaxDisplacement(sim, start), 1e-4f, $"yaw {yaw}");
            }
        }

        [Test]
        public void StiffMaterialOnFineLatticeStaysAtRest()
        {
            // mu dt^2 / (rho h^2) is about 40 here: far inside the regime where the stable
            // Neo-Hookean constraint pair diverged on the first substep.
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            parameters.YoungsModulus = 1.0e6f;
            SoftBodySimulation sim = CreateBox(new float3(Size), parameters, 32);
            float3[] start = sim.Positions.ToArray();
            StepFrames(sim, 30);
            Assert.Less(MaxDisplacement(sim, start), 1e-4f);
            Assert.Less(MaxSpeed(sim), 1e-3f);
        }

        [Test]
        public void SelfWeightCompressionMatchesYoungsModulus()
        {
            // A column with a clamped base sagging under its own weight (nu = 0):
            // top displacement = rho g H^2 / (2 E).
            var parameters = SoftBodyParameters.Default;
            parameters.PoissonRatio = 0.0f;
            parameters.YoungsModulus = 1.0e4f;
            parameters.Damping = 5.0f;
            SoftBodySimulation sim = CreateBox(new float3(Size), parameters, Resolution);
            float bottom = float.MaxValue, top = float.MinValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                bottom = math.min(bottom, sim.Positions[i].y);
                top = math.max(top, sim.Positions[i].y);
            }
            var clamped = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.Positions[i].y < bottom + 1e-4f)
                {
                    clamped.Add(i);
                }
            }
            sim.Attach(clamped, 1, float4x4.identity);
            float topBefore = TopCenterHeight(sim, top);
            StepFrames(sim, 240);
            float sag = topBefore - TopCenterHeight(sim, top);
            float height = top - bottom;
            float expected = parameters.Density * 9.81f * height * height / (2.0f * parameters.YoungsModulus);
            Assert.AreEqual(expected, sag, 0.3f * expected, $"sag {sag * 1000:0.00} mm, expected {expected * 1000:0.00} mm");
        }

        private static float TopCenterHeight(SoftBodySimulation sim, float restTop)
        {
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 r = sim.RestPositions[i];
                if (math.abs(r.y - restTop) < 1e-4f && math.abs(r.x) < 0.02f && math.abs(r.z) < 0.02f)
                {
                    sum += sim.Positions[i].y;
                    count++;
                }
            }
            return sum / math.max(count, 1);
        }

        [Test]
        public void DropOnFloorSettlesWithoutLosingVolume()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: true, height: 0.02f);
            using var floor = new NativeArray<ColliderShape>(new[]
            {
                ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f),
            }, Allocator.Persistent);
            float startVolume = BoundsVolume(sim);
            StepFrames(sim, 180, floor);

            AssertFinite(sim);
            float minY = float.MaxValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                minY = math.min(minY, sim.Positions[i].y);
            }
            Assert.Greater(minY, -0.005f, "lattice sank into the floor");
            Assert.Less(minY, 0.01f, "body did not land");
            Assert.AreEqual(startVolume, BoundsVolume(sim), 0.2f * startVolume);
            Assert.Less(MaxSpeed(sim), 0.05f, "body did not come to rest");
        }

        [Test]
        public void FullCutSeparatesTwoPiecesThatFallApart()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            Assert.AreEqual(1, sim.CountPieces());
            int severed = CutPlaneX(sim, 0.0f, -0.2f, 0.2f);
            Assert.Greater(severed, 0);
            Assert.AreEqual(2, sim.CountPieces());
            Assert.Greater(sim.CopyCount, sim.CellCount);

            // Pull the two halves apart with opposite velocities; they must not stay attached.
            for (int i = 0; i < sim.NodeCount; i++)
            {
                sim.Velocities[i] = new float3(sim.Positions[i].x < 0.0f ? -0.2f : 0.2f, 0.0f, 0.0f);
            }
            StepFrames(sim, 30);
            AssertFinite(sim);
            Assert.Greater(MinPairGap(sim), 0.05f);
        }

        [Test]
        public void FullCutThroughALayerOfNodesSeparatesTwoPieces()
        {
            // The blade plane holds a layer of lattice nodes exactly: those nodes go to one side,
            // so only the edges from the layer to the other side are severed.
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(new float3(Size), parameters, 16);
            float layer = float.MaxValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (math.abs(sim.Positions[i].x) < math.abs(layer))
                {
                    layer = sim.Positions[i].x;
                }
            }
            int onPlane = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                onPlane += sim.Positions[i].x == layer ? 1 : 0;
            }
            Assert.Greater(onPlane, 100, "no layer of nodes on the blade plane");

            int severed = CutPlaneX(sim, layer, -0.2f, 0.2f);
            Assert.Greater(severed, 0);
            List<int> pieces = sim.PieceSizes();
            Assert.AreEqual(2, pieces.Count, "piece sizes " + string.Join(", ", pieces));
            StepFrames(sim, 10);
            AssertFinite(sim);
        }

        [Test]
        public void BladeLeavingThroughTheSurfaceSeparatesBeforeReachingTheOuterNodes()
        {
            // The lowest node layer lies half a cell below the bottom face (y = -0.05). A blade tip
            // between the two has cut all the material; a tip above the face leaves some uncut.
            SoftBodySimulation through = CreateBox(new float3(Size), gravity: false);
            float bottomNodes = float.MaxValue;
            for (int i = 0; i < through.NodeCount; i++)
            {
                bottomNodes = math.min(bottomNodes, through.Positions[i].y);
            }
            Assert.Less(bottomNodes, -0.5f * Size - 0.4f * through.CellSize);
            CutPlaneX(through, 0.0f, -0.2f, 0.2f, yBottom: -0.5f * Size - 0.2f * through.CellSize);
            Assert.AreEqual(2, through.CountPieces(), "blade through the bottom face did not separate");

            SoftBodySimulation partial = CreateBox(new float3(Size), gravity: false);
            CutPlaneX(partial, 0.0f, -0.2f, 0.2f, yBottom: -0.5f * Size + 0.2f * partial.CellSize);
            Assert.AreEqual(1, partial.CountPieces(), "blade above the bottom face cut through");
        }

        [Test]
        public void StrokeOfManySmallSweepsCutsThrough()
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(new float3(0.12f, 0.05f, 0.1f), parameters, 20);
            const int frames = 40;
            for (int f = 1; f <= frames; f++)
            {
                float z0 = math.lerp(-0.09f, 0.09f, (f - 1) / (float)frames);
                float z1 = math.lerp(-0.09f, 0.09f, f / (float)frames);
                sim.Cut(new float3(0.0f, 0.06f, z0), new float3(0.0f, -0.06f, z0),
                    new float3(0.0f, 0.06f, z1), new float3(0.0f, -0.06f, z1));
            }
            var intact = new List<string>();
            for (int c = 0; c < sim.CellCount; c++)
            {
                for (int e = 0; e < 4; e++)
                {
                    float3 a = sim.RestPositions[sim.CopyNodes[sim.CellCopyRange[c].x * 8 + Lattice.EdgeCorners[2 * e]]];
                    float3 b = sim.RestPositions[sim.CopyNodes[sim.CellCopyRange[c].x * 8 + Lattice.EdgeCorners[2 * e + 1]]];
                    if ((sim.Severed[c] & (1 << e)) == 0 && a.x < 0.0f && b.x > 0.0f)
                    {
                        intact.Add($"cell {sim.CellCoord[c]} edge {e} y {a.y:0.0000} z {a.z:0.0000}");
                    }
                }
            }
            Assert.IsEmpty(intact, string.Join("\n", intact));
            Assert.AreEqual(2, sim.CountPieces());
        }

        [Test]
        public void CutOffPieceRestsOnTheRestInsteadOfFallingThrough()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: true, height: 0.001f);
            using var floor = new NativeArray<ColliderShape>(new[]
            {
                ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f),
            }, Allocator.Persistent);
            StepFrames(sim, 30, floor);
            float cutHeight = 0.001f + 0.5f * Size + 0.002f;
            sim.Cut(new float3(-0.2f, cutHeight, -0.2f), new float3(0.2f, cutHeight, -0.2f),
                new float3(-0.2f, cutHeight, 0.2f), new float3(0.2f, cutHeight, 0.2f));
            Assert.AreEqual(2, sim.CountPieces());
            float before = TopPieceCenter(sim, cutHeight);
            StepFrames(sim, 90, floor);
            AssertFinite(sim);
            Assert.Less(before - TopPieceCenter(sim, cutHeight), 0.01f, "top piece sank into the bottom piece");
        }

        private static float TopPieceCenter(SoftBodySimulation sim, float cutHeight)
        {
            // Rest positions are body-centred; the body centre sits at 0.001 + Size / 2.
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.NodeReal[i] != 0 && sim.RestPositions[i].y + 0.001f + 0.5f * Size > cutHeight)
                {
                    sum += sim.Positions[i].y;
                    count++;
                }
            }
            return sum / count;
        }

        [Test]
        public void PartialCutLeavesOnePiece()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            int severed = CutPlaneX(sim, 0.0f, -0.2f, 0.0f);
            Assert.Greater(severed, 0);
            Assert.AreEqual(1, sim.CountPieces());
            Assert.Greater(sim.CopyCount, sim.CellCount);
            StepFrames(sim, 30);
            AssertFinite(sim);
        }

        [Test]
        public void CrossCutAndReCutKeepSplitting()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            CutPlaneX(sim, 0.0f, -0.2f, 0.2f);
            CutPlaneZ(sim, 0.0f, -0.2f, 0.2f);
            Assert.AreEqual(4, sim.CountPieces());
            // Cut one quarter again.
            sim.Cut(new float3(0.025f, 0.2f, -0.2f), new float3(0.025f, -0.2f, -0.2f),
                new float3(0.025f, 0.2f, -0.001f), new float3(0.025f, -0.2f, -0.001f));
            Assert.AreEqual(5, sim.CountPieces());
            StepFrames(sim, 10);
            AssertFinite(sim);
        }

        [Test]
        public void PreStrainedPinnedSlabOpensIncision()
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            parameters.PreStrain = 0.15f;
            SoftBodySimulation sim = CreateBox(new float3(0.16f, 0.03f, 0.08f), parameters, 16);
            var pinned = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (math.abs(sim.Positions[i].x) > 0.07f)
                {
                    pinned.Add(i);
                }
            }
            sim.Attach(pinned, 1, float4x4.identity);
            StepFrames(sim, 30);

            CutPlaneX(sim, 0.0f, -0.2f, 0.2f, yTop: 0.2f, yBottom: 0.0f);
            Assert.AreEqual(1, sim.CountPieces(), "incision should not go through the slab");
            StepFrames(sim, 60);
            AssertFinite(sim);
            Assert.Greater(MaxPairGap(sim), 0.3f * sim.CellSize, "pre-strained incision did not gape");
        }

        [Test]
        public void SkinTensionOpensSmallIncisionInSolidTissue()
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            parameters.SkinThickness = 0.004f;
            parameters.SkinStiffness = 20.0f;
            parameters.SkinPreStrain = 0.2f;
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

            // 2 cm long, 6 mm deep incision in the middle of the 3 cm thick block.
            float surface = 0.015f;
            sim.Cut(new float3(-0.01f, surface + 0.01f, 0.0005f), new float3(-0.01f, surface - 0.006f, 0.0005f),
                new float3(0.01f, surface + 0.01f, 0.0005f), new float3(0.01f, surface - 0.006f, 0.0005f));
            Assert.AreEqual(1, sim.CountPieces());
            StepFrames(sim, 60);
            AssertFinite(sim);
            Assert.Greater(MaxPairGap(sim), 0.3f * sim.CellSize, "skin tension did not open the incision");

            // Below the incision the tissue is still solid: no duplicated nodes deeper than the cut.
            var byRest = new Dictionary<float3, int>();
            float deepest = float.MaxValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                byRest[sim.RestPositions[i]] = byRest.TryGetValue(sim.RestPositions[i], out int c) ? c + 1 : 1;
            }
            foreach (KeyValuePair<float3, int> entry in byRest)
            {
                if (entry.Value > 1)
                {
                    deepest = math.min(deepest, entry.Key.y);
                }
            }
            Assert.Greater(deepest, surface - 0.006f - 2.0f * sim.CellSize, "the incision reaches deeper than the blade");
        }

        [Test]
        public void SurfaceMeshBuildsCutFacesInsideTheSkin()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            surface.Update(float4x4.identity);
            Assert.AreEqual(0, surface.CutTriangleCount);
            int skinBefore = surface.SkinTriangleCount;

            CutPlaneX(sim, 0.0f, -0.2f, 0.2f);
            surface.Update(float4x4.identity);
            Assert.Greater(surface.CutTriangleCount, 0);
            Assert.GreaterOrEqual(surface.SkinTriangleCount, skinBefore);
            Assert.Less(surface.SkinTriangleCount, skinBefore * 1.2f);
            foreach (Vector3 v in surface.Mesh.vertices)
            {
                Assert.IsTrue(math.all(math.isfinite((float3)v)));
                Assert.LessOrEqual(math.cmax(math.abs((float3)v)), 0.5f * Size + 1e-3f, "cut face sticks out of the skin");
            }
        }

        [Test]
        public void SkinDoesNotBridgeSeparatedPieces()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            CutPlaneX(sim, 0.0f, -0.2f, 0.2f);
            CutPlaneZ(sim, 0.0f, -0.2f, 0.2f);
            Assert.AreEqual(4, sim.CountPieces());
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 p = sim.Positions[i];
                sim.Velocities[i] = new float3(p.x < 0.0f ? -0.3f : 0.3f, 0.0f, p.z < 0.0f ? -0.3f : 0.3f);
            }
            StepFrames(sim, 20);
            surface.Update(float4x4.identity);

            Vector3[] vertices = surface.Mesh.vertices;
            int[] skin = surface.Mesh.GetTriangles(0);
            float longest = 0.0f;
            for (int t = 0; t < skin.Length; t += 3)
            {
                for (int i = 0; i < 3; i++)
                {
                    longest = math.max(longest, Vector3.Distance(vertices[skin[t + i]], vertices[skin[t + (i + 1) % 3]]));
                }
            }
            Assert.Less(longest, 2.0f * sim.CellSize, "a skin triangle spans the gap between pieces");
        }

        [Test]
        public void SameInputsGiveBitIdenticalResults()
        {
            float3[] Run()
            {
                SoftBodySimulation sim = CreateBox(new float3(Size), gravity: true, height: 0.05f);
                using var floor = new NativeArray<ColliderShape>(new[]
                {
                    ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f),
                }, Allocator.Persistent);
                StepFrames(sim, 20, floor);
                CutPlaneX(sim, 0.01f, -0.2f, 0.0f);
                StepFrames(sim, 20, floor);
                return sim.Positions.ToArray();
            }

            float3[] a = Run();
            float3[] b = Run();
            Assert.AreEqual(a.Length, b.Length);
            for (int i = 0; i < a.Length; i++)
            {
                Assert.IsTrue(math.all(a[i] == b[i]), $"node {i} differs");
            }
        }

        private SoftBodySimulation CreateBox(float3 size, bool gravity, float height = 0.0f) =>
            CreateBox(size, gravity, out _, out _, out _, out _, height);

        private SoftBodySimulation CreateBox(float3 size, bool gravity, out float3[] rest, out float3[] normals,
            out Vector2[] uvs, out int[] triangles, float height = 0.0f)
        {
            var parameters = SoftBodyParameters.Default;
            if (!gravity)
            {
                parameters.Gravity = float3.zero;
            }
            return CreateBox(size, parameters, Resolution, out rest, out normals, out uvs, out triangles, height);
        }

        private SoftBodySimulation CreateBox(float3 size, SoftBodyParameters parameters, int resolution, float yaw = 0.0f) =>
            CreateBox(size, parameters, resolution, out _, out _, out _, out _, 0.0f, yaw);

        /// <param name="yaw">Rotation of the body about the vertical axis, in degrees.</param>
        private SoftBodySimulation CreateBox(float3 size, SoftBodyParameters parameters, int resolution,
            out float3[] rest, out float3[] normals, out Vector2[] uvs, out int[] triangles, float height, float yaw = 0.0f)
        {
            Mesh mesh = MeshUtil.CreateBox(size, 8);
            var p = new List<float3>();
            var n = new List<float3>();
            foreach (Vector3 v in mesh.vertices) p.Add(v);
            foreach (Vector3 v in mesh.normals) n.Add(v);
            var uv = new List<Vector2>(mesh.uv);
            var tris = new List<int>(mesh.triangles);
            Object.DestroyImmediate(mesh);
            float cell = math.cmax(size) / resolution;
            MeshUtil.Subdivide(p, n, uv, tris, 0.5f * cell);
            rest = p.ToArray();
            normals = n.ToArray();
            uvs = uv.ToArray();
            triangles = tris.ToArray();
            VoxelGrid grid = Voxelizer.Voxelize(rest, triangles, cell);
            var sim = new SoftBodySimulation(grid, parameters, float4x4.TRS(
                new float3(0.0f, height + 0.5f * size.y, 0.0f) * (height > 0.0f ? 1.0f : 0.0f),
                quaternion.RotateY(math.radians(yaw)), new float3(1.0f)));
            _sims.Add(sim);
            return sim;
        }

        private static void StepFrames(SoftBodySimulation sim, int frames, NativeArray<ColliderShape> shapes = default)
        {
            bool owned = !shapes.IsCreated;
            if (owned)
            {
                shapes = new NativeArray<ColliderShape>(0, Allocator.Persistent);
            }
            for (int f = 0; f < frames; f++)
            {
                sim.Step(Dt, Substeps, shapes);
            }
            if (owned)
            {
                shapes.Dispose();
            }
        }

        /// <summary>Sweeps a vertical blade lying in the plane x = <paramref name="x"/> along z.</summary>
        private static int CutPlaneX(SoftBodySimulation sim, float x, float z0, float z1, float yTop = 0.5f,
            float yBottom = -0.5f)
        {
            return sim.Cut(new float3(x, yTop, z0), new float3(x, yBottom, z0), new float3(x, yTop, z1),
                new float3(x, yBottom, z1));
        }

        private static int CutPlaneZ(SoftBodySimulation sim, float z, float x0, float x1)
        {
            return sim.Cut(new float3(x0, 0.5f, z), new float3(x0, -0.5f, z), new float3(x1, 0.5f, z),
                new float3(x1, -0.5f, z));
        }

        private static float MaxDisplacement(SoftBodySimulation sim, float3[] start)
        {
            float max = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                max = math.max(max, math.distance(sim.Positions[i], start[i]));
            }
            return max;
        }

        private static float MaxSpeed(SoftBodySimulation sim)
        {
            float max = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                max = math.max(max, math.length(sim.Velocities[i]));
            }
            return max;
        }

        private static float BoundsVolume(SoftBodySimulation sim)
        {
            float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
            for (int i = 0; i < sim.NodeCount; i++)
            {
                lo = math.min(lo, sim.Positions[i]);
                hi = math.max(hi, sim.Positions[i]);
            }
            float3 d = hi - lo;
            return d.x * d.y * d.z;
        }

        /// <summary>Separation of nodes that were duplicated from the same grid point.</summary>
        private static float MinPairGap(SoftBodySimulation sim) => PairGap(sim, true);

        private static float MaxPairGap(SoftBodySimulation sim) => PairGap(sim, false);

        private static float PairGap(SoftBodySimulation sim, bool min)
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
            float result = min ? float.MaxValue : 0.0f;
            bool any = false;
            foreach (List<int> group in byRest.Values)
            {
                if (group.Count < 2)
                {
                    continue;
                }
                float span = 0.0f;
                for (int a = 0; a < group.Count; a++)
                for (int b = a + 1; b < group.Count; b++)
                {
                    span = math.max(span, math.distance(sim.Positions[group[a]], sim.Positions[group[b]]));
                }
                result = min ? math.min(result, span) : math.max(result, span);
                any = true;
            }
            Assert.IsTrue(any, "no duplicated nodes");
            return result;
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
