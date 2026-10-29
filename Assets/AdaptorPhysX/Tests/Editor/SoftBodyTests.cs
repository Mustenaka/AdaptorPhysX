using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.TestTools;

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

        [Test]
        public void DensityChangeAppliesOnTheNextStep()
        {
            // Self-weight sag is proportional to the density: doubling it at runtime doubles the sag
            // without a topology rebuild.
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

            parameters.Density *= 2.0f;
            sim.Parameters = parameters;
            StepFrames(sim, 240);
            float heavySag = topBefore - TopCenterHeight(sim, top);
            Assert.AreEqual(2.0f, heavySag / sag, 0.2f, $"sag {sag * 1000:0.00} mm, doubled density {heavySag * 1000:0.00} mm");
        }

        [Test]
        public void PreStrainChangeAppliesOnTheNextStep()
        {
            // A free body with a 10 % rest-shape contraction shrinks to 90 % of its size.
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            parameters.Damping = 5.0f;
            SoftBodySimulation sim = CreateBox(new float3(Size), parameters, Resolution);
            float before = MeanDistanceFromCentroid(sim);
            parameters.PreStrain = 0.1f;
            sim.Parameters = parameters;
            StepFrames(sim, 120);
            AssertFinite(sim);
            Assert.AreEqual(0.9f, MeanDistanceFromCentroid(sim) / before, 0.01f);
        }

        [Test]
        public void RebuildAppliesResolutionAndKeepsPins()
        {
            var bodyObject = new GameObject("Rebuild Test", typeof(MeshFilter), typeof(MeshRenderer));
            var pinObject = new GameObject("Rebuild Test Pin");
            Mesh mesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            ApxSoftBody body = null;
            try
            {
                bodyObject.GetComponent<MeshFilter>().sharedMesh = mesh;
                pinObject.transform.position = new Vector3(-0.05f, 0.0f, 0.0f);
                var pin = pinObject.AddComponent<BoxCollider>();
                pin.size = new Vector3(0.03f, 0.2f, 0.2f);
                Physics.SyncTransforms();
                body = bodyObject.AddComponent<ApxSoftBody>();
                body.resolution = 8;
                body.pinColliders = new Collider[] { pin };
                body.Initialize();
                int cells = body.Simulation.CellCount;
                int pinned = CountPinned(body.Simulation);
                Assert.Greater(pinned, 0);

                body.resolution = 12;
                body.Rebuild();
                Assert.IsTrue(body.IsInitialized);
                Assert.Greater(body.Simulation.CellCount, cells, "resolution not applied");
                Assert.Greater(CountPinned(body.Simulation), pinned, "pins not applied to the new lattice");
                body.Simulate(Dt);
                body.UpdateRender();
                AssertFinite(body.Simulation);
            }
            finally
            {
                // Edit mode calls no OnDestroy: release the body so it leaves ApxSoftBody.Active.
                body?.Release();
                Object.DestroyImmediate(bodyObject);
                Object.DestroyImmediate(pinObject);
                Object.DestroyImmediate(mesh);
            }
        }

        [Test]
        public void OpenMeshWarnsOnInitialize()
        {
            var bodyObject = new GameObject("Open Mesh Test", typeof(MeshFilter), typeof(MeshRenderer));
            Mesh closed = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            Mesh open = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            int[] triangles = open.triangles;
            open.triangles = triangles[3..];
            ApxSoftBody body = null;
            try
            {
                MeshUtil.CountOpenEdges(System.Array.ConvertAll(closed.vertices, v => (float3)v), closed.triangles,
                    out int closedBoundary, out int closedNonManifold);
                MeshUtil.CountOpenEdges(System.Array.ConvertAll(open.vertices, v => (float3)v), open.triangles,
                    out int openBoundary, out _);
                Assert.AreEqual(0, closedBoundary);
                Assert.AreEqual(0, closedNonManifold);
                Assert.AreEqual(3, openBoundary);
                // The ellipsoid's seam vertices (theta = 0 and 2 pi) differ in the last bits; they are welded.
                Mesh ellipsoid = MeshUtil.CreateEllipsoid(new Vector3(0.05f, 0.03f, 0.04f));
                MeshUtil.CountOpenEdges(System.Array.ConvertAll(ellipsoid.vertices, v => (float3)v), ellipsoid.triangles,
                    out int ellipsoidBoundary, out _);
                Object.DestroyImmediate(ellipsoid);
                Assert.AreEqual(0, ellipsoidBoundary);

                body = bodyObject.AddComponent<ApxSoftBody>();
                body.resolution = 8;
                body.sourceMesh = closed;
                body.Initialize();
                LogAssert.NoUnexpectedReceived();
                Assert.IsTrue(body.IsInitialized);

                body.sourceMesh = open;
                LogAssert.Expect(LogType.Warning, new Regex("has 3 boundary edges"));
                body.Rebuild();
                Assert.IsTrue(body.IsInitialized);
            }
            finally
            {
                body?.Release();
                Object.DestroyImmediate(bodyObject);
                Object.DestroyImmediate(closed);
                Object.DestroyImmediate(open);
            }
        }

        [Test]
        public void UnusableNormalsAreRecalculatedFromTheTriangles()
        {
            var bodyObject = new GameObject("Normals Test", typeof(MeshFilter), typeof(MeshRenderer));
            Mesh box = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            ApxSoftBody body = null;
            try
            {
                body = bodyObject.AddComponent<ApxSoftBody>();
                body.resolution = 8;
                body.useGravity = false;
                body.sourceMesh = box;
                body.Initialize();
                LogAssert.NoUnexpectedReceived();

                // All normals up, as an importer leaves them for triangles too small to compute normals from.
                var up = new Vector3[box.vertexCount];
                for (int i = 0; i < up.Length; i++)
                {
                    up[i] = Vector3.up;
                }
                box.normals = up;
                LogAssert.Expect(LogType.Warning, new Regex("normals do not follow its triangles"));
                body.Rebuild();
                Vector3[] vertices = body.Surface.Mesh.vertices;
                Vector3[] normals = body.Surface.Mesh.normals;
                int bottom = 0;
                for (int i = 0; i < vertices.Length; i++)
                {
                    if (vertices[i].y < -0.0249f && math.abs(vertices[i].x) < 0.04f && math.abs(vertices[i].z) < 0.03f)
                    {
                        Assert.Less(normals[i].y, -0.99f, $"vertex {i} on the bottom face");
                        bottom++;
                    }
                }
                Assert.Greater(bottom, 0);
            }
            finally
            {
                body?.Release();
                Object.DestroyImmediate(bodyObject);
                Object.DestroyImmediate(box);
            }
        }

        [Test]
        public void OpenMeshIsVoxelizedLikeTheClosedMesh()
        {
            Mesh box = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            try
            {
                float3[] vertices = System.Array.ConvertAll(box.vertices, v => (float3)v);
                int[] closed = box.triangles;
                // The -x face (the first 4 x 4 x 2 triangles) removed: rays along +x enter and never leave.
                int[] open = closed[(4 * 4 * 2 * 3)..];
                MeshUtil.CountOpenEdges(vertices, open, out int boundary, out _);
                Assert.AreEqual(16, boundary);
                float cell = 0.1f / 8;

                VoxelGrid reference = Voxelizer.Voxelize(vertices, closed, cell);
                VoxelGrid winding = Voxelizer.Voxelize(vertices, open, cell, closed: false);
                VoxelGrid parity = Voxelizer.Voxelize(vertices, open, cell, closed: true);
                VoxelGrid closedWinding = Voxelizer.Voxelize(vertices, closed, cell, closed: false);
                Assert.AreEqual(reference.Sdf.Length, winding.Sdf.Length);
                int windingWrong = 0, parityWrong = 0, closedWrong = 0;
                for (int i = 0; i < reference.Sdf.Length; i++)
                {
                    bool inside = reference.Sdf[i] < 0.0f;
                    windingWrong += winding.Sdf[i] < 0.0f != inside ? 1 : 0;
                    parityWrong += parity.Sdf[i] < 0.0f != inside ? 1 : 0;
                    closedWrong += closedWinding.Sdf[i] < 0.0f != inside ? 1 : 0;
                }
                Assert.AreEqual(0, windingWrong, "winding number on the open box");
                Assert.AreEqual(0, closedWrong, "winding number on the closed box");
                Assert.Greater(parityWrong, 0, "ray parity should fail on the open box");
                CollectionAssert.AreEqual(reference.Cells, winding.Cells);
            }
            finally
            {
                Object.DestroyImmediate(box);
            }
        }

        private static int CountPinned(SoftBodySimulation sim)
        {
            int count = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                count += sim.KinematicId[i] > SoftBodySimulation.GrabFrame ? 1 : 0;
            }
            return count;
        }

        /// <summary>Size measure that scales with the body: the bounding box is skewed by corner nodes of nearly empty cells.</summary>
        private static float MeanDistanceFromCentroid(SoftBodySimulation sim)
        {
            float3 centroid = float3.zero;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                centroid += sim.Positions[i];
            }
            centroid /= sim.NodeCount;
            float sum = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                sum += math.distance(sim.Positions[i], centroid);
            }
            return sum / sim.NodeCount;
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
            // The outer nodes are half a cell outside the faces and enter the floor by as much (TODO MED-17).
            Assert.Greater(minY, -0.005f - 0.5f * sim.CellSize, "lattice sank into the floor");
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
        public void FullCutOfABodyLyingOnAColliderSeparates()
        {
            // The box 64 of the benchmark: after landing, friction shifts a layer of nodes across the
            // blade plane while the blade sweeps along it. Within the stroke the nodes keep the side
            // they were on when the blade passed them, so no edge joins the two sides.
            float3 size = new float3(0.15f, 0.06f, 0.1f);
            SoftBodySimulation sim = CreateBox(size, SoftBodyParameters.Default, 64, out _, out _, out _, out _, 0.006f);
            using var floor = new NativeArray<ColliderShape>(new[]
            {
                ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f),
            }, Allocator.Persistent);
            StepFrames(sim, 90, floor);

            float h = sim.CellSize;
            float3 lo = sim.BoundsMin, hi = sim.BoundsMax;
            // The blade plane runs through the middle of the central cell layer of the rest lattice.
            float z = math.transform(sim.RestToWorld, sim.Grid.Origin + ((float3)(sim.Grid.Dims / 2) + 0.5f) * h).z;
            float3 top = new float3(0.0f, hi.y + 2.0f * h, z), bottom = new float3(0.0f, lo.y - 2.0f * h, z);
            const int frames = 60;
            float3 previous = new float3(lo.x - 2.0f * h, 0.0f, 0.0f);
            for (int f = 1; f <= frames; f++)
            {
                float3 x = new float3(math.lerp(lo.x - 2.0f * h, hi.x + 2.0f * h, f / (float)frames), 0.0f, 0.0f);
                sim.Cut(previous + top, previous + bottom, x + top, x + bottom, 1);
                previous = x;
                StepFrames(sim, 1, floor);
            }
            AssertFinite(sim);
            List<int> pieces = sim.PieceSizes();
            int total = 0;
            foreach (int piece in pieces)
            {
                total += piece;
            }
            int large = pieces.FindAll(piece => piece >= 0.02f * total).Count;
            Assert.AreEqual(2, large, "piece sizes " + string.Join(", ", pieces));
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
        public void WavyStrokePutsTheCutPointsOnTheBladePath()
        {
            // A wavy stroke (3 mm amplitude, 45 mm wavelength) of 3 mm sweeps: many edges have their
            // ends passed by different sweeps and are severed by the side labels. Their cut points come
            // from the distances of the ends to the blade; the edge midpoints were up to 3 mm off.
            SoftBodySimulation sim = CutAlongWave(0.003f, 0.045f, out float2[] path);
            Assert.AreEqual(2, sim.CountPieces());
            AssertCutFollowsPath(sim, path, 0.15f);
        }

        [Test]
        public void SharplyTurningStrokeSeversOnlyTheEdgesItCrosses()
        {
            // 2.5 mm amplitude, 15 mm wavelength: the path turns within half a cell (radius about
            // 2.3 mm, cell 6 mm). Extending each sweep backwards along its own motion left the path
            // at every turn, and nodes beside the swept surface were labelled by the plane of one sweep,
            // on the wrong side where the path turns back: edges the blade never crossed were severed,
            // with cut points up to 4.2 mm off.
            // Cut points: the sweeps reach 0.1h ahead at the turns, and t is kept within [0.05, 0.95].
            SoftBodySimulation sim = CutAlongWave(0.0025f, 0.015f, out float2[] path);
            Assert.AreEqual(2, sim.CountPieces());
            AssertCutFollowsPath(sim, path, 0.2f);
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

        [Test]
        public void SaggingArmRestsOnTheBodyInsteadOfPassingThrough()
        {
            // A C shape of three boxes: lower arm, back and upper arm (0.1 x 0.02 x 0.06 m, 20 mm above the lower
            // arm), cell 5 mm, on a floor with friction. The soft upper arm sags under its weight onto the lower
            // arm: a fold of one piece. Pushed apart along the line between two nodes, nodes that had passed each
            // other under the load were held on the wrong side: the upper arm went through the lower arm and 1103
            // of the 3864 nodes kept moving faster than 0.05 m/s (TODO BUG-13).
            FoldCShape(true, out int upper, out int below, out int fast, out float maxSpeed, out int nodes);
            Assert.Less(below, 0.01f * upper, $"{below} of {upper} upper-arm nodes inside the lower arm");
            Assert.Less(fast, 0.01f * nodes, $"{fast} of {nodes} nodes faster than 0.05 m/s");
            Assert.Less(maxSpeed, 0.1f, "nodes shake");

            FoldCShape(false, out upper, out below, out _, out _, out _);
            Assert.Greater(below, 0.1f * upper, "without contacts the upper arm should pass through the lower arm");
        }

        /// <summary>
        /// The C shape of <see cref="SaggingArmRestsOnTheBodyInsteadOfPassingThrough"/> after 240 frames on the floor:
        /// upper-arm nodes inside their own surface (rest SDF at most 0) below the top of the lower arm, and the nodes
        /// faster than 0.05 m/s.
        /// </summary>
        private void FoldCShape(bool selfCollision, out int upper, out int below, out int fast, out float maxSpeed, out int nodes)
        {
            var vertices = new List<float3>();
            var triangles = new List<int>();
            void AddBox(float3 center, float3 size)
            {
                Mesh mesh = MeshUtil.CreateBox(size, 4);
                int first = vertices.Count;
                foreach (Vector3 v in mesh.vertices) vertices.Add((float3)v + center);
                foreach (int t in mesh.triangles) triangles.Add(first + t);
                Object.DestroyImmediate(mesh);
            }
            const float top = 0.02f;
            AddBox(new float3(0.0f, 0.01f, 0.0f), new float3(0.1f, 0.02f, 0.06f));
            AddBox(new float3(-0.04f, 0.03f, 0.0f), new float3(0.02f, 0.06f, 0.06f));
            AddBox(new float3(0.0f, 0.05f, 0.0f), new float3(0.1f, 0.02f, 0.06f));
            // The boxes overlap: inside is where the winding number says so.
            VoxelGrid grid = Voxelizer.Voxelize(vertices.ToArray(), triangles.ToArray(), 0.005f, closed: false);
            var parameters = SoftBodyParameters.Default;
            parameters.SelfCollision = selfCollision;
            var sim = new SoftBodySimulation(grid, parameters, float4x4.identity);
            _sims.Add(sim);
            // The floor at the bottom face (y = 0).
            using var floor = new NativeArray<ColliderShape>(new[]
            {
                ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f),
            }, Allocator.Persistent);
            StepFrames(sim, 240, floor);

            AssertFinite(sim);
            upper = below = fast = 0;
            maxSpeed = 0.0f;
            nodes = sim.NodeCount;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float speed = math.length(sim.Velocities[i]);
                maxSpeed = math.max(maxSpeed, speed);
                fast += speed > 0.05f ? 1 : 0;
                float3 rest = sim.RestPositions[i];
                if (rest.y > 0.03f && rest.x > 0.0f)
                {
                    upper++;
                    below += sim.NodeSdf[i] <= 0.0f && sim.Positions[i].y < top ? 1 : 0;
                }
            }
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
        public void RebuildJobsMatchTheManagedRebuild()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            // A full cut, a crossing cut and an oblique cut that stops inside: cells with one and two planes,
            // split points on split edges, and crack faces.
            CutPlaneX(sim, 0.003f, -0.2f, 0.2f);
            CutPlaneZ(sim, -0.004f, -0.2f, 0.2f);
            sim.Cut(new float3(-0.2f, 0.06f, 0.03f), new float3(-0.2f, -0.02f, 0.01f),
                new float3(0.02f, 0.06f, 0.035f), new float3(0.02f, -0.02f, 0.015f));
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 p = sim.Positions[i];
                sim.Velocities[i] = new float3(p.x < 0.0f ? -0.2f : 0.2f, 0.0f, p.z < 0.0f ? -0.2f : 0.2f);
            }
            StepFrames(sim, 10);

            using var job = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            job.Update(float4x4.identity);
            Assert.AreEqual(0, job.SplitOverflowCount);
            Assert.Greater(job.Mesh.vertexCount, rest.Length, "no triangle was split");
            AssertManagedRebuildMatches(sim, rest, normals, uvs, triangles, job);
        }

        /// <summary>The managed rebuild of the surface of <paramref name="sim"/> equals <paramref name="job"/>.</summary>
        private static void AssertManagedRebuildMatches(SoftBodySimulation sim, float3[] rest, float3[] normals, Vector2[] uvs,
            int[] triangles, SurfaceMesh job)
        {
            SurfaceMesh.ManagedRebuild = true;
            try
            {
                using var managed = new SurfaceMesh(sim, rest, normals, uvs, triangles);
                managed.Update(float4x4.identity);
                Assert.AreEqual(managed.Mesh.vertexCount, job.Mesh.vertexCount);
                CollectionAssert.AreEqual(managed.Mesh.GetTriangles(0), job.Mesh.GetTriangles(0));
                CollectionAssert.AreEqual(managed.Mesh.GetTriangles(1), job.Mesh.GetTriangles(1));
                Vector3[] jobVertices = job.Mesh.vertices, managedVertices = managed.Mesh.vertices;
                Vector3[] jobNormals = job.Mesh.normals, managedNormals = managed.Mesh.normals;
                var jobUv0 = new List<Vector2>();
                var managedUv0 = new List<Vector2>();
                var jobUv1 = new List<Vector3>();
                var managedUv1 = new List<Vector3>();
                job.Mesh.GetUVs(0, jobUv0);
                managed.Mesh.GetUVs(0, managedUv0);
                job.Mesh.GetUVs(1, jobUv1);
                managed.Mesh.GetUVs(1, managedUv1);
                for (int i = 0; i < jobVertices.Length; i++)
                {
                    Assert.Less(Vector3.Distance(jobVertices[i], managedVertices[i]), 1e-6f, $"vertex {i}");
                    Assert.Less(Vector3.Distance(jobNormals[i], managedNormals[i]), 1e-5f, $"normal {i}");
                    Assert.Less(Vector2.Distance(jobUv0[i], managedUv0[i]), 1e-6f, $"uv0 {i}");
                    Assert.Less(Vector3.Distance(jobUv1[i], managedUv1[i]), 1e-6f, $"uv1 {i}");
                }
            }
            finally
            {
                SurfaceMesh.ManagedRebuild = false;
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

        /// <summary>
        /// A cut passing 0.035 h from a row of lattice lines, with the edges along those lines severed in the cut plane, as a
        /// blade running along them leaves them in the Hammock of the interaction demo (TODO BUG-17): the two corners of each
        /// such edge lie on one side of the cut but became copies of their own, each drew a triangle around its corner, and
        /// the wall between the two triangles stayed open. The walls have no open edges below the surface, in the jobs and
        /// in the managed rebuild.
        /// </summary>
        [Test]
        public void CutAlongLatticeEdgesLeavesNoHoleInTheWalls()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            int3 dims = sim.Grid.Dims;
            int3 middle = dims / 2;
            // The lines x - y = k (grid units) through the middle; the plane x - y = k - 0.05 leaves them just on its high side,
            // crossing the x edges ending on them at t = 0.95 and the y edges starting on them at t = 0.05.
            var normal = math.normalize(new float3(1.0f, -1.0f, 0.0f));
            int severed = 0;
            for (int x = 0; x <= dims.x; x++)
            {
                int y = x - (middle.x - middle.y);
                for (int z = 0; z < dims.z && y >= 0 && y <= dims.y; z++)
                {
                    severed += SeverEdge(sim, new int3(x, y, z), 2, 0.3f, normal) ? 1 : 0;
                }
            }
            Assert.Greater(severed, 0);
            float3 point = math.transform(sim.RestToWorld,
                sim.Grid.Origin + new float3(middle.x - 0.05f, middle.y, 0.0f) * sim.CellSize);
            float3 along = 0.3f * math.normalize(new float3(1.0f, 1.0f, 0.0f));
            var front = new float3(0.0f, 0.0f, -0.2f);
            var back = new float3(0.0f, 0.0f, 0.2f);
            sim.Cut(point + along + front, point - along + front, point + along + back, point - along + back);
            Assert.AreEqual(2, sim.CountPieces());
            surface.Update(float4x4.identity);
            Assert.AreEqual(0, OpenCutFaceEdges(sim, surface), "open edges of the cut faces below the surface");
            AssertManagedRebuildMatches(sim, rest, normals, uvs, triangles, surface);
        }

        /// <summary>
        /// A half-depth cut 5° off the lattice axis crosses a lattice line inside the body and severs the edges along it
        /// nearly in its plane, between corners on its two sides, which stay one piece. Both walls are drawn there as well:
        /// the cut faces cover the two walls of the incision.
        /// </summary>
        [Test]
        public void SlantedPartialCutKeepsBothWalls()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            float angle = math.radians(5.0f);
            float3 Blade(float z) => new float3(0.003f + math.tan(angle) * z, 0.0f, z);
            float3 top = new float3(0.0f, 0.2f, 0.0f), tip = new float3(0.0f, 0.0f, 0.0f);
            sim.Cut(Blade(-0.2f) + top, Blade(-0.2f) + tip, Blade(0.2f) + top, Blade(0.2f) + tip);
            Assert.AreEqual(1, sim.CountPieces());
            surface.Update(float4x4.identity);

            var restPositions = new List<Vector3>();
            surface.Mesh.GetUVs(1, restPositions);
            int[] cut = surface.Mesh.GetTriangles(1);
            float area = 0.0f;
            for (int t = 0; t < cut.Length; t += 3)
            {
                float3 a = restPositions[cut[t]], b = restPositions[cut[t + 1]], c = restPositions[cut[t + 2]];
                area += 0.5f * math.length(math.cross(b - a, c - a));
            }
            // The cells holding the blade tip stay whole; the walls run from their top to the top of the box.
            float floor = sim.Grid.Origin.y + math.ceil(-sim.Grid.Origin.y / sim.CellSize) * sim.CellSize;
            float walls = 2.0f * (0.5f * Size - floor) * Size / math.cos(angle);
            TestContext.WriteLine($"cut-face area {area / walls:0.000} of two walls");
            Assert.Greater(area, 0.95f * walls, "a wall is missing");
        }

        /// <summary>
        /// Severs the edge from grid point <paramref name="start"/> along <paramref name="axis"/> in every cell sharing it,
        /// as <see cref="CutJob"/> does; false when no cell has it.
        /// </summary>
        /// <summary>
        /// Two half incisions crossing at a right angle, ending 0.3 cells above a row of lattice points (TODO BUG-21): the cells
        /// of the crossing above the floor of the incisions are split into four copies, each bounded by both cuts. One polygon
        /// through the Hermite points of both cuts cut across the corner between them, the four such polygons met the cell below
        /// (crossed but not separated, without faces) along the diagonals of their shared face, and a hole was left at the floor
        /// of the incisions. The cut faces have no open edges below the surface, in the jobs and in the managed rebuild.
        /// </summary>
        [Test]
        public void CrossingHalfIncisionsLeaveNoHoleAtTheirFloor()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false, out float3[] rest, out float3[] normals,
                out Vector2[] uvs, out int[] triangles);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            int3 middle = sim.Grid.Dims / 2;
            float3 p = math.transform(sim.RestToWorld, sim.Grid.Origin + ((float3)middle + new float3(0.37f, 0.3f, 0.41f)) * sim.CellSize);
            sim.Cut(new float3(p.x, 0.5f, -0.2f), new float3(p.x, p.y, -0.2f), new float3(p.x, 0.5f, 0.2f), new float3(p.x, p.y, 0.2f));
            sim.Cut(new float3(-0.2f, 0.5f, p.z), new float3(-0.2f, p.y, p.z), new float3(0.2f, 0.5f, p.z), new float3(0.2f, p.y, p.z));
            Assert.AreEqual(1, sim.CountPieces());
            surface.Update(float4x4.identity);
            Assert.AreEqual(0, OpenCutFaceEdges(sim, surface), "open edges of the cut faces below the surface");
            AssertManagedRebuildMatches(sim, rest, normals, uvs, triangles, surface);
        }

        private static bool SeverEdge(SoftBodySimulation sim, int3 start, int axis, float t, float3 normal)
        {
            int3 dims = sim.Grid.Dims;
            int axisB = (axis + 1) % 3, axisC = (axis + 2) % 3;
            bool any = false;
            for (int ob = 0; ob < 2; ob++)
            {
                for (int oc = 0; oc < 2; oc++)
                {
                    int3 offset = int3.zero;
                    offset[axisB] = ob;
                    offset[axisC] = oc;
                    int3 cell = start - offset;
                    if (math.any(cell < 0) || math.any(cell >= dims))
                    {
                        continue;
                    }
                    int c = sim.GridToCell[cell.x + dims.x * (cell.y + dims.y * cell.z)];
                    if (c < 0)
                    {
                        continue;
                    }
                    int e = Lattice.EdgeFromStart(axis, Lattice.CornerIndex(offset));
                    sim.Severed[c] = (ushort)(sim.Severed[c] | (1 << e));
                    sim.EdgeT[c * Lattice.EdgeCount + e] = t;
                    sim.EdgeNormal[c * Lattice.EdgeCount + e] = normal;
                    any = true;
                }
            }
            return any;
        }

        /// <summary>
        /// Edges of the cut faces, with vertices welded by rest position and piece, that belong to one triangle only and lie
        /// below the surface (along the surface the cut faces meet the skin).
        /// </summary>
        private static int OpenCutFaceEdges(SoftBodySimulation sim, SurfaceMesh surface)
        {
            var depth = new List<Vector2>();
            var restPositions = new List<Vector3>();
            surface.Mesh.GetUVs(0, depth);
            surface.Mesh.GetUVs(1, restPositions);
            int[] cut = surface.Mesh.GetTriangles(1);
            var weld = new Dictionary<(long, long, long, int), int>();
            var welded = new List<int>();
            int Id(int v)
            {
                int copy = surface.VertexCopy(v);
                int node = -1;
                for (int k = 0; k < 8 && node < 0; k++)
                {
                    node = ((sim.Copies[copy].Mask >> k) & 1) != 0 ? sim.CopyNodes[copy * 8 + k] : -1;
                }
                Vector3 p = restPositions[v];
                var key = ((long)math.round(p.x * 1e6), (long)math.round(p.y * 1e6), (long)math.round(p.z * 1e6), sim.PieceId[node]);
                if (!weld.TryGetValue(key, out int id))
                {
                    id = weld.Count;
                    weld[key] = id;
                    welded.Add(v);
                }
                return id;
            }
            var uses = new Dictionary<(int, int), int>();
            for (int t = 0; t < cut.Length; t += 3)
            {
                for (int i = 0; i < 3; i++)
                {
                    int a = Id(cut[t + i]), b = Id(cut[t + (i + 1) % 3]);
                    var edge = (math.min(a, b), math.max(a, b));
                    uses[edge] = uses.TryGetValue(edge, out int n) ? n + 1 : 1;
                }
            }
            float below = 0.05f * sim.CellSize;
            int open = 0;
            foreach (KeyValuePair<(int, int), int> edge in uses)
            {
                open += edge.Value == 1 && depth[welded[edge.Key.Item1]].x > below && depth[welded[edge.Key.Item2]].x > below ? 1 : 0;
            }
            TestContext.WriteLine($"open cut-face edges below the surface: {open}");
            return open;
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

        /// <summary>
        /// A stroke that leaves a half incision at a slant (TODO BUG-18): a box 12 x 5 x 10 cm (cell 5 mm, no gravity) with an
        /// incision 2 cm deep along z; a second stroke, as deep, starts on it and leaves it at 10 degrees. Over the first 2 cm the
        /// two cuts are less than 0.75 cells apart. When the stroke ends, every edge it severed is still severed: the thin layer
        /// between the cuts joins the far side of the incision, whose edges there are restored. Before, the stroke's own edges
        /// there were restored, and its cut started 2 cm away from the incision.
        /// </summary>
        [Test]
        public void StrokeLeavingAHalfIncisionKeepsItsWholeCut()
        {
            SoftBodySimulation sim = HalfIncision(out ushort[] incision, out float top);
            float slope = math.tan(math.radians(10.0f));
            ushort[] stroke = Stroke(sim, 2, new float3(0.0013f, 0.0f, -0.02f), new float3(0.0013f + 0.06f * slope, 0.0f, 0.04f), top);
            int healed = sim.EndStroke(2);
            int lost = 0, restored = 0;
            for (int c = 0; c < sim.CellCount; c++)
            {
                lost += math.countbits((uint)(stroke[c] & ~incision[c] & ~sim.Severed[c]));
                restored += math.countbits((uint)(incision[c] & ~sim.Severed[c]));
            }
            TestContext.WriteLine($"edges restored at the end of the stroke: {healed}; in the cells: of the incision {restored}, of the stroke {lost}");
            Assert.Greater(healed, 0, "the stroke ran alongside the incision");
            Assert.AreEqual(0, lost, "edges of the stroke restored");
            Assert.Greater(restored, 0);
        }

        /// <summary>
        /// A stroke along a half incision 2.2 mm (0.44 cells) beside it, with a layer of lattice nodes between the two (TODO
        /// BUG-18): more than half of the edges it severs lie alongside the incision, so it traced the incision again: its edges
        /// there are restored and the incision keeps all of its own.
        /// </summary>
        [Test]
        public void StrokeAlongAHalfIncisionKeepsTheIncision()
        {
            SoftBodySimulation sim = HalfIncision(out ushort[] incision, out float top);
            ushort[] stroke = Stroke(sim, 2, new float3(0.0035f, 0.0f, -0.035f), new float3(0.0035f, 0.0f, 0.035f), top);
            int healed = sim.EndStroke(2);
            int lost = 0, restored = 0;
            for (int c = 0; c < sim.CellCount; c++)
            {
                lost += math.countbits((uint)(incision[c] & ~sim.Severed[c]));
                restored += math.countbits((uint)(stroke[c] & ~incision[c] & ~sim.Severed[c]));
            }
            TestContext.WriteLine($"edges restored at the end of the stroke: {healed}; in the cells: of the stroke {restored}, of the incision {lost}");
            Assert.Greater(healed, 0, "the stroke ran alongside the incision");
            Assert.AreEqual(0, lost, "edges of the incision restored");
            Assert.Greater(restored, 0);
        }

        /// <summary>
        /// A box 12 x 5 x 10 cm (cell 5 mm, no gravity) with an incision 2 cm deep along z at x = 1.3 mm, ended; the severed
        /// edges afterwards and the height of the top.
        /// </summary>
        private SoftBodySimulation HalfIncision(out ushort[] severed, out float top)
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(new float3(0.12f, 0.05f, 0.1f), parameters, 24);
            top = sim.BoundsMax.y;
            Stroke(sim, 1, new float3(0.0013f, 0.0f, -0.04f), new float3(0.0013f, 0.0f, 0.04f), top);
            Assert.AreEqual(0, sim.EndStroke(1));
            Assert.AreEqual(1, sim.CountPieces());
            severed = sim.Severed.ToArray();
            return sim;
        }

        /// <summary>
        /// A stroke of a vertical blade from 1 cm above <paramref name="top"/> to 2 cm below it, moved from
        /// <paramref name="from"/> to <paramref name="to"/> (x, z) in 1 mm sweeps; the severed edges at its end.
        /// </summary>
        private static ushort[] Stroke(SoftBodySimulation sim, int stroke, float3 from, float3 to, float top)
        {
            int sweeps = (int)math.ceil(math.distance(from.xz, to.xz) / 0.001f);
            float3 Point(int i) => math.lerp(from, to, i / (float)sweeps);
            for (int i = 1; i <= sweeps; i++)
            {
                float3 a = Point(i - 1), b = Point(i);
                sim.Cut(new float3(a.x, top + 0.01f, a.z), new float3(a.x, top - 0.02f, a.z), new float3(b.x, top + 0.01f, b.z),
                    new float3(b.x, top - 0.02f, b.z), stroke);
            }
            return sim.Severed.ToArray();
        }

        /// <summary>
        /// A cut that only severs edges updates the topology instead of rebuilding it (TODO PERF-5), with the rebuild's
        /// result bit for bit, also after stepping on. Two boxes pinned at one end and sagging under gravity take the same
        /// strokes, one rebuilding every time: a stroke through, a half-depth stroke across it, a stroke along the first
        /// incision a fifth of a cell beside it, a stroke after a severed edge was restored (as EndStroke restores the
        /// edges it heals) and one after restoring a state. The first cut after those two rebuilds the topology.
        /// </summary>
        [Test]
        public void TopologyUpdateMatchesTheRebuild()
        {
            var size = new float3(0.12f, 0.05f, 0.1f);
            SoftBodySimulation updated = CreateBox(size, SoftBodyParameters.Default, 16);
            SoftBodySimulation rebuilt = CreateBox(size, SoftBodyParameters.Default, 16);
            rebuilt.TopologyUpdates = false;
            const float cutX = 0.0123f;
            foreach (SoftBodySimulation sim in new[] { updated, rebuilt })
            {
                var pinned = new List<int>();
                for (int i = 0; i < sim.NodeCount; i++)
                {
                    if (sim.RestPositions[i].x < -0.05f)
                    {
                        pinned.Add(i);
                    }
                }
                sim.Attach(pinned, 1, float4x4.identity);
                // Material points beside the strokes follow the copies (SUT-1).
                foreach (float dx in new[] { -0.3f, 0.3f, 2.0f })
                {
                    Assert.GreaterOrEqual(sim.AddMaterialPoint(new float3(cutX + dx * sim.CellSize, 0.01f, 0.0071f + dx * sim.CellSize)), 0);
                }
            }
            using var shapes = new NativeArray<ColliderShape>(0, Allocator.Persistent);
            int changes = 0, updates = 0;

            // Whether the first cut of the stroke that changed the topology updated it.
            bool Stroke(int stroke, float3 heel, float3 tip, float3 move)
            {
                const int sweeps = 12;
                bool? firstUpdated = null;
                for (int s = 0; s < sweeps; s++)
                {
                    float3 from = move * s / sweeps, to = move * (s + 1) / sweeps;
                    int severed = updated.Cut(heel + from, tip + from, heel + to, tip + to, stroke);
                    Assert.AreEqual(rebuilt.Cut(heel + from, tip + from, heel + to, tip + to, stroke), severed);
                    if (severed > 0)
                    {
                        changes++;
                        updates += updated.LastTopologyUpdated ? 1 : 0;
                        firstUpdated ??= updated.LastTopologyUpdated;
                        AssertSameState(rebuilt, updated, $"stroke {stroke}, sweep {s}");
                    }
                    updated.Step(Dt, Substeps, shapes);
                    rebuilt.Step(Dt, Substeps, shapes);
                }
                int healed = updated.EndStroke(stroke);
                Assert.AreEqual(rebuilt.EndStroke(stroke), healed);
                TestContext.WriteLine($"stroke {stroke}: healed {healed}");
                AssertSameState(rebuilt, updated, $"end of stroke {stroke}");
                Assert.IsTrue(firstUpdated.HasValue, $"stroke {stroke} severed nothing");
                return firstUpdated.Value;
            }

            Assert.IsTrue(Stroke(1, new float3(cutX, 0.1f, -0.08f), new float3(cutX, -0.1f, -0.08f), new float3(0.0f, 0.0f, 0.16f)));
            Assert.GreaterOrEqual(updated.CountPieces(), 2);
            SoftBodyState savedUpdated = updated.CaptureState(), savedRebuilt = rebuilt.CaptureState();
            Assert.IsTrue(Stroke(2, new float3(-0.07f, 0.1f, 0.0071f), new float3(-0.07f, 0.0f, 0.0071f), new float3(0.14f, 0.0f, 0.0f)));
            float beside = cutX - 0.2f * updated.CellSize;
            Assert.IsTrue(Stroke(3, new float3(beside, 0.1f, 0.08f), new float3(beside, -0.1f, 0.08f), new float3(0.0f, 0.0f, -0.16f)));
            RestoreFirstSeveredEdge(updated);
            RestoreFirstSeveredEdge(rebuilt);
            Assert.IsFalse(Stroke(5, new float3(-0.07f, 0.1f, 0.035f), new float3(-0.07f, 0.0f, 0.035f), new float3(0.14f, 0.0f, 0.0f)));
            updated.RestoreState(savedUpdated);
            rebuilt.RestoreState(savedRebuilt);
            Assert.IsFalse(Stroke(4, new float3(-0.07f, 0.1f, -0.031f), new float3(-0.07f, 0.0f, -0.031f), new float3(0.14f, 0.0f, 0.0f)));
            for (int f = 0; f < 30; f++)
            {
                updated.Step(Dt, Substeps, shapes);
                rebuilt.Step(Dt, Substeps, shapes);
            }
            AssertSameState(rebuilt, updated, "30 frames after the last stroke");
            TestContext.WriteLine($"topology changes {changes}, updates {updates}");
            Assert.Greater(updates, changes / 2);
        }

        /// <summary>Restores the first severed edge, in the order of the cells, on all cells sharing it.</summary>
        private static void RestoreFirstSeveredEdge(SoftBodySimulation sim)
        {
            int c = 0;
            while (sim.Severed[c] == 0)
            {
                c++;
            }
            int e = math.tzcnt((uint)sim.Severed[c]);
            int axis = e / 4;
            int3 start = sim.CellCoord[c] + Lattice.CornerOffset(Lattice.EdgeCorners[2 * e]);
            int3 dims = sim.Grid.Dims;
            for (int ob = 0; ob < 2; ob++)
            {
                for (int oc = 0; oc < 2; oc++)
                {
                    int3 offset = int3.zero;
                    offset[(axis + 1) % 3] = ob;
                    offset[(axis + 2) % 3] = oc;
                    int3 cell = start - offset;
                    if (math.any(cell < 0) || math.any(cell >= dims))
                    {
                        continue;
                    }
                    int n = sim.GridToCell[cell.x + dims.x * (cell.y + dims.y * cell.z)];
                    if (n >= 0)
                    {
                        int edge = Lattice.EdgeFromStart(axis, Lattice.CornerIndex(offset));
                        sim.Severed[n] = (ushort)(sim.Severed[n] & ~(1 << edge));
                    }
                }
            }
        }

        private static void AssertSameState(SoftBodySimulation expected, SoftBodySimulation actual, string context)
        {
            AssertSameBits(expected.CellCopyRange, actual.CellCopyRange, context + ", copy ranges");
            Assert.AreEqual(expected.CopyCount, actual.CopyCount, context + ", copies");
            for (int q = 0; q < expected.CopyCount; q++)
            {
                CellCopy e = expected.Copies[q], a = actual.Copies[q];
                Assert.IsTrue(e.Cell == a.Cell && e.Mask == a.Mask && math.asuint(e.Weight) == math.asuint(a.Weight),
                    $"{context}, copy {q}");
            }
            AssertSameBits(expected.CopyNodes.AsArray(), actual.CopyNodes.AsArray(), context + ", copy nodes");
            AssertSameBits(expected.Positions, actual.Positions, context + ", positions");
            AssertSameBits(expected.Previous, actual.Previous, context + ", previous positions");
            AssertSameBits(expected.Velocities, actual.Velocities, context + ", velocities");
            AssertSameBits(expected.RestPositions, actual.RestPositions, context + ", rest positions");
            AssertSameBits(expected.InverseMass, actual.InverseMass, context + ", inverse masses");
            AssertSameBits(expected.KinematicId, actual.KinematicId, context + ", kinematic frames");
            AssertSameBits(expected.KinematicLocal, actual.KinematicLocal, context + ", kinematic positions");
            AssertSameBits(expected.NodeReal, actual.NodeReal, context + ", real nodes");
            AssertSameBits(expected.PieceId, actual.PieceId, context + ", pieces");
            AssertSameBits(expected.StrokeSide, actual.StrokeSide, context + ", stroke sides");
            AssertSameBits(expected.StrokeTravel, actual.StrokeTravel, context + ", stroke travel");
            AssertSameBits(expected.StrokeDistance, actual.StrokeDistance, context + ", stroke distances");
            AssertSameBits(expected.NodeSdf, actual.NodeSdf, context + ", node SDF");
            AssertSameBits(expected.MaterialPoints, actual.MaterialPoints, context + ", material points");
        }

        /// <summary>
        /// Material points made at the rest positions of skin vertices move with them (TODO SUT-1): the cell and the
        /// position in it are the vertex's, so is the trilinear interpolation. A point made at a deformed position is there.
        /// </summary>
        [Test]
        public void MaterialPointsMoveWithTheSkin()
        {
            SoftBodySimulation sim = CreateBox(new float3(0.12f, 0.03f, 0.05f), SoftBodyParameters.Default, 12,
                out float3[] rest, out float3[] normals, out Vector2[] uvs, out int[] triangles, 0.0f);
            var pinned = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.RestPositions[i].x < -0.05f)
                {
                    pinned.Add(i);
                }
            }
            sim.Attach(pinned, 1, float4x4.identity);
            using var surface = new SurfaceMesh(sim, rest, normals, uvs, triangles);
            var points = new List<int>();
            var vertices = new List<int>();
            for (int v = 0; v < rest.Length; v += 53)
            {
                int point = sim.AddMaterialPoint(math.transform(sim.RestToWorld, rest[v]));
                Assert.GreaterOrEqual(point, 0, $"vertex {v}");
                points.Add(point);
                vertices.Add(v);
            }
            StepFrames(sim, 40);
            surface.Update(float4x4.identity);
            Vector3[] deformed = surface.Mesh.vertices;
            float largestMove = 0.0f, worst = 0.0f, worstAgain = 0.0f;
            for (int i = 0; i < points.Count; i++)
            {
                float3 x = sim.MaterialPointPosition(points[i]);
                largestMove = math.max(largestMove, math.distance(x, math.transform(sim.RestToWorld, rest[vertices[i]])));
                worst = math.max(worst, math.distance(x, deformed[vertices[i]]));
                int again = sim.AddMaterialPoint(x);
                Assert.GreaterOrEqual(again, 0, $"deformed position of point {i}");
                worstAgain = math.max(worstAgain, math.distance(sim.MaterialPointPosition(again), x));
            }
            TestContext.WriteLine($"largest move {largestMove * 1000:F2} mm, skin {worst * 1e6:F3} µm, again {worstAgain * 1e6:F3} µm");
            Assert.Greater(largestMove, 0.005f, "the free end sags");
            Assert.Less(worst, 1e-6f);
            Assert.Less(worstAgain, 1e-6f);
            Assert.IsTrue(sim.RemoveMaterialPoint(points[0]));
            Assert.IsFalse(sim.IsMaterialPoint(points[0]));
            Assert.IsFalse(sim.RemoveMaterialPoint(points[0]));
        }

        /// <summary>
        /// Points a fifth of a cell and a cell and a half on either side of a cut through their cells end up in copies whose
        /// real corners are all on their side, in the two pieces (TODO SUT-1).
        /// </summary>
        [Test]
        public void MaterialPointsKeepTheirSideOfACut()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            float h = sim.CellSize;
            const float cutX = 0.004f;
            float[] offsets = { -1.5f, -0.2f, 0.2f, 1.5f };
            var points = new int[offsets.Length];
            for (int i = 0; i < offsets.Length; i++)
            {
                points[i] = sim.AddMaterialPoint(new float3(cutX + offsets[i] * h, 0.011f, -0.013f));
                Assert.GreaterOrEqual(points[i], 0);
            }
            Assert.Greater(CutPlaneX(sim, cutX, -0.2f, 0.2f), 0);
            for (int i = 0; i < offsets.Length; i++)
            {
                MaterialPoint point = sim.GetMaterialPoint(points[i]);
                CellCopy copy = sim.Copies[point.Copy];
                float cellX = sim.Grid.Origin.x + sim.CellCoord[copy.Cell].x * h;
                Assert.AreEqual(offsets[i] > 0.0f, cellX + point.U.x * h > cutX, $"point {i} moved in its cell");
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    if (((copy.Mask >> k) & 1) != 0)
                    {
                        Assert.AreEqual(offsets[i] > 0.0f, cellX + Lattice.CornerOffset(k).x * h > cutX, $"point {i}, corner {k}");
                    }
                }
            }
            Assert.AreEqual(sim.MaterialPointPiece(points[0]), sim.MaterialPointPiece(points[1]));
            Assert.AreEqual(sim.MaterialPointPiece(points[2]), sim.MaterialPointPiece(points[3]));
            Assert.AreNotEqual(sim.MaterialPointPiece(points[1]), sim.MaterialPointPiece(points[2]));
        }

        /// <summary>
        /// A point cut off with one corner of its cell stays with that corner, although the other seven corners carry more
        /// of its trilinear weight: the side comes from the cut point and normal of the nearest severed edge (TODO SUT-1).
        /// </summary>
        [Test]
        public void MaterialPointCutOffWithACornerStaysWithIt()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), gravity: false);
            int3 dims = sim.Grid.Dims;
            int3 coord = dims / 2;
            int cell = sim.GridToCell[coord.x + dims.x * (coord.y + dims.y * coord.z)];
            Assert.AreEqual(1, sim.CellCopyRange[cell].y);
            int copy = sim.CellCopyRange[cell].x;
            // The plane x + y + z = 0.9 (cell units) cuts off corner 0; the corner has 0.37 of the weight at the first point.
            int cutOff = sim.AddMaterialPoint(copy, new float3(0.3f, 0.3f, 0.25f));
            int rest = sim.AddMaterialPoint(copy, new float3(0.6f, 0.5f, 0.5f));
            float3 normal = math.normalize(new float3(1.0f));
            for (int axis = 0; axis < 3; axis++)
            {
                Assert.IsTrue(SeverEdge(sim, coord, axis, 0.9f, normal));
            }
            // A cut elsewhere rebuilds the topology.
            Assert.Greater(CutPlaneX(sim, -0.04f, -0.2f, 0.2f), 0);
            CellCopy a = sim.Copies[sim.GetMaterialPoint(cutOff).Copy], b = sim.Copies[sim.GetMaterialPoint(rest).Copy];
            Assert.AreEqual(cell, a.Cell);
            Assert.AreEqual(cell, b.Cell);
            Assert.AreEqual(0x01, a.Mask);
            Assert.AreEqual(0xFE, b.Mask);
        }

        private static void AssertSameBits<T>(NativeArray<T> expected, NativeArray<T> actual, string what) where T : struct
        {
            Assert.AreEqual(expected.Length, actual.Length, what + ", length");
            int size = UnsafeUtility.SizeOf<T>();
            NativeArray<byte> e = expected.Reinterpret<byte>(size), a = actual.Reinterpret<byte>(size);
            for (int i = 0; i < e.Length; i++)
            {
                if (e[i] != a[i])
                {
                    Assert.Fail($"{what}: element {i / size} differs");
                }
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

        /// <summary>
        /// Box 0.12 x 0.05 x 0.1 m at resolution 20 (cell 6 mm) without gravity, cut in one stroke of 60
        /// sweeps by a vertical blade along z, at x = amplitude * sin(2 pi z / wavelength).
        /// </summary>
        private SoftBodySimulation CutAlongWave(float amplitude, float wavelength, out float2[] path)
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(new float3(0.12f, 0.05f, 0.1f), parameters, 20);
            const int frames = 60;
            path = new float2[frames + 1];
            for (int f = 0; f <= frames; f++)
            {
                float z = math.lerp(-0.09f, 0.09f, f / (float)frames);
                path[f] = new float2(amplitude * math.sin(2.0f * math.PI * z / wavelength), z);
            }
            for (int f = 1; f <= frames; f++)
            {
                sim.Cut(new float3(path[f - 1].x, 0.06f, path[f - 1].y), new float3(path[f - 1].x, -0.06f, path[f - 1].y),
                    new float3(path[f].x, 0.06f, path[f].y), new float3(path[f].x, -0.06f, path[f].y), 1);
            }
            return sim;
        }

        /// <summary>
        /// Every severed edge passes within 0.1 cells of the path of a vertical blade (seen from above;
        /// each sweep reaches 0.1 cells ahead), and its cut point lies within <paramref name="maxDistance"/>
        /// cells of the path.
        /// </summary>
        private static void AssertCutFollowsPath(SoftBodySimulation sim, float2[] path, float maxDistance)
        {
            float h = sim.CellSize;
            float worst = 0.0f;
            int points = 0;
            var away = new List<string>();
            for (int c = 0; c < sim.CellCount; c++)
            {
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((sim.Severed[c] & (1 << e)) == 0)
                    {
                        continue;
                    }
                    float3 g0 = Lattice.CornerOffset(Lattice.EdgeCorners[2 * e]);
                    float3 g1 = Lattice.CornerOffset(Lattice.EdgeCorners[2 * e + 1]);
                    float3 cell = sim.Grid.Origin + (float3)sim.CellCoord[c] * h;
                    float2 x0 = math.transform(sim.RestToWorld, cell + g0 * h).xz;
                    float2 x1 = math.transform(sim.RestToWorld, cell + g1 * h).xz;
                    float2 p = math.lerp(x0, x1, sim.EdgeT[c * Lattice.EdgeCount + e]);
                    float distance = float.MaxValue;
                    float reach = float.MaxValue;
                    for (int f = 1; f < path.Length; f++)
                    {
                        distance = math.min(distance, SegmentDistance(p, p, path[f - 1], path[f]));
                        reach = math.min(reach, SegmentDistance(x0, x1, path[f - 1], path[f]));
                    }
                    if (reach > 0.1f * h && away.Count < 10)
                    {
                        away.Add($"cell {sim.CellCoord[c]} edge {e} from ({x0.x * 1000.0f:0.0}, {x0.y * 1000.0f:0.0}) " +
                                 $"to ({x1.x * 1000.0f:0.0}, {x1.y * 1000.0f:0.0}) mm, {reach * 1000.0f:0.00} mm from the path");
                    }
                    worst = math.max(worst, distance);
                    points++;
                }
            }
            Assert.Greater(points, 0);
            Assert.IsEmpty(away, "severed edges farther than 0.1 cells from the blade path:\n" + string.Join("\n", away));
            Assert.Less(worst, maxDistance * h, $"cut point {worst * 1000.0f:0.00} mm from the blade path, cell {h * 1000.0f:0.00} mm");
        }

        /// <summary>Distance between segments pq and ab in the plane, 0 where they cross.</summary>
        private static float SegmentDistance(float2 p, float2 q, float2 a, float2 b)
        {
            static float Cross(float2 u, float2 v) => u.x * v.y - u.y * v.x;
            static float PointDistance(float2 x, float2 s0, float2 s1)
            {
                float2 d = s1 - s0;
                float s = math.lengthsq(d) > 0.0f ? math.saturate(math.dot(x - s0, d) / math.lengthsq(d)) : 0.0f;
                return math.distance(x, s0 + s * d);
            }
            float d1 = Cross(b - a, p - a), d2 = Cross(b - a, q - a);
            float d3 = Cross(q - p, a - p), d4 = Cross(q - p, b - p);
            if (d1 * d2 < 0.0f && d3 * d4 < 0.0f)
            {
                return 0.0f;
            }
            return math.min(math.min(PointDistance(p, a, b), PointDistance(q, a, b)),
                math.min(PointDistance(a, p, q), PointDistance(b, p, q)));
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
