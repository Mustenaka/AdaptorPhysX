using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
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
        public void SplitJobMatchesTheManagedSplit()
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
            SurfaceMesh.ManagedSplitOnly = true;
            try
            {
                using var managed = new SurfaceMesh(sim, rest, normals, uvs, triangles);
                managed.Update(float4x4.identity);
                Assert.AreEqual(0, job.SplitOverflowCount);
                Assert.Greater(job.Mesh.vertexCount, rest.Length, "no triangle was split");
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
                SurfaceMesh.ManagedSplitOnly = false;
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
