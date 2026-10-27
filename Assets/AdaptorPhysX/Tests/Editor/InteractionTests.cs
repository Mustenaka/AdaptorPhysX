using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Tests
{
    /// <summary>
    /// Contact with other bodies and instruments: distance-field colliders, collider forces,
    /// rigid-body coupling, body-body contact, adhesion, plasticity, state snapshots and blueprints
    /// (TODO ECO-1 to ECO-3, ECO-6 to ECO-8, HAP-1 to HAP-3). Component tests run in a preview scene with its
    /// own physics scene, so the scene open in the editor neither takes part nor changes.
    /// </summary>
    public sealed class InteractionTests
    {
        private const float Size = 0.1f;
        private const float Dt = 1.0f / 60.0f;
        private const int Substeps = 16;

        private readonly List<SoftBodySimulation> _sims = new List<SoftBodySimulation>();
        private readonly List<Object> _objects = new List<Object>();
        private readonly List<ApxSoftBody> _bodies = new List<ApxSoftBody>();
        private readonly List<System.IDisposable> _natives = new List<System.IDisposable>();
        private Scene _scene;

        [TearDown]
        public void TearDown()
        {
            foreach (SoftBodySimulation sim in _sims)
            {
                sim.Dispose();
            }
            _sims.Clear();
            foreach (System.IDisposable native in _natives)
            {
                native.Dispose();
            }
            _natives.Clear();
            // Edit mode calls no OnDestroy: release the bodies so they leave ApxSoftBody.Active.
            foreach (ApxSoftBody body in _bodies)
            {
                body.Release();
            }
            _bodies.Clear();
            foreach (Object o in _objects)
            {
                if (o != null)
                {
                    Object.DestroyImmediate(o);
                }
            }
            _objects.Clear();
            if (_scene.IsValid())
            {
                EditorSceneManager.ClosePreviewScene(_scene);
            }
        }

        [Test]
        public void DistanceFieldMatchesTheBoxDistance()
        {
            var size = new Vector3(0.1f, 0.06f, 0.08f);
            Mesh mesh = Track(MeshUtil.CreateBox(size, 4));
            DistanceField field = DistanceField.Bake(mesh.vertices, mesh.triangles, 32, 0.0f);
            using var data = new NativeArray<float>(field.Values, Allocator.Temp);
            float3 half = 0.5f * (float3)size;
            var random = new Unity.Mathematics.Random(7);
            // The grid extends two cells (6.25 mm) beyond the mesh.
            for (int i = 0; i < 500; i++)
            {
                float3 p = random.NextFloat3(-half - 0.006f, half + 0.006f);
                float exact = BoxDistance(p, half);
                float sampled = DistanceField.Sample(data, 0, field.Dims, field.Origin, field.CellSize, p, out float3 gradient);
                Assert.AreEqual(exact, sampled, 0.5f * field.CellSize, $"distance at {p}");
                if (math.abs(exact) > field.CellSize)
                {
                    Assert.AreEqual(math.sign(exact), math.sign(sampled), $"sign at {p}");
                }
                Assert.IsTrue(math.all(math.isfinite(gradient)));
            }
            // Beyond the grid the value is an upper bound of the distance (no false contact).
            for (int i = 0; i < 200; i++)
            {
                float3 p = random.NextFloat3(-half - 0.03f, half + 0.03f);
                if (math.all(math.abs(p) < half + 0.007f))
                {
                    continue;
                }
                float sampled = DistanceField.Sample(data, 0, field.Dims, field.Origin, field.CellSize, p, out _);
                Assert.GreaterOrEqual(sampled, BoxDistance(p, half) - 0.5f * field.CellSize, $"distance at {p}");
            }
        }

        private static float BoxDistance(float3 p, float3 half)
        {
            float3 q = math.abs(p) - half;
            return math.length(math.max(q, 0.0f)) + math.min(math.cmax(q), 0.0f);
        }

        [Test]
        public void DistanceFieldFloorHoldsTheBodyLikeABoxCollider()
        {
            // A flat floor is linear in the field, so the body must land as on the box primitive.
            Mesh floorMesh = Track(MeshUtil.CreateBox(new Vector3(0.6f, 0.2f, 0.6f), 2));
            DistanceField field = DistanceField.Bake(floorMesh.vertices, floorMesh.triangles, 32, 0.0f);
            float BoxMinY() => DropAndMeasure(ColliderShape.Box(new float3(0.0f, -0.1f, 0.0f), quaternion.identity,
                new float3(0.3f, 0.1f, 0.3f), 0.6f), null);
            float FieldMinY() => DropAndMeasure(ColliderShape.DistanceField(new float3(0.0f, -0.1f, 0.0f), quaternion.identity,
                new float3(1.0f), 0, field, 0.6f), field.Values);
            float box = BoxMinY();
            float sdf = FieldMinY();
            TestContext.WriteLine($"lowest node: box {box * 1000:0.000} mm, field {sdf * 1000:0.000} mm");
            Assert.AreEqual(box, sdf, 5e-4f, $"box {box * 1000:0.00} mm, field {sdf * 1000:0.00} mm");
        }

        private float DropAndMeasure(ColliderShape floor, float[] fields)
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), SoftBodyParameters.Default, 8, 0.01f);
            using var shapes = new NativeArray<ColliderShape>(new[] { floor }, Allocator.Persistent);
            using var fieldData = new NativeArray<float>(fields ?? new float[1], Allocator.Persistent);
            for (int f = 0; f < 120; f++)
            {
                sim.Step(Dt, Substeps, shapes, fieldData);
            }
            AssertFinite(sim);
            float minY = float.MaxValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                minY = math.min(minY, sim.Positions[i].y);
            }
            Assert.Less(minY, 0.005f, "body did not land");
            return minY;
        }

        [Test]
        public void MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider()
        {
            // The component paths to a distance field: a MeshCollider baked at runtime, and an
            // ApxDistanceField asset on a collider that only bounds it. Each floor is a unit cube scaled to
            // 0.6 x 0.2 x 0.6 m with its top face at y = 0.
            Mesh unitBox = Track(MeshUtil.CreateBox(Vector3.one, 2));
            float boxY = LowestNodeOnFloor(floor => floor.AddComponent<BoxCollider>());
            float meshY = LowestNodeOnFloor(floor => floor.AddComponent<MeshCollider>().sharedMesh = unitBox);
            ApxDistanceField baked = Track(ScriptableObject.CreateInstance<ApxDistanceField>());
            baked.mesh = unitBox;
            baked.Bake();
            // A copy through serialization, as an asset loaded from disk: the field comes from the
            // serialized values without baking again.
            ApxDistanceField asset = Track(Object.Instantiate(baked));
            Assert.IsTrue(asset.IsBaked, "the baked values were not serialized");
            float fieldY = LowestNodeOnFloor(floor =>
            {
                // The box reaches 50 mm above the field's top face: the body lands at y = 0 only when the
                // field replaces the collider's shape.
                floor.AddComponent<BoxCollider>().size = new Vector3(1.0f, 1.5f, 1.0f);
                floor.AddComponent<ApxColliderMaterial>().distanceField = asset;
            });
            TestContext.WriteLine($"lowest node: box {boxY * 1000:0.000} mm, mesh collider {meshY * 1000:0.000} mm, " +
                                  $"field asset {fieldY * 1000:0.000} mm");
            Assert.AreEqual(boxY, meshY, 5e-4f, $"box {boxY * 1000:0.00} mm, mesh collider {meshY * 1000:0.00} mm");
            Assert.AreEqual(boxY, fieldY, 5e-4f, $"box {boxY * 1000:0.00} mm, field asset {fieldY * 1000:0.00} mm");
        }

        /// <summary>Drops a 10 cm cube from 10 mm onto the floor <paramref name="addCollider"/> builds; the lowest node height after 2 s.</summary>
        private float LowestNodeOnFloor(System.Action<GameObject> addCollider)
        {
            GameObject floor = InScene(new GameObject("Floor"));
            floor.transform.position = new Vector3(0.0f, -0.1f, 0.0f);
            floor.transform.localScale = new Vector3(0.6f, 0.2f, 0.6f);
            addCollider(floor);
            ApxSoftBody body = CreateBody("Body", new Vector3(Size, Size, Size), new Vector3(0.0f, 0.5f * Size + 0.01f, 0.0f), 8);
            Physics.SyncTransforms();
            body.Initialize();
            for (int f = 0; f < 120; f++)
            {
                body.Simulate(Dt);
                body.UpdateRender();
            }
            AssertFinite(body.Simulation);
            float minY = float.MaxValue;
            for (int i = 0; i < body.Simulation.NodeCount; i++)
            {
                minY = math.min(minY, body.Simulation.Positions[i].y);
            }
            Release(body);
            Object.DestroyImmediate(floor);
            Assert.Less(minY, 0.005f, "body did not land");
            return minY;
        }

        [Test]
        public void ColliderForceEqualsTheWeightOfARestingBody()
        {
            var parameters = SoftBodyParameters.Default;
            parameters.Damping = 2.0f;
            SoftBodySimulation sim = CreateBox(new float3(Size), parameters, 8, 0.002f + LatticeOffset(8));
            using var floor = Floor();
            using var fields = new NativeArray<float>(1, Allocator.Persistent);
            for (int f = 0; f < 240; f++)
            {
                sim.Step(Dt, Substeps, floor, fields);
            }
            Assert.IsTrue(sim.TryGetShapeWrench(0, out ApxWrench wrench));
            float weight = sim.TotalMass * 9.81f;
            TestContext.WriteLine($"force {wrench.Force} N, weight {weight:0.0000} N, contact nodes {wrench.ContactNodes:0.0}");
            Assert.AreEqual(-weight, wrench.Force.y, 0.02f * weight, $"force {wrench.Force}, weight {weight:0.000} N");
            Assert.Less(math.length(wrench.Force.xz), 0.02f * weight, $"force {wrench.Force}");
            Assert.Greater(wrench.ContactNodes, 0.0f);
            Assert.Less(math.abs(wrench.ContactCentroid.y), 0.005f, $"contact centroid {wrench.ContactCentroid}");
        }

        [Test]
        public void GrabForceEqualsTheWeightOfALiftedBody()
        {
            // A 10 cm cube resting on the floor is grabbed at the top, lifted 40 mm and held: the grab
            // carries the whole weight, the floor none.
            GameObject floor = InScene(new GameObject("Floor"));
            floor.transform.position = new Vector3(0.0f, -0.1f, 0.0f);
            floor.transform.localScale = new Vector3(0.6f, 0.2f, 0.6f);
            Collider floorCollider = floor.AddComponent<BoxCollider>();
            ApxSoftBody body = CreateBody("Lifted", new Vector3(Size, Size, Size), new Vector3(0.0f, 0.5f * Size + 0.002f, 0.0f), 8);
            body.damping = 3.0f;
            Physics.SyncTransforms();
            body.Initialize();
            for (int f = 0; f < 60; f++)
            {
                body.Simulate(Dt);
                body.UpdateRender();
            }
            Assert.IsTrue(body.BeginGrab(new Ray(new Vector3(0.0f, 1.0f, 0.0f), Vector3.down), 0.03f, out Vector3 point));
            Assert.IsFalse(body.TryGetGrabWrench(out _), "no grab force before the first step of the grab");
            for (int f = 0; f < 300; f++)
            {
                body.MoveGrab(point + Vector3.up * (0.04f * math.saturate(f / 60.0f)));
                body.Simulate(Dt);
                body.UpdateRender();
            }
            AssertFinite(body.Simulation);
            Assert.IsTrue(body.TryGetGrabWrench(out ApxWrench grab));
            Assert.IsFalse(body.TryGetColliderWrench(floorCollider, out _), "the lifted body still touches the floor");
            SoftBodySimulation sim = body.Simulation;
            float weight = sim.TotalMass * 9.81f;
            float held = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                held += sim.KinematicId[i] == SoftBodySimulation.GrabFrame && sim.InverseMass[i] > 0.0f ? 9.81f / sim.InverseMass[i] : 0.0f;
            }
            TestContext.WriteLine($"grab force {grab.Force} N, weight {weight:0.0000} N, held nodes {grab.ContactNodes} " +
                                  $"(weight {held:0.0000} N), torque {grab.Torque} N m, max speed {sim.MaxSpeed:0.0000} m/s");
            Assert.AreEqual(-weight, grab.Force.y, 0.02f * weight, $"grab force {grab.Force}, weight {weight:0.000} N");
            Assert.Less(math.length(grab.Force.xz), 0.02f * weight, $"grab force {grab.Force}");
            body.EndGrab();
            Assert.IsFalse(body.TryGetGrabWrench(out _), "no grab force after the grab ended");
        }

        [Test]
        public void PinForcesCarryTheWeightOfAPinnedBody()
        {
            // A slab held at both ends by pin colliders: the two pin forces add up to its weight and,
            // by symmetry, are equal.
            ApxSoftBody slab = CreateBody("Slab", new Vector3(0.14f, 0.02f, 0.06f), Vector3.zero, 10);
            slab.youngsModulus = 2.0e4f;
            slab.damping = 3.0f;
            slab.pinColliders = new[]
            {
                Trigger(new Vector3(-0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                Trigger(new Vector3(0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
            };
            Physics.SyncTransforms();
            slab.Initialize();
            for (int f = 0; f < 240; f++)
            {
                slab.Simulate(Dt);
                slab.UpdateRender();
            }
            AssertFinite(slab.Simulation);
            Assert.IsTrue(slab.TryGetPinWrench(0, out ApxWrench left));
            Assert.IsTrue(slab.TryGetPinWrench(1, out ApxWrench right));
            Assert.IsFalse(slab.TryGetPinWrench(2, out _));
            float weight = slab.Simulation.TotalMass * 9.81f;
            float3 sum = left.Force + right.Force;
            TestContext.WriteLine($"pin forces {left.Force} + {right.Force} = {sum} N, weight {weight:0.0000} N, " +
                                  $"pinned nodes {left.ContactNodes} + {right.ContactNodes}, sleeping {slab.IsSleeping}");
            Assert.AreEqual(-weight, sum.y, 0.02f * weight, $"pin forces {sum}, weight {weight:0.000} N");
            Assert.Less(math.length(sum.xz), 0.02f * weight, $"pin forces {sum}");
            Assert.AreEqual(left.Force.y, right.Force.y, 0.05f * weight, $"pin forces {left.Force}, {right.Force}");
        }

        [Test]
        public void InstrumentAndPinForcesBalanceTheWeight()
        {
            // A slab held at both ends, pressed off-centre by a ball; at rest the forces the tissue exerts on the
            // ball and the pins add up to its weight, and their torques to the torque of its weight.
            ApxSoftBody slab = CreateBody("Slab", new Vector3(0.14f, 0.02f, 0.06f), Vector3.zero, 10);
            slab.youngsModulus = 2.0e4f;
            slab.damping = 3.0f;
            slab.pinColliders = new[]
            {
                Trigger(new Vector3(-0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                Trigger(new Vector3(0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
            };
            GameObject ball = Primitive(PrimitiveType.Sphere);
            ball.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
            ball.transform.position = new Vector3(0.03f, 0.3f, 0.0f);
            Physics.SyncTransforms();
            slab.Initialize();
            for (int f = 0; f < 60; f++)
            {
                slab.Simulate(Dt);
                slab.UpdateRender();
            }
            // From 1 mm above the top nodes under the ball down to 5 mm below them in 30 frames, then held.
            float top = float.MinValue;
            for (int i = 0; i < slab.Simulation.NodeCount; i++)
            {
                float3 x = slab.Simulation.Positions[i];
                top = math.abs(x.x - 0.03f) < 0.01f && math.abs(x.z) < 0.01f ? math.max(top, x.y) : top;
            }
            for (int f = 0; f < 210; f++)
            {
                float depth = 0.006f * math.saturate(f / 30.0f);
                ball.transform.position = new Vector3(0.03f, top + 0.011f - depth, 0.0f);
                Physics.SyncTransforms();
                slab.Simulate(Dt);
                slab.UpdateRender();
            }
            SoftBodySimulation sim = slab.Simulation;
            AssertFinite(sim);
            Assert.IsTrue(slab.TryGetColliderWrench(ball.GetComponent<Collider>(), out ApxWrench instrument));
            Assert.IsTrue(slab.TryGetPinWrench(0, out ApxWrench left));
            Assert.IsTrue(slab.TryGetPinWrench(1, out ApxWrench right));
            float mass = sim.TotalMass;
            float3 gravity = new float3(0.0f, -9.81f, 0.0f);
            float3 force = instrument.Force + left.Force + right.Force;
            float3 instrumentTorque = instrument.TorqueAbout(float3.zero);
            float3 leftTorque = left.TorqueAbout(float3.zero), rightTorque = right.TorqueAbout(float3.zero);
            float3 torque = instrumentTorque + leftTorque + rightTorque;
            float3 weightTorque = math.cross(CenterOfMass(sim), mass * gravity);
            float scale = mass * 9.81f + math.length(instrument.Force);
            float torqueScale = math.length(instrumentTorque) + math.length(leftTorque) + math.length(rightTorque);
            TestContext.WriteLine($"instrument {instrument.Force} N, pins {left.Force} + {right.Force} N, sum {force} N, " +
                                  $"weight {mass * gravity} N; torques about the origin {instrumentTorque} + {leftTorque} + {rightTorque} = {torque} N m, " +
                                  $"of the weight {weightTorque} N m; " +
                                  $"max speed {sim.MaxSpeed:0.0000} m/s");
            Assert.Greater(instrument.Force.y, 0.1f * mass * 9.81f, $"the ball barely presses ({instrument.Force})");
            Assert.Less(math.length(force - mass * gravity), 0.02f * scale, $"force sum {force}, weight {mass * gravity}");
            Assert.Less(math.length(torque - weightTorque), 0.02f * torqueScale, $"torque sum {torque}, weight {weightTorque}");
        }

        [Test]
        public void IndenterForceGrowsWithDepthAndMatchesHertz()
        {
            // A frictionless 2 cm ball pressed into an 8 x 4 x 8 cm block with a clamped base (no gravity), 2.5 mm
            // cells. Hertz: F = 4/3 E / (1 - nu^2) sqrt(R) d^1.5 for a half-space; the block is 6 contact radii deep
            // at 2 mm. One solver iteration per substep makes the lattice softer than E unless mu h^2 / (rho dx^2)
            // is small (DESIGN 9.2): here 0.011, with E = 3 kPa and 64 substeps.
            var parameters = SoftBodyParameters.Default;
            parameters.YoungsModulus = 3.0e3f;
            parameters.PoissonRatio = 0.45f;
            parameters.Damping = 20.0f;
            const float radius = 0.02f;
            float[] depths = { 0.5e-3f, 1.0e-3f, 2.0e-3f, 3.0e-3f };
            float[] forces = Indent(parameters, new float3(0.08f, 0.04f, 0.08f), 32, 64, radius, depths, out float[] clamp);
            for (int k = 0; k < depths.Length; k++)
            {
                float hertz = Hertz(parameters.YoungsModulus, parameters.PoissonRatio, radius, depths[k]);
                TestContext.WriteLine($"depth {depths[k] * 1000:0.0} mm: force {forces[k] * 1000:0.000} mN, Hertz {hertz * 1000:0.000} mN " +
                                      $"(ratio {forces[k] / hertz:0.000}), clamp {clamp[k] * 1000:0.000} mN");
            }
            for (int k = 0; k < depths.Length; k++)
            {
                if (k > 0)
                {
                    Assert.Greater(forces[k], forces[k - 1], $"force at {depths[k] * 1000:0.0} mm");
                }
                if (depths[k] <= 2.0e-3f)
                {
                    float hertz = Hertz(parameters.YoungsModulus, parameters.PoissonRatio, radius, depths[k]);
                    Assert.AreEqual(hertz, forces[k], 0.3f * hertz, $"force at {depths[k] * 1000:0.0} mm");
                }
            }
        }

        [Test]
        public void SameInputsGiveBitIdenticalForces()
        {
            // Floor, a pressing ball, a pinned side and a moving grab: every wrench of two runs is equal bit for bit.
            List<uint> Run()
            {
                SoftBodySimulation sim = CreateBox(new float3(Size), SoftBodyParameters.Default, 12, 0.002f + LatticeOffset(12));
                NativeArray<ColliderShape> shapes = Track(new NativeArray<ColliderShape>(2, Allocator.Persistent));
                NativeArray<float> fields = Track(new NativeArray<float>(1, Allocator.Persistent));
                shapes[0] = FloorShape();
                var side = new List<int>();
                float minX = float.MaxValue;
                for (int i = 0; i < sim.NodeCount; i++)
                {
                    minX = math.min(minX, sim.Positions[i].x);
                }
                for (int i = 0; i < sim.NodeCount; i++)
                {
                    if (sim.Positions[i].x < minX + 1e-4f)
                    {
                        side.Add(i);
                    }
                }
                sim.Attach(side, 1, float4x4.Translate(new float3(minX, 0.05f, 0.0f)));
                int node = sim.PickNode(new float3(0.02f, 1.0f, 0.0f), new float3(0.0f, -1.0f, 0.0f), sim.CellSize);
                float3 grab = sim.Positions[node];
                sim.Attach(sim.ConnectedNodes(node, 0.02f), SoftBodySimulation.GrabFrame, float4x4.Translate(grab));
                var bits = new List<uint>();
                float3 ball = new float3(-0.02f, 0.13f, 0.0f);
                for (int f = 0; f < 60; f++)
                {
                    float3 target = grab + new float3(0.0f, 0.02f, 0.01f) * math.saturate(f / 30.0f);
                    sim.FramesStart[SoftBodySimulation.GrabFrame] = sim.FramesEnd[SoftBodySimulation.GrabFrame];
                    sim.FramesEnd[SoftBodySimulation.GrabFrame] = float4x4.Translate(target);
                    float3 next = new float3(-0.02f, 0.13f - 0.0003f * f, 0.0f);
                    ColliderShape sphere = ColliderShape.Sphere(next, 0.02f, 0.6f);
                    sphere.A0 = ball;
                    ball = next;
                    shapes[1] = sphere;
                    sim.Step(Dt, Substeps, shapes, fields);
                    for (int s = 0; s < 2; s++)
                    {
                        if (sim.TryGetShapeWrench(s, out ApxWrench w))
                        {
                            AddBits(bits, w);
                        }
                    }
                    for (int frame = 0; frame < 2; frame++)
                    {
                        Assert.IsTrue(sim.TryGetFrameWrench(frame, out ApxWrench w), $"frame {frame}");
                        AddBits(bits, w);
                    }
                }
                Assert.IsTrue(sim.TryGetShapeWrench(1, out _), "the ball never touched the body");
                return bits;
            }

            List<uint> first = Run();
            List<uint> second = Run();
            Assert.AreEqual(first.Count, second.Count);
            for (int i = 0; i < first.Count; i++)
            {
                Assert.AreEqual(first[i], second[i], $"value {i} differs");
            }
        }

        private static void AddBits(List<uint> bits, ApxWrench w)
        {
            foreach (float3 v in new[] { w.Force, w.Torque, w.Origin, w.ContactCentroid })
            {
                bits.Add(math.asuint(v.x));
                bits.Add(math.asuint(v.y));
                bits.Add(math.asuint(v.z));
            }
            bits.Add(math.asuint(w.ContactNodes));
        }

        /// <summary>
        /// Presses a frictionless sphere into the middle of the top face of a block with a clamped base (frame 1,
        /// no gravity), to each of <paramref name="depths"/> in turn below the top lattice nodes, 60 steps each.
        /// Returns the force on the sphere at each depth and the force on the clamp in <paramref name="clampForces"/>
        /// (y components).
        /// </summary>
        private float[] Indent(SoftBodyParameters parameters, float3 size, int resolution, int substeps, float radius,
            float[] depths, out float[] clampForces)
        {
            parameters.Gravity = float3.zero;
            SoftBodySimulation sim = CreateBox(size, parameters, resolution, 0.0f);
            float bottom = float.MaxValue, top = float.MinValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                bottom = math.min(bottom, sim.Positions[i].y);
                top = math.max(top, sim.Positions[i].y);
            }
            var clamped = new List<int>();
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.Positions[i].y < bottom + 1e-5f)
                {
                    clamped.Add(i);
                }
            }
            sim.Attach(clamped, 1, float4x4.identity);
            NativeArray<ColliderShape> shapes = Track(new NativeArray<ColliderShape>(1, Allocator.Persistent));
            NativeArray<float> fields = Track(new NativeArray<float>(1, Allocator.Persistent));
            var forces = new float[depths.Length];
            clampForces = new float[depths.Length];
            float3 previous = new float3(0.0f, top + radius, 0.0f);
            float depth = 0.0f;
            for (int k = 0; k < depths.Length; k++)
            {
                for (int f = 0; f < 60; f++)
                {
                    float3 center = new float3(0.0f, top + radius - math.lerp(depth, depths[k], math.saturate((f + 1) / 20.0f)), 0.0f);
                    ColliderShape sphere = ColliderShape.Sphere(center, radius, 0.0f);
                    sphere.A0 = previous;
                    previous = center;
                    shapes[0] = sphere;
                    sim.Step(Dt, substeps, shapes, fields);
                }
                depth = depths[k];
                AssertFinite(sim);
                Assert.IsTrue(sim.TryGetShapeWrench(0, out ApxWrench indenter), $"no contact at {depth * 1000:0.0} mm");
                Assert.IsTrue(sim.TryGetFrameWrench(1, out ApxWrench clamp));
                forces[k] = indenter.Force.y;
                clampForces[k] = clamp.Force.y;
            }
            return forces;
        }

        /// <summary>Hertz contact of a rigid sphere on an elastic half-space: F = 4/3 E / (1 - nu^2) sqrt(R) d^1.5.</summary>
        private static float Hertz(float youngsModulus, float poissonRatio, float radius, float depth) =>
            4.0f / 3.0f * youngsModulus / (1.0f - poissonRatio * poissonRatio) * math.sqrt(radius) * math.pow(depth, 1.5f);

        [Test]
        public void RigidBodyRestsOnPinnedTissueAndBendsIt()
        {
            // Two-way coupling: the tissue holds the ball (PhysX alone lets it fall through), carries
            // its weight and sags more than under its own weight.
            const float mass = 0.1f;
            float Run(bool twoWay, out float sag, out ApxWrench wrench)
            {
                SimulationMode mode = Physics.simulationMode;
                Physics.simulationMode = SimulationMode.Script;
                try
                {
                    ApxSoftBody slab = CreateBody("Slab", new Vector3(0.14f, 0.02f, 0.06f), Vector3.zero, 10);
                    slab.youngsModulus = 2.0e4f;
                    slab.pinColliders = new[]
                    {
                        Trigger(new Vector3(-0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                        Trigger(new Vector3(0.07f, 0.0f, 0.0f), new Vector3(0.02f, 0.1f, 0.1f)),
                    };
                    slab.twoWayCoupling = twoWay;
                    Physics.SyncTransforms();
                    slab.Initialize();
                    float restCenter = CenterHeight(slab.Simulation);
                    // A 2 cm ball 2 mm above the top lattice nodes.
                    GameObject ball = Primitive(PrimitiveType.Sphere);
                    ball.transform.position = new Vector3(0.0f, slab.Simulation.BoundsMax.y + 0.022f, 0.0f);
                    ball.transform.localScale = new Vector3(0.04f, 0.04f, 0.04f);
                    var rigidbody = ball.AddComponent<Rigidbody>();
                    rigidbody.mass = mass;
                    Physics.SyncTransforms();
                    for (int f = 0; f < 300; f++)
                    {
                        slab.Simulate(Dt);
                        slab.UpdateRender();
                        _scene.GetPhysicsScene().Simulate(Dt);
                    }
                    AssertFinite(slab.Simulation);
                    sag = restCenter - CenterHeight(slab.Simulation);
                    slab.TryGetColliderWrench(ball.GetComponent<Collider>(), out wrench);
                    float y = ball.transform.position.y;
                    Release(slab);
                    return y;
                }
                finally
                {
                    Physics.simulationMode = mode;
                }
            }

            float coupled = Run(true, out float sag, out ApxWrench wrench);
            float uncoupled = Run(false, out float ownWeightSag, out _);
            TestContext.WriteLine($"ball y {coupled * 1000:0.0} mm, force on the ball {wrench.Force} N, sag {sag * 1000:0.00} mm " +
                                  $"(own weight {ownWeightSag * 1000:0.00} mm), uncoupled ball y {uncoupled * 1000:0} mm");
            Assert.Greater(coupled, 0.0f, "the ball fell through the tissue");
            Assert.AreEqual(mass * 9.81f, wrench.Force.y, 0.05f * mass * 9.81f, $"force on the ball {wrench.Force}");
            Assert.Greater(sag - ownWeightSag, 1e-3f,
                $"sag {sag * 1000:0.00} mm with the ball, {ownWeightSag * 1000:0.00} mm without");
            Assert.Less(uncoupled, -0.05f, "without coupling the ball falls through");
        }

        [Test]
        public void StackedBodiesRestOnEachOther()
        {
            float Run(bool contact, out float3[] positions)
            {
                GameObject floor = Primitive(PrimitiveType.Cube);
                floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
                floor.transform.localScale = new Vector3(0.5f, 0.1f, 0.5f);
                // The bottom face touches the floor (the lattice reaches half a cell below the surface);
                // the top body starts 4 mm above the bottom body.
                ApxSoftBody bottom = CreateBody("Bottom", new Vector3(0.1f, 0.04f, 0.1f), new Vector3(0.0f, 0.02f, 0.0f), 10);
                ApxSoftBody top = CreateBody("Top", new Vector3(0.06f, 0.04f, 0.06f), new Vector3(0.005f, 0.064f, 0.0f), 8);
                bottom.collisionMargin = -LatticeOffset(10, 0.1f);
                top.collisionMargin = -LatticeOffset(8, 0.06f);
                bottom.collideWithBodies = contact;
                top.collideWithBodies = contact;
                Physics.SyncTransforms();
                bottom.Initialize();
                top.Initialize();
                for (int f = 0; f < 150; f++)
                {
                    bottom.Simulate(Dt);
                    top.Simulate(Dt);
                    bottom.UpdateRender();
                    top.UpdateRender();
                }
                AssertFinite(bottom.Simulation);
                AssertFinite(top.Simulation);
                positions = top.Simulation.Positions.ToArray();
                float center = MeanHeight(top.Simulation);
                Release(top);
                Release(bottom);
                Object.DestroyImmediate(floor);
                return center;
            }

            // The bottom body's top face is at 40 mm: the top body rests with its centre about 20 mm
            // higher, less the compression of both. Without body contact it sinks to the floor (centre at 20 mm).
            float stacked = Run(true, out float3[] first);
            Assert.Greater(stacked, 0.055f, $"top body centre {stacked * 1000:0.0} mm");
            Assert.Less(stacked, 0.0625f, $"top body centre {stacked * 1000:0.0} mm");
            Run(true, out float3[] second);
            for (int i = 0; i < first.Length; i++)
            {
                Assert.IsTrue(math.all(first[i] == second[i]), $"node {i} differs between identical runs");
            }
            float fallen = Run(false, out _);
            TestContext.WriteLine($"top body centre: stacked {stacked * 1000:0.00} mm, without contact {fallen * 1000:0.00} mm");
            Assert.Less(fallen, 0.03f, $"without body contact the top body sinks through (centre {fallen * 1000:0.0} mm)");
        }

        [Test]
        public void PinnedBodyPushesAnotherWithoutPassingThrough()
        {
            // ECO-1 acceptance: the pushed body slides on a frictionless floor. With friction see
            // PinnedBodyPushesAnotherAlongAFloorWithFriction (BUG-8).
            Squeeze contact = RunSqueeze(true, 0.0f, false, 36, 0);
            Squeeze unopposed = RunSqueeze(false, 0.0f, false, 36, 0);
            float h = contact.Cell;
            TestContext.WriteLine($"pushed body moved {contact.Moved * 1000:0.00} mm, deepest node {contact.Deepest * 1000:0.00} mm " +
                                  $"(cell {h * 1000:0.0} mm); without contact moved {unopposed.Moved * 1000:0.00} mm, " +
                                  $"deepest node {unopposed.Deepest * 1000:0.0} mm");
            Assert.LessOrEqual(contact.Deepest, h, $"a node went {contact.Deepest * 1000:0.00} mm into the other body (cell {h * 1000:0.0} mm)");
            Assert.Greater(contact.Moved, h, $"the pushed body moved {contact.Moved * 1000:0.00} mm");
            Assert.Greater(unopposed.Deepest, h, $"without body contact the pusher enters the other body ({unopposed.Deepest * 1000:0.0} mm)");
            Assert.Less(unopposed.Moved, h, $"without body contact the other body stays ({unopposed.Moved * 1000:0.00} mm)");
        }

        [Test]
        [Ignore("TODO BUG-8: on a floor with friction 0.6 the pushed body hardly slides, tips forward, and the pusher goes more than a cell into it")]
        public void PinnedBodyPushesAnotherAlongAFloorWithFriction()
        {
            Squeeze push = RunSqueeze(true, 0.6f, false, 36, 0);
            float h = push.Cell;
            TestContext.WriteLine($"pushed body moved {push.Moved * 1000:0.00} mm, deepest node {push.Deepest * 1000:0.00} mm " +
                                  $"(cell {h * 1000:0.0} mm), back face tilt {push.Tilt * 1000:0.0} mm");
            Assert.LessOrEqual(push.Deepest, h, $"a node went {push.Deepest * 1000:0.00} mm into the other body (cell {h * 1000:0.0} mm)");
            Assert.Greater(push.Moved, h, $"the pushed body moved {push.Moved * 1000:0.00} mm");
        }

        [Test]
        public void SqueezedBodiesKeepTheirContactUnderASustainedLoad()
        {
            // The pushed body's far end is pinned; the pusher presses 5 mm into it and holds for a second.
            // Before the substeps of bodies in contact were synchronised (BUG-8), the overlap grew during
            // the hold (deepest node 4.5 -> 6.2 mm) and the contact nodes moved at up to 1.4 m/s.
            Squeeze hold = RunSqueeze(true, 0.6f, true, 6, 60);
            float h = hold.Cell;
            TestContext.WriteLine($"deepest node {hold.Deepest * 1000:0.00} mm (cell {h * 1000:0.0} mm), growth during the hold " +
                                  $"{hold.HoldGrowth * 1000:0.00} mm, fastest node in the last 30 frames {hold.MaxSpeed:0.000} m/s");
            Assert.LessOrEqual(hold.Deepest, h, $"a node went {hold.Deepest * 1000:0.00} mm into the other body");
            Assert.Less(hold.HoldGrowth, 2e-4f, $"the overlap grew by {hold.HoldGrowth * 1000:0.00} mm while holding");
            Assert.Less(hold.MaxSpeed, 0.01f, $"nodes still move at {hold.MaxSpeed:0.000} m/s");
        }

        private struct Squeeze
        {
            /// <summary>Displacement of the pushed body's centre of mass along the push.</summary>
            public float Moved;
            /// <summary>Deepest node of either body inside the other (<see cref="Penetration"/>) from the first push on.</summary>
            public float Deepest;
            /// <summary>Change of the deepest node from the end of the push to the end of the hold.</summary>
            public float HoldGrowth;
            /// <summary>Fastest node of either body in the last 30 frames.</summary>
            public float MaxSpeed;
            /// <summary>
            /// How much further the top of the pushed body's back face moved along the push than its bottom
            /// (0 when it slides, large when it tips over).
            /// </summary>
            public float Tilt;
            public float Cell;
        }

        /// <summary>
        /// A 4 cm cube (resolution 8) with its rear half pinned, its centre 35 mm above the floor, is pushed at 0.1 m/s
        /// for <paramref name="pushFrames"/> into a 6 cm cube (resolution 12) resting on the floor, then holds for
        /// <paramref name="holdFrames"/>. Both lattices have 5 mm cells on the same grid; the facing faces start one
        /// cell apart, so the outer nodes of both faces start at x = 0. <paramref name="pinFarEnd"/> pins the last
        /// 6 mm of the pushed body.
        /// </summary>
        private Squeeze RunSqueeze(bool contact, float friction, bool pinFarEnd, int pushFrames, int holdFrames)
        {
            const float pusherHeight = 0.035f, pushedLength = 0.06f;
            GameObject floor = Primitive(PrimitiveType.Cube);
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            floor.transform.localScale = new Vector3(0.5f, 0.1f, 0.5f);
            ApxSoftBody pusher = CreateBody("Pusher", new Vector3(0.04f, 0.04f, 0.04f), new Vector3(-0.0225f, pusherHeight, 0.0f), 8);
            int resolution = Mathf.RoundToInt(pushedLength / 0.005f);
            ApxSoftBody pushed = CreateBody("Pushed", new Vector3(pushedLength, 0.06f, 0.06f),
                new Vector3(0.0025f + 0.5f * pushedLength, 0.03f, 0.0f), resolution);
            Collider pin = Trigger(new Vector3(-0.0325f, pusherHeight, 0.0f), new Vector3(0.025f, 0.06f, 0.06f));
            pusher.pinColliders = new[] { pin };
            if (pinFarEnd)
            {
                pushed.pinColliders = new[] { Trigger(new Vector3(0.0025f + pushedLength, 0.03f, 0.0f), new Vector3(0.012f, 0.08f, 0.08f)) };
            }
            pushed.collisionMargin = -LatticeOffset(resolution, pushedLength);
            pushed.friction = friction;
            pusher.collideWithBodies = contact;
            pushed.collideWithBodies = contact;
            Physics.SyncTransforms();
            pusher.Initialize();
            pushed.Initialize();
            var result = new Squeeze { Cell = pushed.Simulation.CellSize, Deepest = float.MinValue };
            float start = 0.0f, pushedDepth = 0.0f;
            int frames = 30 + pushFrames + holdFrames;
            for (int f = 0; f < frames; f++)
            {
                if (f == 30)
                {
                    start = CenterOfMass(pushed.Simulation).x;
                }
                if (f >= 30 && f < 30 + pushFrames)
                {
                    pin.transform.position += new Vector3(0.1f * Dt, 0.0f, 0.0f);
                }
                pusher.Simulate(Dt);
                pushed.Simulate(Dt);
                pusher.UpdateRender();
                pushed.UpdateRender();
                if (f >= 30)
                {
                    float depth = Penetration(pusher.Simulation, pushed.Simulation);
                    result.Deepest = math.max(result.Deepest, depth);
                    if (f == 30 + pushFrames - 1)
                    {
                        pushedDepth = depth;
                    }
                    result.HoldGrowth = depth - pushedDepth;
                }
                if (f >= frames - 30)
                {
                    result.MaxSpeed = math.max(result.MaxSpeed, math.max(pusher.Simulation.MaxSpeed, pushed.Simulation.MaxSpeed));
                }
            }
            AssertFinite(pusher.Simulation);
            AssertFinite(pushed.Simulation);
            result.Moved = CenterOfMass(pushed.Simulation).x - start;
            result.Tilt = BackFaceTilt(pushed.Simulation);
            Release(pusher);
            Release(pushed);
            Object.DestroyImmediate(floor);
            return result;
        }

        [Test]
        public void StickyInstrumentPullsTheSurfaceAfterIt()
        {
            float Lift(float stickiness)
            {
                SoftBodySimulation sim = CreateBox(new float3(Size), SoftBodyParameters.Default, 8, LatticeOffset(8));
                NativeArray<ColliderShape> shapes = Track(new NativeArray<ColliderShape>(2, Allocator.Persistent));
                NativeArray<float> fields = Track(new NativeArray<float>(1, Allocator.Persistent));
                shapes[0] = FloorShape();
                StepWith(sim, shapes, fields, 60, 1.0f);
                float top = TopCenter(sim);
                ColliderShape Plate(float y)
                {
                    ColliderShape plate = ColliderShape.Box(new float3(0.0f, y + 0.01f, 0.0f), quaternion.identity,
                        new float3(0.03f, 0.01f, 0.03f), 0.6f);
                    plate.Stickiness = stickiness;
                    plate.StickDistance = 0.003f;
                    return plate;
                }
                ColliderShape previous = Plate(top - 5e-4f);
                for (int f = 0; f < 60; f++)
                {
                    // Pressed 0.5 mm into the surface for half a second, then raised by 1.5 mm.
                    float y = f < 30 ? top - 5e-4f : top - 5e-4f + 1.5e-3f * math.saturate((f - 30) / 20.0f);
                    ColliderShape plate = Plate(y);
                    plate.A0 = previous.A1;
                    shapes[1] = plate;
                    previous = plate;
                    sim.Step(Dt, Substeps, shapes, fields);
                }
                AssertFinite(sim);
                return TopCenter(sim) - top;
            }

            float plain = Lift(0.0f);
            float sticky = Lift(1.0f);
            TestContext.WriteLine($"surface lift: plain {plain * 1000:0.00} mm, sticky {sticky * 1000:0.00} mm");
            Assert.Less(plain, 3e-4f, $"without adhesion the surface stays below the plate ({plain * 1000:0.00} mm)");
            Assert.Greater(sticky, 6e-4f, $"the sticky plate did not pull the surface up ({sticky * 1000:0.00} mm)");
        }

        [Test]
        public void PressedPlasticBodyKeepsADent()
        {
            float Dent(bool plastic)
            {
                var parameters = SoftBodyParameters.Default;
                parameters.Damping = 3.0f;
                if (plastic)
                {
                    parameters.PlasticYield = 0.02f;
                    parameters.PlasticCreep = 20.0f;
                    parameters.MaxPlasticDeformation = 0.5f;
                }
                SoftBodySimulation sim = CreateBox(new float3(Size), parameters, 10, LatticeOffset(10));
                NativeArray<ColliderShape> shapes = Track(new NativeArray<ColliderShape>(2, Allocator.Persistent));
                NativeArray<float> fields = Track(new NativeArray<float>(1, Allocator.Persistent));
                shapes[0] = FloorShape();
                StepWith(sim, shapes, fields, 30, 1.0f);
                float restTop = TopCenter(sim);
                float top = restTop;
                // A 2 cm ball pressed 15 mm into the top face, held, then lifted away.
                const float radius = 0.02f;
                float3 previous = new float3(0.0f, top + radius + 0.002f, 0.0f);
                for (int f = 0; f < 150; f++)
                {
                    float depth = f < 40 ? 0.017f * f / 40.0f : f < 80 ? 0.017f : math.max(0.017f - 0.05f * (f - 80) / 10.0f, -0.05f);
                    float3 center = new float3(0.0f, top + radius + 0.002f - depth, 0.0f);
                    ColliderShape ball = ColliderShape.Sphere(center, radius, 0.6f);
                    ball.A0 = previous;
                    previous = center;
                    shapes[1] = ball;
                    sim.Step(Dt, Substeps, shapes, fields);
                }
                AssertFinite(sim);
                return restTop - TopCenter(sim);
            }

            float elastic = Dent(false);
            float plastic = Dent(true);
            TestContext.WriteLine($"dent: elastic {elastic * 1000:0.00} mm, plastic {plastic * 1000:0.00} mm");
            Assert.Less(elastic, 1e-3f, $"an elastic body recovers ({elastic * 1000:0.00} mm)");
            Assert.Greater(plastic, 3e-3f, $"a plastic body keeps the dent ({plastic * 1000:0.00} mm)");
        }

        private static void StepWith(SoftBodySimulation sim, NativeArray<ColliderShape> shapes, NativeArray<float> fields,
            int frames, float parkedBallY)
        {
            shapes[1] = ColliderShape.Sphere(new float3(0.0f, parkedBallY, 0.0f), 0.001f, 0.0f);
            for (int f = 0; f < frames; f++)
            {
                sim.Step(Dt, Substeps, shapes, fields);
            }
        }

        [Test]
        public void RestoredStateRepeatsTheSameSteps()
        {
            SoftBodySimulation sim = CreateBox(new float3(Size), SoftBodyParameters.Default, 8, 0.02f);
            using var floor = Floor();
            using var fields = new NativeArray<float>(1, Allocator.Persistent);
            void Frames(int count)
            {
                for (int f = 0; f < count; f++)
                {
                    sim.Step(Dt, Substeps, floor, fields);
                }
            }

            Frames(20);
            sim.Cut(new float3(0.0f, 0.5f, -0.2f), new float3(0.0f, -0.5f, -0.2f), new float3(0.0f, 0.5f, 0.0f),
                new float3(0.0f, -0.5f, 0.0f));
            Frames(5);
            SoftBodyState state = sim.CaptureState();
            int nodes = sim.NodeCount;
            Frames(20);
            float3[] expected = sim.Positions.ToArray();

            // Change the topology, then go back.
            sim.Cut(new float3(-0.2f, 0.5f, 0.01f), new float3(-0.2f, -0.5f, 0.01f), new float3(0.2f, 0.5f, 0.01f),
                new float3(0.2f, -0.5f, 0.01f));
            Assert.Greater(sim.CountPieces(), 1);
            Frames(5);
            sim.RestoreState(state);
            Assert.AreEqual(nodes, sim.NodeCount);
            Assert.AreEqual(1, sim.CountPieces());
            Frames(20);
            float3[] actual = sim.Positions.ToArray();
            Assert.AreEqual(expected.Length, actual.Length);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.IsTrue(math.all(expected[i] == actual[i]), $"node {i} differs after restoring");
            }
        }

        [Test]
        public void RestoredBodyRendersItsTopology()
        {
            ApxSoftBody body = CreateBody("Snapshot", new Vector3(0.1f, 0.04f, 0.08f), Vector3.zero, 8);
            body.useGravity = false;
            body.Initialize();
            ApxSoftBodyState state = body.CaptureState();
            int triangles = body.Surface.SkinTriangleCount;
            body.Cut(new Vector3(0.0f, 0.5f, -0.2f), new Vector3(0.0f, -0.5f, -0.2f), new Vector3(0.0f, 0.5f, 0.2f),
                new Vector3(0.0f, -0.5f, 0.2f));
            body.Simulate(Dt);
            body.UpdateRender();
            Assert.Greater(body.Surface.CutTriangleCount, 0);
            body.RestoreState(state);
            body.UpdateRender();
            Assert.AreEqual(0, body.Surface.CutTriangleCount);
            Assert.AreEqual(triangles, body.Surface.SkinTriangleCount);
            Assert.AreEqual(1, body.Simulation.CountPieces());
        }

        [Test]
        public void BlueprintGivesTheSameSimulationWithoutVoxelizing()
        {
            Mesh mesh = Track(MeshUtil.CreateBox(new Vector3(0.1f, 0.04f, 0.08f), 4));
            float3[] Run(ApxSoftBodyBlueprint blueprint, out ApxSoftBody.InitializeTimings timings)
            {
                ApxSoftBody body = CreateBody("Blueprint", mesh, new Vector3(0.0f, 0.05f, 0.0f), 12);
                body.blueprint = blueprint;
                body.Initialize();
                timings = body.LastInitializeTimings;
                for (int f = 0; f < 20; f++)
                {
                    body.Simulate(Dt);
                    body.UpdateRender();
                }
                float3[] positions = body.Simulation.Positions.ToArray();
                Release(body);
                return positions;
            }

            float3[] reference = Run(null, out _);
            ApxSoftBody source = CreateBody("Blueprint Source", mesh, new Vector3(0.0f, 0.05f, 0.0f), 12);
            ApxSoftBodyBlueprint baked = Track(ScriptableObject.CreateInstance<ApxSoftBodyBlueprint>());
            Assert.IsTrue(source.BakeBlueprint(baked));
            Assert.IsTrue(baked.Matches(mesh, 12, Vector3.one));
            _bodies.Remove(source);
            Object.DestroyImmediate(source.gameObject);

            float3[] fromBlueprint = Run(baked, out ApxSoftBody.InitializeTimings timings);
            Assert.AreEqual(0.0f, timings.Voxelize, "the blueprint path must not voxelize");
            Assert.AreEqual(reference.Length, fromBlueprint.Length);
            for (int i = 0; i < reference.Length; i++)
            {
                Assert.IsTrue(math.all(reference[i] == fromBlueprint[i]), $"node {i} differs");
            }
        }

        private NativeArray<T> Track<T>(NativeArray<T> array) where T : struct
        {
            _natives.Add(array);
            return array;
        }

        private T Track<T>(T o) where T : Object
        {
            _objects.Add(o);
            return o;
        }

        /// <summary>Moves a new object into the test's preview scene.</summary>
        private GameObject InScene(GameObject go)
        {
            if (!_scene.IsValid())
            {
                _scene = EditorSceneManager.NewPreviewScene();
            }
            SceneManager.MoveGameObjectToScene(go, _scene);
            return Track(go);
        }

        private GameObject Primitive(PrimitiveType type) => InScene(GameObject.CreatePrimitive(type));

        private ApxSoftBody CreateBody(string name, Vector3 size, Vector3 position, int resolution) =>
            CreateBody(name, Track(MeshUtil.CreateBox(size, 4)), position, resolution);

        private ApxSoftBody CreateBody(string name, Mesh mesh, Vector3 position, int resolution)
        {
            GameObject go = InScene(new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer)));
            go.transform.position = position;
            go.GetComponent<MeshFilter>().sharedMesh = mesh;
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = resolution;
            _bodies.Add(body);
            return body;
        }

        /// <summary>
        /// Distance from the faces of an axis-aligned box to its outer lattice nodes: the grid starts
        /// half a cell outside the mesh, so the nodes stand half a cell outside the faces.
        /// </summary>
        private static float LatticeOffset(int resolution, float longest = Size) => 0.5f * longest / resolution;

        private void Release(ApxSoftBody body)
        {
            body.Release();
            _bodies.Remove(body);
        }

        private Collider Trigger(Vector3 position, Vector3 size)
        {
            GameObject go = InScene(new GameObject("Pin"));
            go.transform.position = position;
            var box = go.AddComponent<BoxCollider>();
            box.size = size;
            box.isTrigger = true;
            return box;
        }

        private static ColliderShape FloorShape() =>
            ColliderShape.Box(new float3(0.0f, -0.5f, 0.0f), quaternion.identity, new float3(1.0f, 0.5f, 1.0f), 0.6f);

        private static NativeArray<ColliderShape> Floor() =>
            new NativeArray<ColliderShape>(new[] { FloorShape() }, Allocator.Persistent);

        /// <param name="height">Gap between the floor (y = 0) and the bottom of the box.</param>
        private SoftBodySimulation CreateBox(float3 size, SoftBodyParameters parameters, int resolution, float height)
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
            VoxelGrid grid = Voxelizer.Voxelize(p.ToArray(), tris.ToArray(), cell);
            var sim = new SoftBodySimulation(grid, parameters,
                float4x4.Translate(new float3(0.0f, height + 0.5f * size.y, 0.0f)));
            _sims.Add(sim);
            return sim;
        }

        /// <summary>Mean height of the nodes in the middle of the top face.</summary>
        private static float TopCenter(SoftBodySimulation sim)
        {
            float sum = 0.0f;
            int count = 0;
            float top = float.MinValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                top = math.max(top, sim.RestPositions[i].y);
            }
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 r = sim.RestPositions[i];
                if (math.abs(r.y - top) < 1e-4f && math.abs(r.x) < 0.015f && math.abs(r.z) < 0.015f)
                {
                    sum += sim.Positions[i].y;
                    count++;
                }
            }
            Assert.Greater(count, 0);
            return sum / count;
        }

        private static float CenterHeight(SoftBodySimulation sim)
        {
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 r = sim.RestPositions[i];
                if (math.abs(r.x) < 0.012f && math.abs(r.z) < 0.012f)
                {
                    sum += sim.Positions[i].y;
                    count++;
                }
            }
            return sum / math.max(count, 1);
        }

        private static float MeanHeight(SoftBodySimulation sim)
        {
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.NodeReal[i] != 0)
                {
                    sum += sim.Positions[i].y;
                    count++;
                }
            }
            return sum / count;
        }

        /// <summary>Displacement along x of the top row of the -x face less that of its bottom row.</summary>
        private static float BackFaceTilt(SoftBodySimulation sim)
        {
            float back = float.MaxValue, bottom = float.MaxValue, top = float.MinValue;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 r = sim.RestPositions[i];
                back = math.min(back, r.x);
                bottom = math.min(bottom, r.y);
                top = math.max(top, r.y);
            }
            float topMove = 0.0f, bottomMove = 0.0f;
            int topCount = 0, bottomCount = 0;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                float3 r = sim.RestPositions[i];
                if (r.x > back + 1e-5f)
                {
                    continue;
                }
                float move = sim.Positions[i].x - math.transform(sim.RestToWorld, r).x;
                if (r.y > top - 1e-5f)
                {
                    topMove += move;
                    topCount++;
                }
                else if (r.y < bottom + 1e-5f)
                {
                    bottomMove += move;
                    bottomCount++;
                }
            }
            return topMove / math.max(topCount, 1) - bottomMove / math.max(bottomCount, 1);
        }

        private static float3 CenterOfMass(SoftBodySimulation sim)
        {
            float3 sum = float3.zero;
            float mass = 0.0f;
            for (int i = 0; i < sim.NodeCount; i++)
            {
                if (sim.NodeReal[i] != 0 && sim.InverseMass[i] > 0.0f)
                {
                    float m = 1.0f / sim.InverseMass[i];
                    sum += m * sim.Positions[i];
                    mass += m;
                }
            }
            return sum / mass;
        }

        /// <summary>Corners of the six tetrahedra of a cell (the Kuhn split the body contact uses).</summary>
        private static readonly int4[] CellTets =
        {
            new int4(0, 1, 3, 7), new int4(0, 1, 7, 5), new int4(0, 2, 7, 3),
            new int4(0, 2, 6, 7), new int4(0, 4, 5, 7), new int4(0, 4, 7, 6),
        };

        /// <summary>Deepest real node of either body inside the other (<see cref="DepthInside"/>).</summary>
        private static float Penetration(SoftBodySimulation a, SoftBodySimulation b) =>
            math.max(DepthInside(a, b), DepthInside(b, a));

        /// <summary>
        /// Deepest real node of <paramref name="body"/> inside <paramref name="other"/>, by the distance the body
        /// contact uses (DESIGN 9.4): the other body's rest SDF interpolated linearly on the deformed tetrahedron
        /// that contains the node, over the length of its world gradient. Where the gradient is shorter than 0.5
        /// (on the medial axis of the SDF) the rest SDF itself is used.
        /// float.MinValue when no node lies in a tetrahedron of the other body.
        /// </summary>
        private static float DepthInside(SoftBodySimulation body, SoftBodySimulation other)
        {
            int copies = other.Copies.Length;
            var lo = new float3[copies];
            var hi = new float3[copies];
            for (int q = 0; q < copies; q++)
            {
                lo[q] = hi[q] = other.Positions[other.CopyNodes[Lattice.CornerCount * q]];
                for (int k = 1; k < Lattice.CornerCount; k++)
                {
                    float3 p = other.Positions[other.CopyNodes[Lattice.CornerCount * q + k]];
                    lo[q] = math.min(lo[q], p);
                    hi[q] = math.max(hi[q], p);
                }
            }
            float deepest = float.MinValue;
            for (int i = 0; i < body.NodeCount; i++)
            {
                if (body.NodeReal[i] == 0)
                {
                    continue;
                }
                float3 x = body.Positions[i];
                for (int q = 0; q < copies; q++)
                {
                    if (math.any(x < lo[q]) || math.any(x > hi[q]))
                    {
                        continue;
                    }
                    int mask = other.Copies[q].Mask;
                    foreach (int4 t in CellTets)
                    {
                        if (((mask >> t.x) & (mask >> t.y) & (mask >> t.z) & (mask >> t.w) & 1) == 0)
                        {
                            continue;
                        }
                        int b = Lattice.CornerCount * q;
                        int4 n = new int4(other.CopyNodes[b + t.x], other.CopyNodes[b + t.y], other.CopyNodes[b + t.z],
                            other.CopyNodes[b + t.w]);
                        float3 p0 = other.Positions[n.x];
                        var m = new float3x3(other.Positions[n.y] - p0, other.Positions[n.z] - p0, other.Positions[n.w] - p0);
                        if (!(math.abs(math.determinant(m)) > 1e-18f))
                        {
                            continue;
                        }
                        float3x3 inverse = math.inverse(m);
                        float3 bc = math.mul(inverse, x - p0);
                        if (1.0f - math.csum(bc) < -1e-5f || math.any(bc < -1e-5f))
                        {
                            continue;
                        }
                        float s0 = other.NodeSdf[n.x];
                        float3 ds = new float3(other.NodeSdf[n.y], other.NodeSdf[n.z], other.NodeSdf[n.w]) - s0;
                        float phi = s0 + math.dot(bc, ds);
                        float g = math.length(math.mul(math.transpose(inverse), ds));
                        deepest = math.max(deepest, g >= 0.5f ? -phi / g : -phi);
                        break;
                    }
                }
            }
            return deepest;
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
