using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace AdaptorPhysX.Tests
{
    /// <summary>The pre-tensioned skin flap of the surgery demo, run through the components in edit mode.</summary>
    public sealed class SkinFlapTests
    {
        private const float Dt = 1.0f / 60.0f;

        private readonly List<GameObject> _objects = new List<GameObject>();
        private ApxSoftBody _flap;
        private ApxScalpel _scalpel;

        [SetUp]
        public void SetUp()
        {
            // As built by SurgeryDemoSceneBuilder: 18 x 1.6 x 12 cm, resolution 36, clamped at both ends.
            GameObject table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            table.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            table.transform.localScale = new Vector3(1.2f, 0.1f, 0.8f);
            _objects.Add(table);

            var flapObject = new GameObject("Skin Flap", typeof(MeshFilter), typeof(MeshRenderer));
            flapObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.18f, 0.016f, 0.12f), 12);
            flapObject.transform.position = new Vector3(0.0f, 0.06f, 0.0f);
            _objects.Add(flapObject);
            _flap = flapObject.AddComponent<ApxSoftBody>();
            _flap.resolution = 36;
            _flap.youngsModulus = 6.0e4f;
            _flap.preStrain = 0.2f;
            _flap.pinColliders = new[] { Clamp(-0.09f), Clamp(0.09f) };

            var scalpelObject = new GameObject("Scalpel");
            _objects.Add(scalpelObject);
            _scalpel = scalpelObject.AddComponent<ApxScalpel>();
            _scalpel.cutting = false;

            Physics.SyncTransforms();
            _flap.Initialize();
        }

        [TearDown]
        public void TearDown()
        {
            _flap.Release();
            foreach (GameObject o in _objects)
            {
                Object.DestroyImmediate(o);
            }
            _objects.Clear();
        }

        /// <summary>
        /// A 15 mm blade kept below the tissue under it, as in the demo, cuts from the middle of the 16 mm flap
        /// out through its near edge, a little above the lowest lattice nodes. Where the cut lies close to lattice
        /// nodes, skin vertices next to it were evaluated in the copy across the cut and stretched skin over
        /// the opening incision.
        /// </summary>
        [TestCase(-0.02f, 60)]
        [TestCase(-0.012f, 40)]
        public void CutOutThroughTheEdgeLeavesNoSkinAcrossTheIncision(float x, int frames)
        {
            SoftBodySimulation sim = _flap.Simulation;
            Run(45);
            _scalpel.edgeStart = Vector3.zero;
            _scalpel.edgeEnd = Vector3.down * 0.025f;
            float height = 0.0f;
            _scalpel.cutting = true;
            int severed = 0;
            for (int f = 0; f <= frames; f++)
            {
                float z = Mathf.Lerp(0.02f, -0.09f, f / (float)frames);
                int node = sim.PickNode(new Vector3(x, 0.3f, z), Vector3.down, sim.CellSize);
                height = node >= 0 ? sim.Positions[node].y : height;
                _scalpel.transform.position = new Vector3(x, height + 0.01f, z);
                Frame();
                severed += _scalpel.LastSeveredEdges;
            }
            _scalpel.cutting = false;
            Run(30);
            Assert.Greater(severed, 0);

            Vector3[] vertices = _flap.Surface.Mesh.vertices;
            int[] skin = _flap.Surface.Mesh.GetTriangles(0);
            int stretched = 0;
            for (int t = 0; t < skin.Length; t += 3)
            {
                float longest = 0.0f;
                for (int i = 0; i < 3; i++)
                {
                    longest = Mathf.Max(longest, Vector3.Distance(vertices[skin[t + i]], vertices[skin[t + (i + 1) % 3]]));
                }
                stretched += longest > 2.0f * sim.CellSize ? 1 : 0;
            }
            Assert.AreEqual(0, stretched, "skin triangles longer than two cells");
        }

        private Collider Clamp(float x)
        {
            GameObject clamp = GameObject.CreatePrimitive(PrimitiveType.Cube);
            clamp.transform.position = new Vector3(x, 0.06f, 0.0f);
            clamp.transform.localScale = new Vector3(0.025f, 0.03f, 0.14f);
            var collider = clamp.GetComponent<BoxCollider>();
            collider.isTrigger = true;
            _objects.Add(clamp);
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
            _flap.Simulate(Dt);
            _flap.UpdateRender();
        }
    }
}
