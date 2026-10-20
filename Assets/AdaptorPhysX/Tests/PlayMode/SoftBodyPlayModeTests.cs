using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace AdaptorPhysX.Tests
{
    /// <summary>Runs the component path (Start / FixedUpdate / LateUpdate) with a moving scalpel.</summary>
    public sealed class SoftBodyPlayModeTests
    {
        [UnityTest]
        public IEnumerator ScalpelStrokeSplitsBodyOnTheFloor()
        {
            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            floor.transform.localScale = new Vector3(1.0f, 0.1f, 1.0f);

            var bodyObject = new GameObject("Body", typeof(MeshFilter), typeof(MeshRenderer));
            bodyObject.GetComponent<MeshFilter>().sharedMesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.04f, 0.08f), 8);
            bodyObject.transform.position = new Vector3(0.0f, 0.025f, 0.0f);
            var body = bodyObject.AddComponent<ApxSoftBody>();
            body.resolution = 12;
            body.substeps = 8;

            var scalpelObject = new GameObject("Scalpel");
            var scalpel = scalpelObject.AddComponent<ApxScalpel>();
            scalpel.edgeStart = Vector3.zero;
            scalpel.edgeEnd = new Vector3(0.0f, -0.08f, 0.0f);
            scalpel.cutting = false;
            scalpelObject.transform.position = new Vector3(0.001f, 0.07f, -0.08f);

            for (int i = 0; i < 20; i++)
            {
                yield return new WaitForFixedUpdate();
            }
            Assert.IsTrue(body.IsInitialized);
            Assert.AreEqual(1, body.Simulation.CountPieces());

            scalpel.cutting = true;
            for (int i = 0; i <= 30; i++)
            {
                scalpelObject.transform.position = new Vector3(0.001f, 0.07f, Mathf.Lerp(-0.08f, 0.08f, i / 30.0f));
                yield return new WaitForFixedUpdate();
            }
            scalpel.cutting = false;
            for (int i = 0; i < 30; i++)
            {
                yield return null;
            }

            Assert.AreEqual(2, body.Simulation.CountPieces());
            Assert.Greater(body.Surface.CutTriangleCount, 0);
            foreach (Vector3 v in body.Surface.Mesh.vertices)
            {
                Assert.IsFalse(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z));
            }

            Object.Destroy(scalpelObject);
            Object.Destroy(bodyObject);
            Object.Destroy(floor);
            yield return null;
        }
    }
}
