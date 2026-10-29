using System.Collections.Generic;
using AdaptorPhysX.Samples;
using AdaptorPhysX.Tools;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Builds the interaction demo (standard scenarios S8-S11): contact between soft bodies, a rigid
    /// ball on tissue (two-way coupling), a ring-shaped mesh instrument (distance field), plastic
    /// putty and a sticky paddle.
    /// </summary>
    public static class InteractionDemoSceneBuilder
    {
        public const string Folder = "Assets/Samples/InteractionDemo";
        public const string ScenePath = Folder + "/InteractionDemo.unity";

        [MenuItem("AdaptorPhysX/Build Interaction Demo Scene")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(Folder))
            {
                AssetDatabase.CreateFolder("Assets/Samples", "InteractionDemo");
            }
            Material skin = CreateMaterial("Skin", "Standard", new Color(0.93f, 0.72f, 0.64f), 0.45f);
            Material liver = CreateMaterial("Liver", "Standard", new Color(0.45f, 0.1f, 0.09f), 0.75f);
            Material putty = CreateMaterial("Putty", "Standard", new Color(0.55f, 0.62f, 0.85f), 0.3f);
            Material interior = CreateMaterial("TissueInterior", "AdaptorPhysX/Tissue Interior", Color.white, 0.65f);
            Material table = CreateMaterial("Table", "Standard", new Color(0.32f, 0.36f, 0.4f), 0.2f);
            Material steel = CreateMaterial("Steel", "Standard", new Color(0.75f, 0.77f, 0.8f), 0.85f);
            if (steel.HasProperty("_Metallic"))
            {
                steel.SetFloat("_Metallic", 0.8f);
            }

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var cameraObject = new GameObject("Main Camera", typeof(Camera)) { tag = "MainCamera" };
            var camera = cameraObject.GetComponent<Camera>();
            camera.nearClipPlane = 0.01f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.12f, 0.13f, 0.15f);
            var light = new GameObject("Light", typeof(Light)).GetComponent<Light>();
            light.type = LightType.Directional;
            light.shadows = LightShadows.Soft;
            light.transform.rotation = Quaternion.Euler(55.0f, -30.0f, 0.0f);
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.35f, 0.35f, 0.38f);

            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Table";
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.1f);
            floor.transform.localScale = new Vector3(1.0f, 0.1f, 0.6f);
            floor.GetComponent<MeshRenderer>().sharedMaterial = table;

            // S8: a lobe dropped onto a block. Both are soft bodies and rest on each other.
            CreateBody("Base Block", ApxDemoShape.Kind.Box, new Vector3(0.12f, 0.04f, 0.1f), new Vector3(-0.33f, 0.0205f, 0.0f),
                skin, interior, 16, 2.0e4f);
            CreateBody("Dropped Lobe", ApxDemoShape.Kind.Ellipsoid, new Vector3(0.09f, 0.04f, 0.065f),
                new Vector3(-0.33f, 0.095f, 0.0f), liver, interior, 14, 1.0e4f);

            // S9: a steel ball (Rigidbody) dropped on a hammock of tissue held by two clamps.
            // Clamped 2 cm thick band: about 4 mm sag under its own weight and 4 mm more under the ball.
            GameObject hammock = CreateBody("Hammock", ApxDemoShape.Kind.Box, new Vector3(0.16f, 0.02f, 0.07f),
                new Vector3(-0.11f, 0.06f, 0.0f), skin, interior, 28, 8.0e4f);
            Material clamp = CreateMaterial("Clamp", "Standard", new Color(0.7f, 0.72f, 0.75f), 0.8f);
            hammock.GetComponent<ApxSoftBody>().pinColliders = new[]
            {
                CreateTrigger("Clamp L", new Vector3(-0.19f, 0.06f, 0.0f), new Vector3(0.02f, 0.03f, 0.09f), clamp),
                CreateTrigger("Clamp R", new Vector3(-0.03f, 0.06f, 0.0f), new Vector3(0.02f, 0.03f, 0.09f), clamp),
            };
            GameObject ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ball.name = "Steel Ball";
            ball.transform.position = new Vector3(-0.11f, 0.1f, 0.0f);
            ball.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
            ball.GetComponent<MeshRenderer>().sharedMaterial = steel;
            var rigidbody = ball.AddComponent<Rigidbody>();
            rigidbody.mass = 0.1f;
            rigidbody.drag = 0.5f;
            rigidbody.angularDrag = 2.0f;

            // S10: a ring-shaped instrument (MeshCollider with a baked distance field) pressed into a block.
            CreateBody("Pressed Block", ApxDemoShape.Kind.Box, new Vector3(0.1f, 0.04f, 0.1f), new Vector3(0.11f, 0.0205f, 0.0f),
                skin, interior, 24, 1.5e4f);
            Mesh ring = SaveAsset(CreateTorus(0.025f, 0.006f, 48, 16), Folder + "/Ring.asset");
            var field = ScriptableObject.CreateInstance<ApxDistanceField>();
            field.mesh = ring;
            field.resolution = 48;
            field.Bake();
            field = SaveAsset(field, Folder + "/Ring Field.asset");
            var ringObject = new GameObject("Ring Instrument", typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider));
            ringObject.transform.position = new Vector3(0.11f, 0.0405f + 0.006f + 0.003f, 0.0f);
            ringObject.GetComponent<MeshFilter>().sharedMesh = ring;
            ringObject.GetComponent<MeshRenderer>().sharedMaterial = steel;
            ringObject.GetComponent<MeshCollider>().sharedMesh = ring;
            ringObject.AddComponent<ApxColliderMaterial>().distanceField = field;
            var ringPress = ringObject.AddComponent<ApxDemoPresser>();
            // 3 mm above the surface, pressed 7 mm: the tube sinks about half way.
            ringPress.depth = 0.007f;
            ringPress.phases = new Vector4(1.0f, 1.0f, 1.0f, 1.0f);

            // S11: plastic putty pressed once by a ball keeps the dent.
            GameObject puttyObject = CreateBody("Putty", ApxDemoShape.Kind.Box, new Vector3(0.1f, 0.03f, 0.08f),
                new Vector3(0.33f, 0.0155f, 0.0f), putty, interior, 24, 2.0e4f);
            var puttyBody = puttyObject.GetComponent<ApxSoftBody>();
            puttyBody.plasticYield = 0.02f;
            puttyBody.plasticCreep = 20.0f;
            puttyBody.maxPlasticDeformation = 0.6f;
            puttyBody.damping = 3.0f;
            GameObject press = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            press.name = "Putty Press";
            press.transform.position = new Vector3(0.33f, 0.031f + 0.015f + 0.003f, 0.0f);
            press.transform.localScale = new Vector3(0.03f, 0.03f, 0.03f);
            press.GetComponent<MeshRenderer>().sharedMaterial = steel;
            var puttyPress = press.AddComponent<ApxDemoPresser>();
            puttyPress.depth = 0.014f;
            puttyPress.overshoot = 0.03f;
            puttyPress.loop = false;

            // S11: a sticky paddle lifts wet tissue until it is pulled beyond the stick distance.
            CreateBody("Wet Tissue", ApxDemoShape.Kind.Box, new Vector3(0.1f, 0.03f, 0.08f), new Vector3(0.33f, 0.0155f, 0.22f),
                skin, interior, 20, 1.5e4f);
            GameObject paddle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            paddle.name = "Sticky Paddle";
            paddle.transform.position = new Vector3(0.33f, 0.031f + 0.005f, 0.22f);
            paddle.transform.localScale = new Vector3(0.04f, 0.01f, 0.04f);
            paddle.GetComponent<MeshRenderer>().sharedMaterial = steel;
            var sticky = paddle.AddComponent<ApxColliderMaterial>();
            sticky.stickiness = 1.0f;
            sticky.stickDistance = 0.006f;
            var paddlePress = paddle.AddComponent<ApxDemoPresser>();
            paddlePress.depth = 0.001f;
            paddlePress.overshoot = 0.012f;
            paddlePress.phases = new Vector4(0.5f, 0.5f, 0.5f, 2.0f);

            var view = new ApxOrbit { Pivot = new Vector3(0.0f, 0.03f, 0.05f), Pitch = 38.0f, Distance = 0.75f };
            camera.transform.SetPositionAndRotation(view.Position, view.Rotation);
            var scalpel = new GameObject("Scalpel", typeof(ApxScalpel)).GetComponent<ApxScalpel>();
            ApxToolbox.CreateDefault(camera, scalpel, view, clamp);

            EditorSceneManager.SaveScene(scene, ScenePath);
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (!scenes.Exists(s => s.path == ScenePath))
            {
                scenes.Add(new EditorBuildSettingsScene(ScenePath, true));
                EditorBuildSettings.scenes = scenes.ToArray();
            }
            AssetDatabase.SaveAssets();
            Debug.Log("AdaptorPhysX: interaction demo scene written to " + ScenePath);
        }

        private static GameObject CreateBody(string name, ApxDemoShape.Kind kind, Vector3 size, Vector3 position,
            Material skin, Material interior, int resolution, float youngsModulus)
        {
            var go = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer), typeof(ApxDemoShape), typeof(ApxSoftBody));
            go.transform.position = position;
            var shape = go.GetComponent<ApxDemoShape>();
            shape.kind = kind;
            shape.size = size;
            go.GetComponent<MeshRenderer>().sharedMaterial = skin;
            var body = go.GetComponent<ApxSoftBody>();
            body.resolution = resolution;
            body.youngsModulus = youngsModulus;
            body.cutMaterial = interior;
            body.collisionMargin = -0.002f;
            return go;
        }

        private static Collider CreateTrigger(string name, Vector3 position, Vector3 size, Material material)
        {
            GameObject part = GameObject.CreatePrimitive(PrimitiveType.Cube);
            part.name = name;
            part.transform.position = position;
            part.transform.localScale = size;
            var collider = part.GetComponent<BoxCollider>();
            collider.isTrigger = true;
            part.GetComponent<MeshRenderer>().sharedMaterial = material;
            return collider;
        }

        /// <summary>Torus in the xz-plane around the local origin.</summary>
        private static Mesh CreateTorus(float major, float minor, int segments, int sides)
        {
            var vertices = new List<Vector3>();
            var normals = new List<Vector3>();
            var triangles = new List<int>();
            for (int i = 0; i < segments; i++)
            {
                float u = 2.0f * Mathf.PI * i / segments;
                var axis = new Vector3(Mathf.Cos(u), 0.0f, Mathf.Sin(u));
                for (int j = 0; j < sides; j++)
                {
                    float v = 2.0f * Mathf.PI * j / sides;
                    Vector3 n = axis * Mathf.Cos(v) + Vector3.up * Mathf.Sin(v);
                    vertices.Add(axis * major + n * minor);
                    normals.Add(n);
                }
            }
            for (int i = 0; i < segments; i++)
            for (int j = 0; j < sides; j++)
            {
                int a = i * sides + j;
                int b = ((i + 1) % segments) * sides + j;
                int c = ((i + 1) % segments) * sides + (j + 1) % sides;
                int d = i * sides + (j + 1) % sides;
                triangles.AddRange(new[] { a, c, b, a, d, c });
            }
            var mesh = new Mesh { name = "Ring" };
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();
            return mesh;
        }

        private static T SaveAsset<T>(T asset, string path) where T : Object
        {
            var existing = AssetDatabase.LoadAssetAtPath<T>(path);
            if (existing != null)
            {
                EditorUtility.CopySerialized(asset, existing);
                Object.DestroyImmediate(asset);
                EditorUtility.SetDirty(existing);
                return existing;
            }
            AssetDatabase.CreateAsset(asset, path);
            return asset;
        }

        private static Material CreateMaterial(string name, string shader, Color color, float smoothness)
        {
            string path = $"{Folder}/Materials/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                if (!AssetDatabase.IsValidFolder(Folder + "/Materials"))
                {
                    AssetDatabase.CreateFolder(Folder, "Materials");
                }
                material = new Material(Shader.Find(shader));
                AssetDatabase.CreateAsset(material, path);
            }
            if (material.HasProperty("_Color") && shader == "Standard")
            {
                material.color = color;
            }
            if (material.HasProperty("_Glossiness"))
            {
                material.SetFloat("_Glossiness", smoothness);
            }
            EditorUtility.SetDirty(material);
            return material;
        }
    }
}
