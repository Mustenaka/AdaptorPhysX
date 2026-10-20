using AdaptorPhysX.Samples;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>Builds the surgery demo scene and its materials (menu or -executeMethod).</summary>
    public static class SurgeryDemoSceneBuilder
    {
        private const string Folder = "Assets/Samples/SurgeryDemo";
        private const string ScenePath = Folder + "/SurgeryDemo.unity";

        [MenuItem("AdaptorPhysX/Build Surgery Demo Scene")]
        public static void Build()
        {
            Material skin = CreateMaterial("Skin", "Standard", new Color(0.93f, 0.72f, 0.64f), 0.45f);
            Material liver = CreateMaterial("Liver", "Standard", new Color(0.45f, 0.1f, 0.09f), 0.75f);
            Material interior = CreateMaterial("TissueInterior", "AdaptorPhysX/Tissue Interior", Color.white, 0.65f);
            Material floor = CreateMaterial("Table", "Standard", new Color(0.32f, 0.36f, 0.4f), 0.2f);

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var cameraObject = new GameObject("Main Camera", typeof(Camera), typeof(AudioListener)) { tag = "MainCamera" };
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

            GameObject table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            table.name = "Table";
            table.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            table.transform.localScale = new Vector3(1.2f, 0.1f, 0.8f);
            table.GetComponent<MeshRenderer>().sharedMaterial = floor;

            CreateBody("Tissue Block", ApxDemoShape.Kind.Box, new Vector3(0.12f, 0.05f, 0.1f),
                new Vector3(-0.2f, 0.028f, 0.0f), skin, interior, 20, 2.0e4f, -0.003f);

            // Pre-tensioned skin flap stretched between two clamps: incisions gape open.
            GameObject flap = CreateBody("Skin Flap", ApxDemoShape.Kind.Box, new Vector3(0.18f, 0.016f, 0.12f),
                new Vector3(0.0f, 0.06f, 0.0f), skin, interior, 36, 6.0e4f, 0.0f);
            var flapBody = flap.GetComponent<ApxSoftBody>();
            flapBody.preStrain = 0.2f;
            flapBody.pinColliders = new Collider[]
            {
                CreateClamp("Clamp L", new Vector3(-0.09f, 0.06f, 0.0f)),
                CreateClamp("Clamp R", new Vector3(0.09f, 0.06f, 0.0f)),
            };

            CreateBody("Liver Lobe", ApxDemoShape.Kind.Ellipsoid, new Vector3(0.16f, 0.06f, 0.11f),
                new Vector3(0.22f, 0.033f, 0.0f), liver, interior, 24, 1.0e4f, -0.003f);

            var scalpel = new GameObject("Scalpel", typeof(ApxScalpel));
            var controller = new GameObject("Surgery Demo", typeof(SurgeryDemoController)).GetComponent<SurgeryDemoController>();
            controller.scalpel = scalpel.GetComponent<ApxScalpel>();

            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();
            Debug.Log("AdaptorPhysX: surgery demo scene written to " + ScenePath);
        }

        private static GameObject CreateBody(string name, ApxDemoShape.Kind kind, Vector3 size, Vector3 position,
            Material skin, Material interior, int resolution, float youngsModulus, float collisionMargin)
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
            body.collisionMargin = collisionMargin;
            return go;
        }

        private static Collider CreateClamp(string name, Vector3 position)
        {
            GameObject clamp = GameObject.CreatePrimitive(PrimitiveType.Cube);
            clamp.name = name;
            clamp.transform.position = position;
            clamp.transform.localScale = new Vector3(0.025f, 0.03f, 0.14f);
            var collider = clamp.GetComponent<BoxCollider>();
            collider.isTrigger = true;
            clamp.GetComponent<MeshRenderer>().sharedMaterial =
                CreateMaterial("Clamp", "Standard", new Color(0.7f, 0.72f, 0.75f), 0.8f);
            return collider;
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
