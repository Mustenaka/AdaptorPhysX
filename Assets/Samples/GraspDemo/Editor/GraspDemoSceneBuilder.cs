using System.Collections.Generic;
using AdaptorPhysX.Tools;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Builds the grasp demo (standard scenario S13, TODO MED-10): a soft tissue sheet held at its two ends by clamps above a
    /// table, with the interaction tools. The grasper tool (key 4, selected at start) takes the sheet by its free edges.
    /// </summary>
    public static class GraspDemoSceneBuilder
    {
        public const string Folder = "Assets/Samples/GraspDemo";
        public const string ScenePath = Folder + "/GraspDemo.unity";

        [MenuItem("AdaptorPhysX/Build Grasp Demo Scene")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(Folder))
            {
                AssetDatabase.CreateFolder("Assets/Samples", "GraspDemo");
            }
            Material skin = CreateMaterial("Skin", "Standard", new Color(0.93f, 0.72f, 0.64f), 0.45f);
            Material interior = CreateMaterial("TissueInterior", "AdaptorPhysX/Tissue Interior", Color.white, 0.65f);
            Material table = CreateMaterial("Table", "Standard", new Color(0.32f, 0.36f, 0.4f), 0.2f);
            Material metal = CreateMaterial("Metal", "Standard", new Color(0.7f, 0.72f, 0.75f), 0.8f);

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
            floor.transform.position = new Vector3(0.0f, -0.05f, 0.0f);
            floor.transform.localScale = new Vector3(0.6f, 0.1f, 0.4f);
            floor.GetComponent<MeshRenderer>().sharedMaterial = table;

            // A sheet 1 cm thick and soft (20 kPa), clamped at its short ends: it sags under its weight, and its long edges are
            // free for a jaw above and one below.
            var sheet = new GameObject("Tissue Sheet", typeof(MeshFilter), typeof(MeshRenderer), typeof(ApxDemoShape), typeof(ApxSoftBody));
            sheet.transform.position = new Vector3(0.0f, 0.07f, 0.0f);
            var shape = sheet.GetComponent<ApxDemoShape>();
            shape.kind = ApxDemoShape.Kind.Box;
            shape.size = new Vector3(0.16f, 0.01f, 0.08f);
            sheet.GetComponent<MeshRenderer>().sharedMaterial = skin;
            var body = sheet.GetComponent<ApxSoftBody>();
            body.resolution = 48;
            body.youngsModulus = 2.0e4f;
            body.cutMaterial = interior;
            body.pinColliders = new[] { CreateClamp("Clamp L", -0.08f, metal), CreateClamp("Clamp R", 0.08f, metal) };

            var view = new ApxOrbit { Pivot = new Vector3(0.0f, 0.045f, 0.0f), Pitch = 30.0f, Distance = 0.3f };
            camera.transform.SetPositionAndRotation(view.Position, view.Rotation);
            var scalpel = new GameObject("Scalpel", typeof(ApxScalpel)).GetComponent<ApxScalpel>();
            ApxToolbox toolbox = ApxToolbox.CreateDefault(camera, scalpel, view, metal);
            toolbox.activeTool = System.Array.FindIndex(toolbox.tools, t => t is ApxGrasperTool);

            EditorSceneManager.SaveScene(scene, ScenePath);
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (!scenes.Exists(s => s.path == ScenePath))
            {
                scenes.Add(new EditorBuildSettingsScene(ScenePath, true));
                EditorBuildSettings.scenes = scenes.ToArray();
            }
            AssetDatabase.SaveAssets();
            Debug.Log("AdaptorPhysX: grasp demo scene written to " + ScenePath);
        }

        /// <summary>Trigger box that pins the sheet's end inside it, drawn in <paramref name="material"/>.</summary>
        private static Collider CreateClamp(string name, float x, Material material)
        {
            GameObject clamp = GameObject.CreatePrimitive(PrimitiveType.Cube);
            clamp.name = name;
            clamp.transform.position = new Vector3(x, 0.07f, 0.0f);
            clamp.transform.localScale = new Vector3(0.025f, 0.03f, 0.1f);
            var collider = clamp.GetComponent<BoxCollider>();
            collider.isTrigger = true;
            clamp.GetComponent<MeshRenderer>().sharedMaterial = material;
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
