using System.Collections.Generic;
using AdaptorPhysX.Tools;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace AdaptorPhysX.Samples.Editor
{
    /// <summary>
    /// Builds the real mesh demo (standard scenarios S12): the scanned Stanford bunny (open at the bottom),
    /// Fertility (genus 4) and the Stanford dragon, the FBX test models in Assets/AdaptorPhysX/Arts/Models,
    /// as soft bodies on a table with the interaction tools.
    /// </summary>
    public static class RealMeshDemoSceneBuilder
    {
        public const string Folder = "Assets/Samples/RealMeshDemo";
        public const string ScenePath = Folder + "/RealMeshDemo.unity";
        private const string Models = "Assets/AdaptorPhysX/Arts/Models/";

        [MenuItem("AdaptorPhysX/Build Real Mesh Demo Scene")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(Folder))
            {
                AssetDatabase.CreateFolder("Assets/Samples", "RealMeshDemo");
            }
            Material skin = CreateMaterial("Skin", "Standard", new Color(0.93f, 0.72f, 0.64f), 0.45f);
            Material clay = CreateMaterial("Clay", "Standard", new Color(0.78f, 0.62f, 0.5f), 0.3f);
            Material jade = CreateMaterial("Jade", "Standard", new Color(0.45f, 0.62f, 0.5f), 0.55f);
            Material interior = CreateMaterial("TissueInterior", "AdaptorPhysX/Tissue Interior", Color.white, 0.65f);
            Material table = CreateMaterial("Table", "Standard", new Color(0.32f, 0.36f, 0.4f), 0.2f);

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
            floor.transform.localScale = new Vector3(1.0f, 0.1f, 0.6f);
            floor.GetComponent<MeshRenderer>().sharedMaterial = table;

            // Resolution 40: cells of 4-5 mm, a few thousand to ten thousand cells each, for interactive rates.
            // The models are y-up; the FBX import turns them as if they were z-up, so they lie down as in a scene
            // they are dragged into. The bunny is stood up again; Fertility and the dragon rest on their sides.
            CreateBody("Scanned Bunny", Models + "Stanford_Bunny_69451_triangles.fbx", new Vector3(-0.26f, 0.0f, 0.0f), true,
                skin, interior, 40);
            CreateBody("Fertility", Models + "Fertility_483226_triangles.fbx", new Vector3(0.0f, 0.0f, 0.0f), false,
                clay, interior, 40);
            CreateBody("Dragon", Models + "Stanford_Dragon_871414_triangles.fbx", new Vector3(0.26f, 0.0f, 0.0f), false,
                jade, interior, 40);

            var view = new ApxOrbit { Pivot = new Vector3(0.0f, 0.05f, 0.0f), Pitch = 30.0f, Distance = 0.8f };
            camera.transform.SetPositionAndRotation(view.Position, view.Rotation);
            var scalpel = new GameObject("Scalpel", typeof(ApxScalpel)).GetComponent<ApxScalpel>();
            ApxToolbox.CreateDefault(camera, scalpel, view);

            EditorSceneManager.SaveScene(scene, ScenePath);
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (!scenes.Exists(s => s.path == ScenePath))
            {
                scenes.Add(new EditorBuildSettingsScene(ScenePath, true));
                EditorBuildSettings.scenes = scenes.ToArray();
            }
            AssetDatabase.SaveAssets();
            Debug.Log("AdaptorPhysX: real mesh demo scene written to " + ScenePath);
        }

        /// <summary>
        /// A soft body with the first mesh of the model at <paramref name="model"/>, oriented and scaled as in the
        /// model (FBX axis conversion; <paramref name="upright"/> turns it back by 90 degrees about x), resting 2 mm
        /// above the table at <paramref name="position"/> (x, z).
        /// </summary>
        private static GameObject CreateBody(string name, string model, Vector3 position, bool upright, Material skin,
            Material interior, int resolution)
        {
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(model);
            MeshFilter source = asset.GetComponentInChildren<MeshFilter>(true);
            Matrix4x4 toRoot = ToRoot(source.transform, asset.transform);
            var go = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer));
            go.transform.rotation = (upright ? Quaternion.Euler(90.0f, 0.0f, 0.0f) : Quaternion.identity) * toRoot.rotation;
            go.transform.localScale = toRoot.lossyScale;
            go.GetComponent<MeshFilter>().sharedMesh = source.sharedMesh;
            go.GetComponent<MeshRenderer>().sharedMaterial = skin;
            Bounds bounds = WorldBounds(go, source.sharedMesh);
            go.transform.position = new Vector3(position.x, 0.002f, position.z) -
                                    new Vector3(bounds.center.x, bounds.min.y, bounds.center.z);
            Debug.Log($"AdaptorPhysX: {name} from {model}: scale {go.transform.localScale}, size {bounds.size.ToString("F3")} m");
            var body = go.AddComponent<ApxSoftBody>();
            body.resolution = resolution;
            body.youngsModulus = 6.0e4f;
            body.cutMaterial = interior;
            body.collisionMargin = -0.002f;
            return go;
        }

        /// <summary>
        /// Rotation and scale of <paramref name="t"/> in the model, composed from the local values of each level up
        /// to and including the model's root (without its position): a single-mesh FBX keeps the mesh on the root,
        /// whose rotation and scale carry the axis conversion.
        /// </summary>
        private static Matrix4x4 ToRoot(Transform t, Transform root)
        {
            Matrix4x4 m = Matrix4x4.identity;
            for (; t != null; t = t.parent)
            {
                m = Matrix4x4.TRS(t == root ? Vector3.zero : t.localPosition, t.localRotation, t.localScale) * m;
                if (t == root)
                {
                    break;
                }
            }
            return m;
        }

        /// <summary>World bounds of the mesh vertices at the object's current rotation and scale (position zero).</summary>
        private static Bounds WorldBounds(GameObject go, Mesh mesh)
        {
            Matrix4x4 m = Matrix4x4.TRS(Vector3.zero, go.transform.rotation, go.transform.localScale);
            Vector3[] vertices = mesh.vertices;
            var bounds = new Bounds(m.MultiplyPoint3x4(vertices[0]), Vector3.zero);
            foreach (Vector3 v in vertices)
            {
                bounds.Encapsulate(m.MultiplyPoint3x4(v));
            }
            return bounds;
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
