using APEX.Usage;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class PhaseOneDemoSceneBuilder
{
    private const string ScenePath = "Assets/Scenes/PhaseOneDemo.unity";

    [MenuItem("AdaptorPhysX/Rebuild Phase 1 Demo Scene")]
    public static void Build()
    {
        Scene scene = EditorSceneManager.NewScene(
            NewSceneSetup.EmptyScene,
            NewSceneMode.Single);

        GameObject cameraObject = new GameObject("Main Camera");
        cameraObject.tag = "MainCamera";
        Camera camera = cameraObject.AddComponent<Camera>();
        camera.clearFlags = CameraClearFlags.SolidColor;
        camera.backgroundColor = new Color(0.025F, 0.035F, 0.055F, 1.0F);
        cameraObject.transform.position = new Vector3(0.0F, 0.0F, -12.0F);
        cameraObject.transform.rotation = Quaternion.identity;

        GameObject lightObject = new GameObject("Directional Light");
        Light light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.1F;
        lightObject.transform.rotation = Quaternion.Euler(35.0F, -30.0F, 0.0F);

        GameObject clothObject = new GameObject("PhaseOne_100k_Cloth");
        clothObject.AddComponent<MeshFilter>();
        MeshRenderer clothRenderer = clothObject.AddComponent<MeshRenderer>();
        clothRenderer.sharedMaterial =
            AssetDatabase.GetBuiltinExtraResource<Material>("Default-Material.mat");
        PhaseOneDemoController controller =
            clothObject.AddComponent<PhaseOneDemoController>();

        GameObject preview = GameObject.CreatePrimitive(PrimitiveType.Quad);
        preview.name = "PreciseCut_Preview";
        preview.transform.position = Vector3.zero;
        preview.transform.localScale = new Vector3(8.0F, 5.0F, 1.0F);
        Object.DestroyImmediate(preview.GetComponent<Collider>());
        ObjCut preciseCut = preview.AddComponent<ObjCut>();
        preciseCut.target = preview;
        preciseCut.usePreciseRenderMeshCut = true;
        preciseCut.tickness = 0.001F;

        GameObject blade = new GameObject("FixedTick_Blade");
        blade.transform.position = new Vector3(-2.0F, 0.0F, -0.1F);
        blade.transform.rotation = Quaternion.LookRotation(Vector3.right, Vector3.up);
        ApxBladeInteractor interactor = blade.AddComponent<ApxBladeInteractor>();
        interactor.bladeTip = blade.transform;
        interactor.sideNormalSource = blade.transform;
        interactor.renderMeshTarget = preciseCut;
        interactor.minimumSampleDistance = 0.02F;
        interactor.cutRadius = 0.001F;

        controller.liveBlade = interactor;
        controller.precisionCutPreview = preciseCut;

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene, ScenePath))
        {
            throw new UnityException($"Unable to save {ScenePath}");
        }
        AssetDatabase.SaveAssets();
        Debug.Log($"Rebuilt {ScenePath}");
    }
}
