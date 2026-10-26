using System.IO;
using UnityEditor;
using UnityEngine;

namespace AdaptorPhysX.Editor
{
    /// <summary>ApxSoftBody inspector: bakes the blueprint asset that skips voxelization at Start.</summary>
    [CustomEditor(typeof(ApxSoftBody))]
    [CanEditMultipleObjects]
    public sealed class ApxSoftBodyEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Blueprint", EditorStyles.boldLabel);
            foreach (Object target in targets)
            {
                var body = (ApxSoftBody)target;
                ApxSoftBodyBlueprint blueprint = body.blueprint;
                if (blueprint == null)
                {
                    EditorGUILayout.HelpBox($"{body.name}: no blueprint, the mesh is voxelized at Start.", MessageType.None);
                }
                else if (!body.BlueprintMatches())
                {
                    EditorGUILayout.HelpBox($"{body.name}: the blueprint was baked for other settings " +
                                            $"(resolution {blueprint.Resolution}, scale {blueprint.Scale}); bake again.",
                        MessageType.Warning);
                }
                else
                {
                    EditorGUILayout.HelpBox($"{body.name}: {blueprint.CellCount} cells, {blueprint.SurfaceVertexCount} surface vertices.",
                        MessageType.Info);
                }
            }
            if (GUILayout.Button(targets.Length > 1 ? "Bake Blueprints" : "Bake Blueprint"))
            {
                foreach (Object target in targets)
                {
                    Bake((ApxSoftBody)target);
                }
            }
        }

        /// <summary>Writes the blueprint into the body's asset, or a new one next to the scene.</summary>
        public static void Bake(ApxSoftBody body)
        {
            ApxSoftBodyBlueprint blueprint = body.blueprint;
            bool create = blueprint == null || !AssetDatabase.Contains(blueprint);
            if (create)
            {
                string folder = Path.GetDirectoryName(body.gameObject.scene.path);
                if (string.IsNullOrEmpty(folder))
                {
                    folder = "Assets";
                }
                string path = AssetDatabase.GenerateUniqueAssetPath($"{folder}/{body.name} Blueprint.asset");
                blueprint = CreateInstance<ApxSoftBodyBlueprint>();
                AssetDatabase.CreateAsset(blueprint, path);
            }
            if (!body.BakeBlueprint(blueprint))
            {
                return;
            }
            EditorUtility.SetDirty(blueprint);
            AssetDatabase.SaveAssets();
            Undo.RecordObject(body, "Bake Blueprint");
            body.blueprint = blueprint;
            EditorUtility.SetDirty(body);
        }
    }

    [CustomEditor(typeof(ApxDistanceField))]
    public sealed class ApxDistanceFieldEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var field = (ApxDistanceField)target;
            EditorGUILayout.HelpBox(field.IsBaked
                ? $"Baked: {field.Field.Dims.x} x {field.Field.Dims.y} x {field.Field.Dims.z} points, cell {field.Field.CellSize:0.####}."
                : "Not baked for the current settings; bakes on first use.", field.IsBaked ? MessageType.Info : MessageType.Warning);
            if (GUILayout.Button("Bake"))
            {
                field.Bake();
                EditorUtility.SetDirty(field);
                AssetDatabase.SaveAssets();
            }
        }
    }
}
