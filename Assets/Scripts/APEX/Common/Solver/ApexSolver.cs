using System.Collections.Generic;
using APEX.Common.Constraints;
using APEX.Common.Particle;
using APEX.Tools;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace APEX.Common.Solver
{
    [CustomEditor(typeof(ApexSolver))]
    public class ApexSolverEditor : Editor
    {
        private SerializedObject serializedSolver;
        private ReorderableList constraintBatchList;

        private void OnEnable()
        {
            serializedSolver = new SerializedObject(target);
            InitializeReorderableList();
        }

        public override void OnInspectorGUI()
        {
            // 更新 SerializedObject
            serializedSolver.Update();

            // Draw the default inspector
            DrawDefaultInspector();
            
            // 绘制约束批次列表
            constraintBatchList.DoLayoutList();

            // 应用修改
            serializedSolver.ApplyModifiedProperties();
        }

        // 在 ApexSolverEditor 中修改
        private void InitializeReorderableList()
        {
            Debug.Log("-------------");
            SerializedProperty constraintBatchProp = serializedSolver.FindProperty("constraintBatch");
            
            // 创建 ReorderableList
            constraintBatchList = new ReorderableList(serializedSolver, constraintBatchProp, true, true, true, true);
            constraintBatchList.elementHeight = 0; // 设置元素高度为 0，确保元素不会被折叠
            Debug.Log("+++++++++++++++" + constraintBatchList.count);
            
            // Batch List Count
            if (constraintBatchList.count == 0)
            {
                constraintBatchList.serializedProperty.arraySize = 1;
            }
            
            // 绘制元素
            constraintBatchList.drawElementCallback = (Rect rect, int index, bool isActive, bool isFocused) =>
            {
                Debug.Log("+++++++++++++++2222");
                SerializedProperty element = constraintBatchList.serializedProperty.GetArrayElementAtIndex(index);
                rect.y += 2;

                SerializedProperty typeProp = element.FindPropertyRelative("constraintBatchType");
                EApexConstraintBatchType type = (EApexConstraintBatchType)typeProp.enumValueIndex;

                Debug.Log("Type :" + type);
                
                switch (type)
                {
                    case EApexConstraintBatchType.AngleConstraint:
                        DrawAngleConstraintEditor(element);
                        break;
                    // 添加其他约束类型的处理方法
                }
            };
            
            // 绘制标题
            constraintBatchList.drawHeaderCallback = (Rect rect) =>
            {
                EditorGUI.LabelField(rect, "Constraint Batches");
            };
        }

        private void DrawAngleConstraintEditor(SerializedProperty element)
        {
            Debug.Log("Drawing AngleConstraint Editor");
            
            // 获取 AngleConstraint 的 SerializedProperty
            SerializedProperty desiredAngleProp = element.FindPropertyRelative("desiredAngle");
            SerializedProperty stiffnessProp = element.FindPropertyRelative("stiffness");

            // 绘制 AngleConstraint 具体属性
            EditorGUILayout.PropertyField(desiredAngleProp, new GUIContent("Desired Angle"));
            EditorGUILayout.PropertyField(stiffnessProp, new GUIContent("Stiffness"));
        }
    }
    
    public class ApexSolver: MonoBehaviour 
    {
        public List<IApexConstraintBatch> constraintBatch = new List<IApexConstraintBatch>();
        public List<ApexParticleBase> particles = new List<ApexParticleBase>();     // particle container
            
        // physics param
        public Vector3 gravity = new Vector3(0, -9.81f, 0);
        [Range(0, 1f)] public float stiffness = 0.5f;
        [Range(0, 1f)] public float damping = 0.5f;
        
        // simulator param
        public float dt = 0.002f;
        public float accTime;
        public int iterator = 10;

        private void Update()
        {
            accTime += Time.deltaTime;
            int cnt = (int)(accTime / dt);

            for (int i = 0; i < cnt; i++)
            {
                Simulator();
            }

            accTime %= dt;
        }

        private void Simulator()
        {
            for (int i = 0; i < iterator; i++)
            {
                // Do Gravite
                SimulateGravity();
                
                // Do Global ｜ Local Force
            
                // Do Constraint
                SimulateConstraint();
                
                // Update
                SimulateUpdate();
            }
        }
        
        private void SimulateGravity()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // simplex pin
                if (particles[i].isStatic)
                {
                    continue;
                }
                
                particles[i].nextPosition = particles[i].nowPosition 
                                            + (1 - damping) * (particles[i].nowPosition - particles[i].previousPosition)
                                            + gravity / particles[i].mass * (dt * dt);
            }
        }

        private void SimulateConstraint()
        {
            foreach (var constraint in constraintBatch)
            {
                constraint.Do();
            }
        }

        private void SimulateUpdate()
        {
            for (int i = 0; i < particles.Count; i++)
            {
                // if particle is static, not apply
                if (particles[i].isStatic)
                {
                    continue;
                }

                // if the nextPosition is NaN, will not apply
                if (NumberCheck.IsVector3NaN(particles[i].nextPosition))
                {
                    particles[i].nextPosition = particles[i].nowPosition;
                }
                
                particles[i].previousPosition = particles[i].nowPosition;
                particles[i].nowPosition = particles[i].nextPosition;
            }
        }
    }
}