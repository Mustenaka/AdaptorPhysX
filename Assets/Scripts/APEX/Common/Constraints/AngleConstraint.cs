using System;
using System.Collections.Generic;
using APEX.Common.Constraints.Base;
using APEX.Common.Particle;
using UnityEditor;
using UnityEngine;

namespace APEX.Common.Constraints
{
    // [CustomPropertyDrawer(typeof(AngleConstraint<>))]
    // public class AngleConstraintDrawer : PropertyDrawer
    // {
    //     public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    //     {
    //         EditorGUI.BeginProperty(position, label, property);
    //
    //         EditorGUI.PrefixLabel(position, label);
    //
    //         EditorGUI.indentLevel++;
    //
    //         // Find the SerializedProperty of the desiredAngle and stiffness fields
    //         SerializedProperty desiredAngleProperty = property.FindPropertyRelative("desiredAngle");
    //         SerializedProperty stiffnessProperty = property.FindPropertyRelative("stiffness");
    //
    //         // Calculate the height of the fields
    //         float desiredAngleHeight = EditorGUI.GetPropertyHeight(desiredAngleProperty);
    //         float stiffnessHeight = EditorGUI.GetPropertyHeight(stiffnessProperty);
    //
    //         // Draw the desiredAngle field
    //         Rect desiredAngleRect = new Rect(position.x, position.y + EditorGUIUtility.singleLineHeight, position.width, desiredAngleHeight);
    //         EditorGUI.PropertyField(desiredAngleRect, desiredAngleProperty);
    //
    //         // Draw the stiffness field below the desiredAngle field
    //         Rect stiffnessRect = new Rect(position.x, desiredAngleRect.y + desiredAngleHeight, position.width, stiffnessHeight);
    //         EditorGUI.PropertyField(stiffnessRect, stiffnessProperty);
    //
    //         EditorGUI.indentLevel--;
    //
    //         EditorGUI.EndProperty();
    //     }
    //
    //     public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    //     {
    //         // Find the SerializedProperty of the desiredAngle and stiffness fields
    //         SerializedProperty desiredAngleProperty = property.FindPropertyRelative("desiredAngle");
    //         SerializedProperty stiffnessProperty = property.FindPropertyRelative("stiffness");
    //
    //         // Calculate the height of the fields
    //         float desiredAngleHeight = EditorGUI.GetPropertyHeight(desiredAngleProperty);
    //         float stiffnessHeight = EditorGUI.GetPropertyHeight(stiffnessProperty);
    //
    //         // Return the total height of the fields
    //         return EditorGUIUtility.singleLineHeight + desiredAngleHeight + stiffnessHeight;
    //     }
    // }
    
    /// <summary>
    /// Angle constraint, a constraint based on the target three particle of the mid particle
    /// </summary>
    /// <typeparam name="T">particle</typeparam>
    [Serializable]
    public class AngleConstraint<T> : ApexConstraintBatchThree where T : ApexParticleBase
    {
        // public float maxAngle = Mathf.PI / ?
        [SerializeField] public float desiredAngle = Mathf.PI;
        // public float minAngle = Mathf.PI / ?
        [SerializeField] [Range(0, 1)] public float stiffness = 0.9f;

        private List<T> _particles;

        public AngleConstraint(ref List<T> particles, bool doubleConnect = false)
        {
            constraintBatchType = EApexConstraintBatchType.AngleConstraint;
            this._particles = particles;

            
            int cnt = particles.Count;

            // quick return, angle constraint must have more than 3 particle
            if (cnt < 3)
            {
                return;
            }

            // TEMP: constraint connect particle construct function.
            this.constraints = new Dictionary<int, List<ApexConstraintParticleThree>>();
            for (int i = 1; i < cnt - 1; i++)
            {
                var lToR = new ApexConstraintParticleThree(this._particles[i - 1].index, this._particles[i].index,
                    this._particles[i + 1].index);

                if (!constraints.ContainsKey(i))
                {
                    constraints.Add(i, new List<ApexConstraintParticleThree>());
                }

                constraints[i].Add(lToR);

                // Angle constraints often do not require a reverse connection
                if (doubleConnect)
                {
                    var rToL = new ApexConstraintParticleThree(this._particles[i + 1].index, this._particles[i].index,
                        this._particles[i - 1].index);

                    constraints[i].Add(rToL);
                }
            }
        }

        public override void Do()
        {
            foreach (var constraint in constraints)
            {
                foreach (var single in constraint.Value)
                {
                    CalcParticleConstraint(ref _particles[single.pl].nextPosition,
                        ref _particles[single.pmid].nextPosition,
                        ref _particles[single.pr].nextPosition,
                        _particles[single.pl].isStatic,
                        _particles[single.pmid].isStatic,
                        _particles[single.pr].isStatic);
                }
            }
        }

        public void CalcParticleConstraint(ref Vector3 l, ref Vector3 mid, ref Vector3 r, bool lStatic, bool midStatic,
            bool rStatic)
        {
            // calc now angle
            Vector3 dirLMid = (mid - l).normalized;
            Vector3 dirRMid = (mid - r).normalized;
            // Avoiding potential NaN issues with Vector3.Dot and Mathf.Acos
            float dotProduct = Vector3.Dot(dirLMid, dirRMid);
            dotProduct = Mathf.Clamp(dotProduct, -1f, 1f); // Ensure dot product is within valid range [-1, 1]
            float currentAngle = Mathf.Acos(dotProduct);
            // float currentAngle = Mathf.Acos(Vector3.Dot(dirLMid, dirRMid));

            // calc error if angle
            float angleError = currentAngle - desiredAngle;

            // position calibration
            Vector3 correction = stiffness * angleError * (dirLMid + dirRMid);

            if (!lStatic)
            {
                l -= correction;
            }

            if (!midStatic)
            {
                // mid += correction;
            }

            if (!rStatic)
            {
                r += correction;
            }
        }
    }
}