using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Contact settings of a collider against soft bodies: friction, adhesion (wet tissue sticking
    /// to an instrument) and an optional distance field that replaces the collider's own shape.
    /// MeshColliders without this component use a distance field baked from their mesh at the
    /// default resolution.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Collider))]
    public sealed class ApxColliderMaterial : MonoBehaviour
    {
        public const int DefaultFieldResolution = 32;

        [Tooltip("Friction coefficient against soft bodies. Negative uses the soft body's friction.")]
        public float friction = -1.0f;
        [Tooltip("Fraction of the gap to the collider closed per substep for nodes within stickDistance: " +
                 "tissue follows an instrument that pulls away until it is stickDistance away.")]
        [Range(0.0f, 1.0f)] public float stickiness;
        [Tooltip("Distance from the collider surface (beyond the soft body's collision margin) within which nodes stick, in metres.")]
        [Min(0.0f)] public float stickDistance = 0.002f;

        [Header("Shape")]
        [Tooltip("Distance field used as the collider's shape (the Collider then only bounds it). " +
                 "Empty: MeshColliders bake one from their mesh; other colliders keep their primitive shape.")]
        public ApxDistanceField distanceField;
        [Tooltip("Grid cells along the longest axis of a field baked from a MeshCollider at runtime.")]
        [Range(8, 128)] public int fieldResolution = DefaultFieldResolution;
    }
}
