using System;
using APEX.Common.Constraints;
using UnityEngine;

namespace APEX.Usage
{
    /// <summary>
    /// ApexPin:
    ///     a pair struct, link particle index with position
    /// </summary>
    public class ApexPin : MonoBehaviour
    {
        private Rigidbody body;
        public int particleIndex;
        public Vector3 pinPosition;

        public EApexParticleConstraintType type = EApexParticleConstraintType.Pin;
        public bool isSoftPin;
        public Vector3 executeForce;
        public float duration = 0.02f;
        public float elapsedTime = 0.0f;

        private void OnEnable()
        {
            pinPosition = transform.position;

            body = GetComponent<Rigidbody>();
            isSoftPin = body != null;
        }

        private void Update()
        {
            pinPosition = transform.position;
            ExecuteForce();
        }

        private void FixedUpdate()
        {
            // ExecuteForce();
        }

        /// <summary>
        /// Add force which decreasing over time(need = dt)
        /// </summary>
        public void AddForce(Vector3 force)
        {
            if (isSoftPin == false)
            {
                return;
            }

            // body.AddForce(force);
            executeForce = force;
        }

        /// <summary>
        /// execute the force(from rigidbody)
        /// </summary>
        private void ExecuteForce()
        {
            if (isSoftPin == false)
            {
                return;
            }

            // Calculate the time that has elapsed
            elapsedTime += Time.deltaTime;

            // Calculate the remaining time ratio
            float remainingTime = Mathf.Max(0, duration - elapsedTime);
            float forceMultiplier = remainingTime / duration;

            // A decreasing force is applied each frame
            var nextForce = executeForce * forceMultiplier * Time.deltaTime;
            Debug.DrawRay(this.transform.position, nextForce, Color.red, 10f);
            // Debug.Log($"start:{this.transform.position} forward to {nextForce}");

            body.AddForce(nextForce);

            // Stop applying force when time exceeds the duration
            if (elapsedTime >= duration)
            {
                // body.velocity = Vector3.zero;
                // body.angularVelocity = Vector3.zero;
                body.ResetInertiaTensor();

                executeForce = Vector3.zero;
                elapsedTime = 0.0f;
            }
        }
    }
}