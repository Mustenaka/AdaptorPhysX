using System;
using UnityEngine;

namespace APEX.Usage
{
    /// <summary>
    /// ApexPin:
    ///     a pair struct, link particle index with position
    /// </summary>
    public class ApexPin : MonoBehaviour
    {
        public int particleIndex;
        public Vector3 pinPosition;

        private void OnEnable()
        {
            pinPosition = transform.position;
        }

        private void Update()
        {
            pinPosition = transform.position;
        }
    }
}