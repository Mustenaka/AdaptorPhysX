using UnityEngine;
using UnityEngine.Serialization;

namespace APEX.Usage
{
    public class ApexDrag: MonoBehaviour
    {
        public int particleIndex;
        public Vector3 dragPosition;
        
        private void OnEnable()
        {
            dragPosition = transform.position;
        }

        private void Update()
        {
            dragPosition = transform.position;
        }
    } 
}