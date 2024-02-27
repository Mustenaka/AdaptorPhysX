using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Constraints
{
    public struct TestConstraint : IJobFor
    {
        public NativeArray<float3> nextPosition;

        public NativeArray<ApexConstraintParticleDouble> constraints;
        
        public void Execute(int index)
        {
            int i = 0;
            for (; i < nextPosition.Length; i++)
            {
                Debug.Log(nextPosition[i] + " i:" + i);
            }
        }
    }
}