using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace APEX.Common.Force
{
    [BurstCompile]
    public struct ForceJob : IJobFor
    {
        [ReadOnly] public NativeArray<float3> previousPosition;
        [ReadOnly] public NativeArray<float3> nowPosition;
        [WriteOnly] public NativeArray<float3> nextPosition;

        [ReadOnly] public NativeArray<float> mass;

        [ReadOnly] public NativeArray<float3> forceExt;
        [ReadOnly] public float3 gravity;
        [ReadOnly] public float3 globalForce;

        [ReadOnly] public float airDrag;
        [ReadOnly] public float damping;
        [ReadOnly] public float dt;

        /// <summary>
        /// Execute force extend. (Contain air resistance, force apply)
        /// </summary>
        /// <param name="index">the particle index</param>
        public void Execute(int index)
        {
            // calc air resistance
            var airResistance = -airDrag * (nowPosition[index] - previousPosition[index]) / dt;

            // calc force apply.
            var forceApply = gravity + globalForce + airResistance + forceExt[index];
            nextPosition[index] = nowPosition[index]
                                  + (1 - damping) * (nowPosition[index] - previousPosition[index])
                                  + forceApply / mass[index] * (dt * dt);
        }
    }
}