using Unity.Collections;
using Unity.Jobs;

namespace APEX.Tools.Extend
{
    /// <summary>
    /// Native Container Extend
    /// </summary>
    public static class NativeContainerExtend
    {
        /// <summary>
        /// Clean the NativeArray
        /// </summary>
        /// <param name="array"></param>
        /// <typeparam name="T"></typeparam>
        public static void Clean<T>(this NativeArray<T> array) where T : struct
        {
            for (var index = 0; index < array.Length; index++)
            {
                array[index] = default;
            }
        }
        
        /// <summary>
        /// Clean the NativeArray(Use Jobs)
        /// </summary>
        /// <param name="array"></param>
        /// <typeparam name="T"></typeparam>
        public static void MultithreadingClean<T>(this NativeArray<T> array) where T : struct
        {
            var job = new CleanJob<T> { Array = array };
            job.Schedule(array.Length, 64).Complete();
        }

        private struct CleanJob<T> : IJobParallelFor where T : struct
        {
            public NativeArray<T> Array;

            public void Execute(int index)
            {
                Array[index] = default;
            }
        }
    }
}