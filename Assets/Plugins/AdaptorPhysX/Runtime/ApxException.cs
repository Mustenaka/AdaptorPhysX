using System;

namespace APEX.Native
{
    public sealed class ApxException : InvalidOperationException
    {
        public ApxException(ApxResult result, string operation)
            : base($"{operation} failed with {result} ({(int)result}).")
        {
            Result = result;
            Operation = operation;
        }

        public ApxResult Result { get; }

        public string Operation { get; }

        internal static void ThrowIfFailed(ApxResult result, string operation)
        {
            if (result != ApxResult.Success)
            {
                throw new ApxException(result, operation);
            }
        }
    }
}
