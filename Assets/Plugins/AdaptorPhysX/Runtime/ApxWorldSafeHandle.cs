using System;
using Microsoft.Win32.SafeHandles;

namespace APEX.Native
{
    public sealed class ApxWorldSafeHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        internal ApxWorldSafeHandle()
            : base(true)
        {
        }

        internal ApxWorldSafeHandle(IntPtr world)
            : base(true)
        {
            SetHandle(world);
        }

        protected override bool ReleaseHandle()
        {
            try
            {
                return NativeMethods.ApxDestroyWorld(handle) == ApxResult.Success;
            }
            catch
            {
                // SafeHandle can run on the finalizer thread, where a missing
                // plugin or any interop failure must never escape.
                return false;
            }
        }
    }
}
