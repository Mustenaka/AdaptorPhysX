using System;
using System.Runtime.InteropServices;

namespace APEX.Native
{
    internal static class NativeMethods
    {
        private const string LibraryName = "adaptorphysx";

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetAbiVersion",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetAbiVersion(out uint outMajor, out uint outMinor);

        [DllImport(
            LibraryName,
            EntryPoint = "apxCreateWorld",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxCreateWorld(
            in ApxWorldDesc description,
            out IntPtr outWorld);

        [DllImport(
            LibraryName,
            EntryPoint = "apxDestroyWorld",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxDestroyWorld(IntPtr world);

        [DllImport(
            LibraryName,
            EntryPoint = "apxAddParticles",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxAddParticles(
            ApxWorldSafeHandle world,
            [In] ApxParticleDesc[] particles,
            uint count,
            out uint outFirstParticleId);

        [DllImport(
            LibraryName,
            EntryPoint = "apxAddDistanceConstraints",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxAddDistanceConstraints(
            ApxWorldSafeHandle world,
            [In] ApxDistanceConstraintDesc[] constraints,
            uint count,
            out uint outFirstConstraintId);

        [DllImport(
            LibraryName,
            EntryPoint = "apxAddClothDistanceConstraints",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxAddClothDistanceConstraints(
            ApxWorldSafeHandle world,
            [In] ApxClothDistanceConstraintDesc[] constraints,
            uint count,
            out uint outFirstConstraintId);

        [DllImport(
            LibraryName,
            EntryPoint = "apxAddBendConstraints",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxAddBendConstraints(
            ApxWorldSafeHandle world,
            [In] ApxBendConstraintDesc[] constraints,
            uint count,
            out uint outFirstConstraintId);

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetBrokenClothDistanceConstraintIds",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetBrokenClothDistanceConstraintIds(
            ApxWorldSafeHandle world,
            [Out] uint[] outConstraintIds,
            uint capacity,
            out uint outCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxSetColliderProxies",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxSetColliderProxies(
            ApxWorldSafeHandle world,
            [In] ApxColliderProxy[] proxies,
            uint count);

        [DllImport(
            LibraryName,
            EntryPoint = "apxStep",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxStep(
            ApxWorldSafeHandle world,
            float frameDeltaTime);

        [DllImport(
            LibraryName,
            EntryPoint = "apxMapParticleBuffer",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxMapParticleBuffer(
            ApxWorldSafeHandle world,
            out ApxBufferView outView);

        [DllImport(
            LibraryName,
            EntryPoint = "apxUnmapParticleBuffer",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxUnmapParticleBuffer(ApxWorldSafeHandle world);
    }
}
