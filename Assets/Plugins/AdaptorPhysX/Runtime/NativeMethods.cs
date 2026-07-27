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
            EntryPoint = "apxCreateWorldWithParticleCapacity",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxCreateWorldWithParticleCapacity(
            in ApxWorldDesc description,
            uint maximumParticleCapacity,
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
            EntryPoint = "apxSetRenderVertexBindings",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxSetRenderVertexBindings(
            ApxWorldSafeHandle world,
            [In] ApxRenderVertexBindingDesc[] bindings,
            uint count);

        [DllImport(
            LibraryName,
            EntryPoint = "apxBindRenderVertices",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxBindRenderVertices(
            ApxWorldSafeHandle world,
            [In] ApxVec3[] renderRestPositions,
            uint renderVertexCount,
            [In] uint[] simulationTriangleParticleIds,
            uint simulationTriangleCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetRenderVertexBindings",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetRenderVertexBindings(
            ApxWorldSafeHandle world,
            [Out] ApxRenderVertexBindingDesc[] outBindings,
            uint capacity,
            out uint outCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxSetRenderTriangles",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxSetRenderTriangles(
            ApxWorldSafeHandle world,
            [In] uint[] triangleVertexIds,
            uint triangleCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetRenderVertexPositions",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetRenderVertexPositions(
            ApxWorldSafeHandle world,
            IntPtr outPositions,
            uint capacity,
            out uint outCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetRenderVertexNormals",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetRenderVertexNormals(
            ApxWorldSafeHandle world,
            IntPtr outNormals,
            uint capacity,
            out uint outCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxCut",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxCut(
            ApxWorldSafeHandle world,
            in ApxCutQuery query,
            out ApxCutResult outResult);

        [DllImport(
            LibraryName,
            EntryPoint = "apxGetLastCutDetails",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxGetLastCutDetails(
            ApxWorldSafeHandle world,
            [Out] uint[] outConstraintIds,
            uint constraintCapacity,
            out uint outConstraintCount,
            [Out] uint[] outAffectedParticleIds,
            uint affectedParticleCapacity,
            out uint outAffectedParticleCount,
            [Out] uint[] outActivatedParticleIds,
            uint activatedParticleCapacity,
            out uint outActivatedParticleCount);

        [DllImport(
            LibraryName,
            EntryPoint = "apxSetKinematicTargets",
            CallingConvention = CallingConvention.Cdecl,
            ExactSpelling = true)]
        internal static extern ApxResult ApxSetKinematicTargets(
            ApxWorldSafeHandle world,
            [In] ApxKinematicTarget[] targets,
            uint count);

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
