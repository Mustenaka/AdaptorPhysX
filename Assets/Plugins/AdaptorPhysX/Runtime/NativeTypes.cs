using System;
using System.Runtime.InteropServices;

namespace APEX.Native
{
    public enum ApxResult : int
    {
        Success = 0,
        InvalidArgument = 1,
        InvalidHandle = 2,
        InvalidState = 3,
        CapacityExceeded = 4,
        OutOfMemory = 5,
        Unsupported = 6,
        NumericalFailure = 7,
        BackendError = 8,
        InternalError = 9,
        AlreadyMapped = 10,
        NotMapped = 11,
    }

    public enum ApxBackendKind : uint
    {
        Cpu = 0,
        Cuda = 1,
    }

    public enum ApxColliderProxyType : uint
    {
        Sphere = 0,
        Capsule = 1,
        Plane = 2,
    }

    public enum ApxClothDirection : uint
    {
        Warp = 0,
        Weft = 1,
        Shear = 2,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxVec3
    {
        public float X;
        public float Y;
        public float Z;

        public ApxVec3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxVec4
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public ApxVec4(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxWorldDesc
    {
        public const uint SizeInBytes = 48;

        public uint StructSize;
        public ApxBackendKind Backend;
        public float FixedTimeStep;
        public uint SolverIterations;
        public ApxVec3 Gravity;
        public float ParticleRadius;
        public uint InitialParticleCapacity;
        public uint Reserved0;
        public uint Reserved1;
        public uint Reserved2;

        public static ApxWorldDesc Create(
            ApxBackendKind backend,
            float fixedTimeStep,
            uint solverIterations,
            ApxVec3 gravity,
            float particleRadius = 0.0F,
            uint initialParticleCapacity = 0)
        {
            return new ApxWorldDesc
            {
                StructSize = SizeInBytes,
                Backend = backend,
                FixedTimeStep = fixedTimeStep,
                SolverIterations = solverIterations,
                Gravity = gravity,
                ParticleRadius = particleRadius,
                InitialParticleCapacity = initialParticleCapacity,
            };
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxParticleDesc
    {
        public ApxVec3 Position;
        public ApxVec3 Velocity;
        public float InverseMass;

        public ApxParticleDesc(ApxVec3 position, ApxVec3 velocity, float inverseMass)
        {
            Position = position;
            Velocity = velocity;
            InverseMass = inverseMass;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxDistanceConstraintDesc
    {
        public uint ParticleA;
        public uint ParticleB;
        public float RestLength;
        public float Compliance;

        public ApxDistanceConstraintDesc(
            uint particleA,
            uint particleB,
            float restLength,
            float compliance)
        {
            ParticleA = particleA;
            ParticleB = particleB;
            RestLength = restLength;
            Compliance = compliance;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxClothDistanceConstraintDesc
    {
        public uint ParticleA;
        public uint ParticleB;
        public float RestLength;
        public float Compliance;
        public float BreakThreshold;
        public ApxClothDirection Direction;

        public ApxClothDistanceConstraintDesc(
            uint particleA,
            uint particleB,
            float restLength,
            float compliance,
            float breakThreshold,
            ApxClothDirection direction)
        {
            ParticleA = particleA;
            ParticleB = particleB;
            RestLength = restLength;
            Compliance = compliance;
            BreakThreshold = breakThreshold;
            Direction = direction;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxBendConstraintDesc
    {
        public const uint NoSupportingClothConstraint = uint.MaxValue;

        public uint OppositeA;
        public uint OppositeB;
        public uint EdgeA;
        public uint EdgeB;
        public uint SupportingClothConstraint;
        public float Compliance;

        public ApxBendConstraintDesc(
            uint oppositeA,
            uint oppositeB,
            uint edgeA,
            uint edgeB,
            uint supportingClothConstraint,
            float compliance)
        {
            OppositeA = oppositeA;
            OppositeB = oppositeB;
            EdgeA = edgeA;
            EdgeB = edgeB;
            SupportingClothConstraint = supportingClothConstraint;
            Compliance = compliance;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxRenderVertexBindingDesc
    {
        public uint ParticleA;
        public uint ParticleB;
        public uint ParticleC;
        public float WeightA;
        public float WeightB;
        public float WeightC;

        public ApxRenderVertexBindingDesc(
            uint particleA,
            uint particleB,
            uint particleC,
            float weightA,
            float weightB,
            float weightC)
        {
            ParticleA = particleA;
            ParticleB = particleB;
            ParticleC = particleC;
            WeightA = weightA;
            WeightB = weightB;
            WeightC = weightC;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxCutQuery
    {
        public ApxVec3 Start;
        public ApxVec3 End;
        public ApxVec3 SideNormal;
        public float Radius;

        public ApxCutQuery(ApxVec3 start, ApxVec3 end, ApxVec3 sideNormal, float radius)
        {
            Start = start;
            End = end;
            SideNormal = sideNormal;
            Radius = radius;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxCutResult
    {
        public uint CutId;
        public uint CutConstraintCount;
        public uint SplitParticleCount;
        public uint FirstSplitParticleId;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxColliderProxy
    {
        public ApxColliderProxyType Type;
        public uint Reserved0;
        public uint Reserved1;
        public uint Reserved2;
        public ApxVec4 Data0;
        public ApxVec4 Data1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ApxBufferView
    {
        public IntPtr Data;
        public ulong SizeBytes;
        public uint ElementCount;
        public uint ElementStrideBytes;
    }
}
