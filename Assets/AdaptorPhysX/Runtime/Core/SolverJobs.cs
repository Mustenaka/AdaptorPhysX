using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>One lattice cell copy (virtual node algorithm): its cell and the corners it owns.</summary>
    public struct CellCopy
    {
        public int Cell;
        /// <summary>Corners that carry real material in this copy.</summary>
        public byte Mask;
        /// <summary>Material weight: cell fill fraction times the owned corner fraction.</summary>
        public float Weight;
    }

    /// <summary>
    /// Per-node work between constraint passes: finish the previous substep
    /// (collisions, friction, velocity) and/or predict the next one.
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    internal struct NodeStepJob : IJobParallelFor
    {
        public NativeArray<float3> Positions;
        public NativeArray<float3> Previous;
        public NativeArray<float3> Velocities;
        [ReadOnly] public NativeArray<int> KinematicId;
        [ReadOnly] public NativeArray<float3> KinematicLocal;
        [ReadOnly] public NativeArray<float4x4> FramesStart;
        [ReadOnly] public NativeArray<float4x4> FramesEnd;
        [ReadOnly] public NativeArray<ColliderShape> Shapes;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<float3> Snapshot;
        [ReadOnly] public NativeArray<int> ContactPartner;
        [ReadOnly] public NativeArray<float> ContactDistance;
        [ReadOnly] public NativeArray<int> ContactCount;
        public bool Contacts;

        public bool Finish;
        public bool Predict;
        /// <summary>Normalised frame time of the substep being finished (end) and predicted (end).</summary>
        public float FinishTime;
        public float FinishTimePrev;
        public float PredictTime;
        public float Dt;
        public float3 Gravity;
        public float DampingFactor;
        public float Margin;

        public void Execute(int i)
        {
            int kin = KinematicId[i];
            if (Finish && kin < 0)
            {
                float3 x = Positions[i];
                if (Contacts)
                {
                    x += ContactCorrection(i);
                }
                for (int s = 0; s < Shapes.Length; s++)
                {
                    ColliderShape shape = Shapes[s];
                    float d = shape.Distance(x, FinishTime, out float3 n);
                    if (d >= Margin)
                    {
                        continue;
                    }
                    float penetration = Margin - d;
                    x += n * penetration;
                    float3 relative = (x - Previous[i]) - shape.Displacement(FinishTimePrev, FinishTime);
                    float3 tangent = relative - math.dot(relative, n) * n;
                    float slip = math.length(tangent);
                    float limit = shape.Friction * penetration;
                    x -= slip <= limit ? tangent : tangent * (limit / slip);
                }
                Positions[i] = x;
                Velocities[i] = (x - Previous[i]) / Dt;
            }

            if (!Predict)
            {
                return;
            }
            if (kin >= 0)
            {
                float3 local = KinematicLocal[i];
                float3 start = math.transform(FramesStart[kin], local);
                float3 end = math.transform(FramesEnd[kin], local);
                Previous[i] = Positions[i];
                Positions[i] = math.lerp(start, end, PredictTime);
                Velocities[i] = (Positions[i] - Previous[i]) / Dt;
                return;
            }
            float3 v = (Velocities[i] + Gravity * Dt) * DampingFactor;
            Previous[i] = Positions[i];
            Positions[i] = Positions[i] + v * Dt;
            Velocities[i] = v;
        }

        /// <summary>Jacobi projection of the node's piece contacts against the post-solve snapshot.</summary>
        private float3 ContactCorrection(int i)
        {
            int count = ContactCount[i];
            float wi = Weights[i];
            if (count == 0 || wi <= 0.0f)
            {
                return float3.zero;
            }
            float3 xi = Snapshot[i];
            float3 delta = float3.zero;
            for (int k = 0; k < count; k++)
            {
                int j = ContactPartner[i * AdaptorPhysX.Contacts.MaxPerNode + k];
                float3 d = xi - Snapshot[j];
                float length = math.length(d);
                float d0 = ContactDistance[i * AdaptorPhysX.Contacts.MaxPerNode + k];
                if (length >= d0 || length < 1e-9f)
                {
                    continue;
                }
                delta += d / length * ((d0 - length) * wi / (wi + Weights[j]));
            }
            return 0.8f * delta;
        }
    }

    [BurstCompile(CompileSynchronously = true)]
    internal struct BoundsJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        public NativeArray<float3> Bounds;

        public void Execute()
        {
            float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
            for (int i = 0; i < Positions.Length; i++)
            {
                lo = math.min(lo, Positions[i]);
                hi = math.max(hi, Positions[i]);
            }
            Bounds[0] = lo;
            Bounds[1] = hi;
        }
    }

    /// <summary>Effective inverse masses: kinematic nodes are infinitely heavy for the constraints.</summary>
    [BurstCompile(CompileSynchronously = true)]
    internal struct EffectiveMassJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> InverseMass;
        [ReadOnly] public NativeArray<int> KinematicId;
        public NativeArray<float> Weights;

        public void Execute(int i) => Weights[i] = KinematicId[i] >= 0 ? 0.0f : InverseMass[i];
    }

    /// <summary>
    /// Stable Neo-Hookean XPBD (Macklin &amp; Müller 2021) for all tetrahedra of the cell
    /// columns of one colour. Same-colour columns share no grid point, so their nodes are
    /// disjoint and the colour runs in parallel without atomics.
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    internal struct SolveCellsJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<int> ColumnCells;
        [ReadOnly] public NativeArray<int2> Columns;
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int> TetCorners;
        [ReadOnly] public NativeArray<float3x3> TetInverseRest;
        [ReadOnly] public NativeArray<float> TetVolume;

        public int ColumnOffset;
        /// <summary>1 / (lambda dt^2) and 1 / (mu dt^2): compliance per unit volume.</summary>
        public float HydrostaticCompliance;
        public float DeviatoricCompliance;
        public float Gamma;

        public void Execute(int index)
        {
            int2 column = Columns[ColumnOffset + index];
            for (int c = column.x; c < column.x + column.y; c++)
            {
                SolveCell(ColumnCells[c]);
            }
        }

        private void SolveCell(int cell)
        {
            int3 coord = CellCoord[cell];
            int parity = (coord.x + coord.y + coord.z) & 1;
            int2 range = CellCopyRange[cell];
            for (int q = range.x; q < range.x + range.y; q++)
            {
                float weight = Copies[q].Weight;
                int nodeBase = q * Lattice.CornerCount;
                for (int t = 0; t < Lattice.TetsPerCell; t++)
                {
                    int tet = parity * Lattice.TetsPerCell + t;
                    int i0 = CopyNodes[nodeBase + TetCorners[4 * tet]];
                    int i1 = CopyNodes[nodeBase + TetCorners[4 * tet + 1]];
                    int i2 = CopyNodes[nodeBase + TetCorners[4 * tet + 2]];
                    int i3 = CopyNodes[nodeBase + TetCorners[4 * tet + 3]];
                    SolveTet(i0, i1, i2, i3, TetInverseRest[tet], TetVolume[tet] * weight);
                }
            }
        }

        private void SolveTet(int i0, int i1, int i2, int i3, float3x3 inverseRest, float volume)
        {
            float w0 = Weights[i0], w1 = Weights[i1], w2 = Weights[i2], w3 = Weights[i3];
            if (w0 + w1 + w2 + w3 <= 0.0f)
            {
                return;
            }
            float3 x0 = Positions[i0], x1 = Positions[i1], x2 = Positions[i2], x3 = Positions[i3];
            float3x3 f = math.mul(new float3x3(x1 - x0, x2 - x0, x3 - x0), inverseRest);
            float3x3 restT = math.transpose(inverseRest);

            // Hydrostatic: C_H = det(F) - gamma, dC/dF = cof(F).
            float3x3 cof = new float3x3(math.cross(f.c1, f.c2), math.cross(f.c2, f.c0), math.cross(f.c0, f.c1));
            float cH = math.dot(f.c0, cof.c0) - Gamma;
            float3x3 gH = math.mul(cof, restT);
            float3 gH0 = -(gH.c0 + gH.c1 + gH.c2);

            // Deviatoric: C_D = |F|_F, dC/dF = F / |F|_F.
            float cD = math.sqrt(math.dot(f.c0, f.c0) + math.dot(f.c1, f.c1) + math.dot(f.c2, f.c2));
            float3x3 gD = cD > 1e-6f ? math.mul(f, restT) * (1.0f / cD) : float3x3.zero;
            float3 gD0 = -(gD.c0 + gD.c1 + gD.c2);

            float alphaH = HydrostaticCompliance / volume;
            float alphaD = DeviatoricCompliance / volume;
            float a11 = w0 * math.lengthsq(gH0) + w1 * math.lengthsq(gH.c0) + w2 * math.lengthsq(gH.c1) +
                        w3 * math.lengthsq(gH.c2) + alphaH;
            float a22 = w0 * math.lengthsq(gD0) + w1 * math.lengthsq(gD.c0) + w2 * math.lengthsq(gD.c1) +
                        w3 * math.lengthsq(gD.c2) + alphaD;
            float a12 = w0 * math.dot(gH0, gD0) + w1 * math.dot(gH.c0, gD.c0) + w2 * math.dot(gH.c1, gD.c1) +
                        w3 * math.dot(gH.c2, gD.c2);
            float det = a11 * a22 - a12 * a12;
            if (!(det > 0.0f) || cD <= 1e-6f)
            {
                // Degenerate deviatoric gradient: project the hydrostatic constraint alone.
                if (!(a11 > 0.0f))
                {
                    return;
                }
                a12 = 0.0f;
                a22 = 1.0f;
                det = a11;
                cD = 0.0f;
            }
            // Solve the coupled 2x2 system so the rest state is an exact fixed point.
            float dH = (-cH * a22 + cD * a12) / det;
            float dD = (-cD * a11 + cH * a12) / det;

            Positions[i0] = x0 + w0 * (gH0 * dH + gD0 * dD);
            Positions[i1] = x1 + w1 * (gH.c0 * dH + gD.c0 * dD);
            Positions[i2] = x2 + w2 * (gH.c1 * dH + gD.c1 * dD);
            Positions[i3] = x3 + w3 * (gH.c2 * dH + gD.c2 * dD);
        }
    }
}
