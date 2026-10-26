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
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
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
        /// <summary>Values of the distance-field shapes.</summary>
        [ReadOnly] public NativeArray<float> Fields;
        /// <summary>Collider impulses per (node block, shape); see <see cref="SoftBodySimulation.NodeBatch"/>.</summary>
        [NativeDisableParallelForRestriction] public NativeArray<ColliderImpulse> Impulses;
        public bool AccumulateImpulses;
        /// <summary>
        /// 1 for nodes held by a sticky collider: they touched it and have not been pulled further
        /// than its stick distance since. Updated only when a shape is sticky.
        /// </summary>
        public NativeArray<byte> Stuck;
        public bool Sticky;

        /// <summary>Contact with other soft bodies (see <see cref="BodyContacts"/>).</summary>
        public bool BodyContacts;
        [ReadOnly] public NativeArray<float> NodeSdf;
        [ReadOnly] public NativeArray<int> PartnerCandidate;
        [ReadOnly] public NativeArray<int> PartnerCandidateCount;
        [ReadOnly] public NativeArray<PartnerCell> PartnerCells;
        [ReadOnly] public NativeArray<float3> PartnerCorner;
        [ReadOnly] public NativeArray<float3> PartnerVelocity;
        [ReadOnly] public NativeArray<float> PartnerSdf;
        [ReadOnly] public NativeArray<float> PartnerWeight;
        /// <summary>Length of the whole step, to place a substep in time for the partners' motion.</summary>
        public float StepDt;
        /// <summary>Largest push out of another body per substep; deeper overlaps resolve over several substeps.</summary>
        public float MaxBodyPush;
        /// <summary>
        /// Per node, the part of its overlap with other bodies left to them in this step. The partner's
        /// copy does not move during the step, so without it each body would resolve the whole overlap
        /// instead of its share, and the two together twice that.
        /// </summary>
        public NativeArray<float> PartnerShare;

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
                if (BodyContacts)
                {
                    x = BodyContact(i, x);
                }
                bool held = false;
                for (int s = 0; s < Shapes.Length; s++)
                {
                    ColliderShape shape = Shapes[s];
                    // A dynamic body's pose is set substep by substep: A1 is the current one.
                    bool dynamic = shape.DynamicBody > 0;
                    float d = shape.Distance(x, dynamic ? 1.0f : FinishTime, Fields, out float3 n);
                    float3 before = x;
                    float pushed;
                    if (d >= Margin)
                    {
                        // Adhesion: a node that touched a sticky collider is pulled back towards its
                        // surface until it is further away than the stick distance.
                        if (!(shape.Stickiness > 0.0f) || Stuck[i] == 0 || d >= Margin + shape.StickDistance)
                        {
                            continue;
                        }
                        pushed = (d - Margin) * shape.Stickiness;
                        x -= n * pushed;
                        held = true;
                    }
                    else
                    {
                        pushed = Margin - d;
                        x += n * pushed;
                        held |= shape.Stickiness > 0.0f;
                    }
                    float3 relative = (x - Previous[i]) -
                                      (dynamic ? MaterialPointMotion(shape, x) : shape.Displacement(FinishTimePrev, FinishTime));
                    float3 tangent = relative - math.dot(relative, n) * n;
                    float slip = math.length(tangent);
                    float limit = shape.Friction * pushed;
                    x -= slip <= limit ? tangent : tangent * (limit / slip);
                    if (AccumulateImpulses)
                    {
                        AddImpulse(i, s, x, x - before, shape.A1);
                    }
                }
                if (Sticky)
                {
                    Stuck[i] = (byte)(held ? 1 : 0);
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

        /// <summary>Motion during the substep of the point of a dynamic body's collider now at <paramref name="x"/> (translation and rotation).</summary>
        private static float3 MaterialPointMotion(ColliderShape shape, float3 x)
        {
            float3 local = math.mul(math.conjugate(shape.R1), x - shape.A1);
            return x - (shape.A0 + math.mul(shape.R0, local));
        }

        /// <summary>Momentum m dx / h the node received from shape <paramref name="s"/>, added to the node's block.</summary>
        private void AddImpulse(int i, int s, float3 x, float3 dx, float3 origin)
        {
            float w = Weights[i];
            if (!(w > 0.0f))
            {
                return;
            }
            int index = i / SoftBodySimulation.NodeBatch * Shapes.Length + s;
            ColliderImpulse impulse = Impulses[index];
            impulse.Add(x, origin, dx * (1.0f / (w * Dt)), math.length(dx));
            Impulses[index] = impulse;
        }

        /// <summary>
        /// Pushes the node out of the other bodies' cells, taking its mass-weighted share, with
        /// friction against the partner's motion (see <see cref="AdaptorPhysX.BodyContacts"/>).
        /// </summary>
        private float3 BodyContact(int i, float3 x)
        {
            int count = PartnerCandidateCount[i];
            float wi = Weights[i];
            if (count == 0 || !(wi > 0.0f))
            {
                return x;
            }
            float depth = AdaptorPhysX.BodyContacts.Penetration(x, NodeSdf[i], i * AdaptorPhysX.BodyContacts.MaxCandidates,
                count, FinishTime * StepDt, PartnerCandidate, PartnerCells, PartnerCorner, PartnerVelocity, PartnerSdf,
                PartnerWeight, out float3 n, out float partnerWeight, out float3 partnerVelocity, out float friction);
            // The partner resolves its share of the overlap in its own step.
            depth -= PartnerShare[i];
            if (!(depth > 0.0f))
            {
                return x;
            }
            float share = wi / (wi + partnerWeight);
            float push = math.min(depth * share, MaxBodyPush);
            PartnerShare[i] += push * (1.0f - share) / share;
            x += n * push;
            float3 relative = (x - Previous[i]) - partnerVelocity * Dt;
            float3 tangent = relative - math.dot(relative, n) * n;
            float slip = math.length(tangent);
            float limit = friction * push;
            return x - (slip <= limit ? tangent : tangent * (limit / slip));
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

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct BoundsJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> Velocities;
        [ReadOnly] public NativeArray<byte> NodeReal;
        /// <summary>Node AABB (0, 1) and the largest speed of a real node (2, x).</summary>
        public NativeArray<float3> Bounds;

        public void Execute()
        {
            float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
            float speedSq = 0.0f;
            for (int i = 0; i < Positions.Length; i++)
            {
                lo = math.min(lo, Positions[i]);
                hi = math.max(hi, Positions[i]);
                if (NodeReal[i] != 0)
                {
                    speedSq = math.max(speedSq, math.lengthsq(Velocities[i]));
                }
            }
            Bounds[0] = lo;
            Bounds[1] = hi;
            Bounds[2] = new float3(math.sqrt(speedSq), 0.0f, 0.0f);
        }
    }

    /// <summary>Effective inverse masses: kinematic nodes are infinitely heavy for the constraints.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct EffectiveMassJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float> InverseMass;
        [ReadOnly] public NativeArray<int> KinematicId;
        /// <summary>Density the masses were built with over the current density.</summary>
        public float Scale;
        public NativeArray<float> Weights;

        public void Execute(int i) => Weights[i] = KinematicId[i] >= 0 ? 0.0f : InverseMass[i] * Scale;
    }

    /// <summary>Four 3-vectors, one per SIMD lane (structure of arrays).</summary>
    internal struct Lanes3
    {
        public float4 X, Y, Z;

        public Lanes3(float4 x, float4 y, float4 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Lanes3 operator +(Lanes3 a, Lanes3 b) => new Lanes3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Lanes3 operator -(Lanes3 a, Lanes3 b) => new Lanes3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Lanes3 operator -(Lanes3 a) => new Lanes3(-a.X, -a.Y, -a.Z);
        public static Lanes3 operator *(Lanes3 a, float4 s) => new Lanes3(a.X * s, a.Y * s, a.Z * s);

        public static float4 Dot(Lanes3 a, Lanes3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static Lanes3 Cross(Lanes3 a, Lanes3 b) =>
            new Lanes3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
    }

    /// <summary>Four 3x3 matrices (columns C0..C2), one per SIMD lane.</summary>
    internal struct LaneMatrix
    {
        public Lanes3 C0, C1, C2;

        public LaneMatrix(Lanes3 c0, Lanes3 c1, Lanes3 c2)
        {
            C0 = c0;
            C1 = c1;
            C2 = c2;
        }

        public static LaneMatrix operator -(LaneMatrix a, LaneMatrix b) => new LaneMatrix(a.C0 - b.C0, a.C1 - b.C1, a.C2 - b.C2);

        public static LaneMatrix Gather(float3x3 a, float3x3 b, float3x3 c, float3x3 d) => new LaneMatrix(
            new Lanes3(new float4(a.c0.x, b.c0.x, c.c0.x, d.c0.x), new float4(a.c0.y, b.c0.y, c.c0.y, d.c0.y), new float4(a.c0.z, b.c0.z, c.c0.z, d.c0.z)),
            new Lanes3(new float4(a.c1.x, b.c1.x, c.c1.x, d.c1.x), new float4(a.c1.y, b.c1.y, c.c1.y, d.c1.y), new float4(a.c1.z, b.c1.z, c.c1.z, d.c1.z)),
            new Lanes3(new float4(a.c2.x, b.c2.x, c.c2.x, d.c2.x), new float4(a.c2.y, b.c2.y, c.c2.y, d.c2.y), new float4(a.c2.z, b.c2.z, c.c2.z, d.c2.z)));

        /// <summary>a * b.</summary>
        public static LaneMatrix Mul(LaneMatrix a, LaneMatrix b) => new LaneMatrix(Mul(a, b.C0), Mul(a, b.C1), Mul(a, b.C2));

        /// <summary>Uniform a times per-lane b.</summary>
        public static LaneMatrix Mul(float3x3 a, LaneMatrix b) => new LaneMatrix(Mul(a, b.C0), Mul(a, b.C1), Mul(a, b.C2));

        private static Lanes3 Mul(LaneMatrix a, Lanes3 v) => a.C0 * v.X + a.C1 * v.Y + a.C2 * v.Z;

        private static Lanes3 Mul(float3x3 a, Lanes3 v) => new Lanes3(
            a.c0.x * v.X + a.c1.x * v.Y + a.c2.x * v.Z,
            a.c0.y * v.X + a.c1.y * v.Y + a.c2.y * v.Z,
            a.c0.z * v.X + a.c1.z * v.Y + a.c2.z * v.Z);

        public LaneMatrix Transpose() => new LaneMatrix(
            new Lanes3(C0.X, C1.X, C2.X), new Lanes3(C0.Y, C1.Y, C2.Y), new Lanes3(C0.Z, C1.Z, C2.Z));

        /// <summary>Rotation matrices of unit quaternions (x, y, z, w), as float3x3(quaternion).</summary>
        public static LaneMatrix Rotation(float4 x, float4 y, float4 z, float4 w)
        {
            float4 xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;
            return new LaneMatrix(
                new Lanes3(1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)),
                new Lanes3(2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)),
                new Lanes3(2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)));
        }
    }

    /// <summary>
    /// Co-rotational XPBD elasticity for all tetrahedra of the cell slices of one colour:
    /// a deviatoric constraint C_D = |F - R| (R the rotation of F) and a volume constraint
    /// C_H = det F - 1, which linearise to isotropic linear elasticity with Lame mu and lambda.
    /// Both are zero at rest. (The stable Neo-Hookean pair, C_D = |F| and C_H = det F - gamma,
    /// balances two large opposing projections at rest and diverges once mu dt^2 / (rho h^2)
    /// exceeds about one, i.e. for stiff skin or fine lattices.) Same-colour slices share no
    /// grid point, so their nodes are disjoint and the colour runs in parallel without atomics.
    /// Uncut cells are solved four at a time with one cell per SIMD lane: the scalar float3 code
    /// spends most of its time on shuffles and horizontal adds, the lane form has none.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true, FloatMode = FloatMode.Fast)]
    internal struct SolveCellsJob : IJobParallelFor
    {
        [NativeDisableParallelForRestriction] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<int> SliceCells;
        /// <summary>Four cells of one slice without a shared grid point, solved in SIMD lanes.</summary>
        [ReadOnly] public NativeArray<int4> SliceGroups;
        /// <summary>Per slice: lane groups (start, count) and single cells (start, count).</summary>
        [ReadOnly] public NativeArray<int4> Slices;
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<float3x3> TetInverseRest;
        [ReadOnly] public NativeArray<float> TetVolume;
        [ReadOnly] public NativeArray<float3x3> CellRest;
        [ReadOnly] public NativeArray<float> CellScale;

        /// <summary>Rotation of every tetrahedron (copy * 6 + tet), warm start for the polar decomposition.</summary>
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> TetRotation;

        public int SliceOffset;
        /// <summary>1 / (lambda dt^2) and 1 / (2 mu dt^2): compliance per unit volume.</summary>
        public float HydrostaticCompliance;
        public float DeviatoricCompliance;

        public void Execute(int index)
        {
            int4 slice = Slices[SliceOffset + index];
            for (int g = slice.x; g < slice.x + slice.y; g++)
            {
                int4 cells = SliceGroups[g];
                int4 copies = new int4(CellCopyRange[cells.x].y, CellCopyRange[cells.y].y,
                    CellCopyRange[cells.z].y, CellCopyRange[cells.w].y);
                if (math.all(copies == 1))
                {
                    SolveGroup(cells);
                    continue;
                }
                // A cut cell has several copies: solve the four cells one by one.
                SolveCell(cells.x);
                SolveCell(cells.y);
                SolveCell(cells.z);
                SolveCell(cells.w);
            }
            for (int c = slice.z; c < slice.z + slice.w; c++)
            {
                SolveCell(SliceCells[c]);
            }
        }

        private void SolveCell(int cell)
        {
            int2 range = CellCopyRange[cell];
            for (int q = range.x; q < range.x + range.y; q++)
            {
                SolveCopy(cell, q);
            }
        }

        /// <summary>The six tetrahedra of four uncut cells without a shared node, one cell per lane.</summary>
        private void SolveGroup(int4 cells)
        {
            int4 copy = new int4(CellCopyRange[cells.x].x, CellCopyRange[cells.y].x,
                CellCopyRange[cells.z].x, CellCopyRange[cells.w].x);
            int4 b = copy * Lattice.CornerCount;
            int4 n0 = Corners(b, 0), n1 = Corners(b, 1), n2 = Corners(b, 2), n3 = Corners(b, 3);
            int4 n4 = Corners(b, 4), n5 = Corners(b, 5), n6 = Corners(b, 6), n7 = Corners(b, 7);
            Lanes3 x0 = Load(n0), x1 = Load(n1), x2 = Load(n2), x3 = Load(n3);
            Lanes3 x4 = Load(n4), x5 = Load(n5), x6 = Load(n6), x7 = Load(n7);
            float4 w0 = LoadWeights(n0), w1 = LoadWeights(n1), w2 = LoadWeights(n2), w3 = LoadWeights(n3);
            float4 w4 = LoadWeights(n4), w5 = LoadWeights(n5), w6 = LoadWeights(n6), w7 = LoadWeights(n7);
            if (!math.any(w0 + w1 + w2 + w3 + w4 + w5 + w6 + w7 > 0.0f))
            {
                // Four pinned cells: nothing moves (the scalar path skips such tetrahedra too).
                return;
            }
            LaneMatrix restScale = LaneMatrix.Gather(CellRest[cells.x], CellRest[cells.y], CellRest[cells.z], CellRest[cells.w]);
            float4 volumeScale = new float4(
                CellScale[cells.x] * Copies[copy.x].Weight, CellScale[cells.y] * Copies[copy.y].Weight,
                CellScale[cells.z] * Copies[copy.z].Weight, CellScale[cells.w] * Copies[copy.w].Weight);
            int4 r = copy * Lattice.TetsPerCell;

            SolveTets(ref x0, ref x1, ref x3, ref x7, w0, w1, w3, w7, 0, restScale, volumeScale, r);
            SolveTets(ref x0, ref x1, ref x7, ref x5, w0, w1, w7, w5, 1, restScale, volumeScale, r + 1);
            SolveTets(ref x0, ref x2, ref x7, ref x3, w0, w2, w7, w3, 2, restScale, volumeScale, r + 2);
            SolveTets(ref x0, ref x2, ref x6, ref x7, w0, w2, w6, w7, 3, restScale, volumeScale, r + 3);
            SolveTets(ref x0, ref x4, ref x5, ref x7, w0, w4, w5, w7, 4, restScale, volumeScale, r + 4);
            SolveTets(ref x0, ref x4, ref x7, ref x6, w0, w4, w7, w6, 5, restScale, volumeScale, r + 5);

            Store(n0, x0);
            Store(n1, x1);
            Store(n2, x2);
            Store(n3, x3);
            Store(n4, x4);
            Store(n5, x5);
            Store(n6, x6);
            Store(n7, x7);
        }

        private int4 Corners(int4 b, int k) =>
            new int4(CopyNodes[b.x + k], CopyNodes[b.y + k], CopyNodes[b.z + k], CopyNodes[b.w + k]);

        private Lanes3 Load(int4 n)
        {
            float3 a = Positions[n.x], b = Positions[n.y], c = Positions[n.z], d = Positions[n.w];
            return new Lanes3(new float4(a.x, b.x, c.x, d.x), new float4(a.y, b.y, c.y, d.y), new float4(a.z, b.z, c.z, d.z));
        }

        private float4 LoadWeights(int4 n) => new float4(Weights[n.x], Weights[n.y], Weights[n.z], Weights[n.w]);

        private void Store(int4 n, Lanes3 v)
        {
            Positions[n.x] = new float3(v.X.x, v.Y.x, v.Z.x);
            Positions[n.y] = new float3(v.X.y, v.Y.y, v.Z.y);
            Positions[n.z] = new float3(v.X.z, v.Y.z, v.Z.z);
            Positions[n.w] = new float3(v.X.w, v.Y.w, v.Z.w);
        }

        /// <summary>Lane form of <see cref="SolveTet"/>: the same constraints, branches turned into selects.</summary>
        private void SolveTets(ref Lanes3 x0, ref Lanes3 x1, ref Lanes3 x2, ref Lanes3 x3,
            float4 w0, float4 w1, float4 w2, float4 w3, int tet, LaneMatrix restScale, float4 volumeScale, int4 rotationIndex)
        {
            LaneMatrix inverseRest = LaneMatrix.Mul(TetInverseRest[tet], restScale);
            float4 volume = TetVolume[tet] * volumeScale;
            LaneMatrix f = LaneMatrix.Mul(new LaneMatrix(x1 - x0, x2 - x0, x3 - x0), inverseRest);
            LaneMatrix restT = inverseRest.Transpose();

            float4 q0 = TetRotation[rotationIndex.x].value, q1 = TetRotation[rotationIndex.y].value;
            float4 q2 = TetRotation[rotationIndex.z].value, q3 = TetRotation[rotationIndex.w].value;
            float4 qx = new float4(q0.x, q1.x, q2.x, q3.x), qy = new float4(q0.y, q1.y, q2.y, q3.y);
            float4 qz = new float4(q0.z, q1.z, q2.z, q3.z), qw = new float4(q0.w, q1.w, q2.w, q3.w);
            LaneMatrix rot = LaneMatrix.Rotation(qx, qy, qz, qw);
            Lanes3 omega = Lanes3.Cross(rot.C0, f.C0) + Lanes3.Cross(rot.C1, f.C1) + Lanes3.Cross(rot.C2, f.C2);
            omega *= 1.0f / (math.abs(Lanes3.Dot(rot.C0, f.C0) + Lanes3.Dot(rot.C1, f.C1) + Lanes3.Dot(rot.C2, f.C2)) + 1e-9f);
            float4 angleSq = Lanes3.Dot(omega, omega);
            bool4 turn = angleSq > 1e-14f;
            omega *= math.min(1.0f, math.rsqrt(math.max(angleSq, 1e-14f)) * 0.5f);
            float4 nx = qx + 0.5f * (omega.X * qw + omega.Y * qz - omega.Z * qy);
            float4 ny = qy + 0.5f * (omega.Y * qw + omega.Z * qx - omega.X * qz);
            float4 nz = qz + 0.5f * (omega.Z * qw + omega.X * qy - omega.Y * qx);
            float4 nw = qw - 0.5f * (omega.X * qx + omega.Y * qy + omega.Z * qz);
            float4 inverseNorm = math.rsqrt(nx * nx + ny * ny + nz * nz + nw * nw);
            qx = math.select(qx, nx * inverseNorm, turn);
            qy = math.select(qy, ny * inverseNorm, turn);
            qz = math.select(qz, nz * inverseNorm, turn);
            qw = math.select(qw, nw * inverseNorm, turn);
            TetRotation[rotationIndex.x] = new quaternion(qx.x, qy.x, qz.x, qw.x);
            TetRotation[rotationIndex.y] = new quaternion(qx.y, qy.y, qz.y, qw.y);
            TetRotation[rotationIndex.z] = new quaternion(qx.z, qy.z, qz.z, qw.z);
            TetRotation[rotationIndex.w] = new quaternion(qx.w, qy.w, qz.w, qw.w);
            rot = LaneMatrix.Rotation(qx, qy, qz, qw);

            var cof = new LaneMatrix(Lanes3.Cross(f.C1, f.C2), Lanes3.Cross(f.C2, f.C0), Lanes3.Cross(f.C0, f.C1));
            float4 cH = Lanes3.Dot(f.C0, cof.C0) - 1.0f;
            LaneMatrix gH = LaneMatrix.Mul(cof, restT);
            Lanes3 gH0 = -(gH.C0 + gH.C1 + gH.C2);

            LaneMatrix diff = f - rot;
            float4 cD2 = Lanes3.Dot(diff.C0, diff.C0) + Lanes3.Dot(diff.C1, diff.C1) + Lanes3.Dot(diff.C2, diff.C2);
            LaneMatrix gD = LaneMatrix.Mul(diff, restT);
            Lanes3 gD0 = -(gD.C0 + gD.C1 + gD.C2);

            float4 alphaH = HydrostaticCompliance / volume;
            float4 alphaD = DeviatoricCompliance / volume;
            float4 a11 = w0 * Lanes3.Dot(gH0, gH0) + w1 * Lanes3.Dot(gH.C0, gH.C0) + w2 * Lanes3.Dot(gH.C1, gH.C1) +
                         w3 * Lanes3.Dot(gH.C2, gH.C2) + alphaH;
            float4 a22 = w0 * Lanes3.Dot(gD0, gD0) + w1 * Lanes3.Dot(gD.C0, gD.C0) + w2 * Lanes3.Dot(gD.C1, gD.C1) +
                         w3 * Lanes3.Dot(gD.C2, gD.C2) + alphaD * cD2;
            float4 a12 = w0 * Lanes3.Dot(gH0, gD0) + w1 * Lanes3.Dot(gH.C0, gD.C0) + w2 * Lanes3.Dot(gH.C1, gD.C1) +
                         w3 * Lanes3.Dot(gH.C2, gD.C2);
            float4 det = a11 * a22 - a12 * a12;
            bool4 coupled = (det > 0.0f) & (cD2 > 1e-14f);
            a12 = math.select(0.0f, a12, coupled);
            a22 = math.select(1.0f, a22, coupled);
            det = math.select(a11, det, coupled);
            cD2 = math.select(0.0f, cD2, coupled);
            float4 inverseDet = math.select(0.0f, 1.0f / det, det > 0.0f);
            float4 dH = (-cH * a22 + cD2 * a12) * inverseDet;
            float4 dD = (-cD2 * a11 + cH * a12) * inverseDet;

            x0 += (gH0 * dH + gD0 * dD) * w0;
            x1 += (gH.C0 * dH + gD.C0 * dD) * w1;
            x2 += (gH.C1 * dH + gD.C1 * dD) * w2;
            x3 += (gH.C2 * dH + gD.C2 * dD) * w3;
        }

        /// <summary>
        /// The six tetrahedra of one cell copy. The eight corners are read once, updated in
        /// registers and written back once: consecutive tetrahedra share the 0-7 diagonal, and
        /// passing their positions through memory stalls on store-to-load forwarding.
        /// </summary>
        private void SolveCopy(int cell, int copy)
        {
            int b = copy * Lattice.CornerCount;
            int n0 = CopyNodes[b], n1 = CopyNodes[b + 1], n2 = CopyNodes[b + 2], n3 = CopyNodes[b + 3];
            int n4 = CopyNodes[b + 4], n5 = CopyNodes[b + 5], n6 = CopyNodes[b + 6], n7 = CopyNodes[b + 7];
            float3 x0 = Positions[n0], x1 = Positions[n1], x2 = Positions[n2], x3 = Positions[n3];
            float3 x4 = Positions[n4], x5 = Positions[n5], x6 = Positions[n6], x7 = Positions[n7];
            float w0 = Weights[n0], w1 = Weights[n1], w2 = Weights[n2], w3 = Weights[n3];
            float w4 = Weights[n4], w5 = Weights[n5], w6 = Weights[n6], w7 = Weights[n7];
            // Pre-strained rest shape: Dm -> S Dm, so Dm^-1 -> Dm^-1 S^-1 and V -> det(S) V. The
            // stiffness multiplier m scales the compliance as 1 / (m V), so it is folded into the
            // volume passed to the solve.
            float3x3 restScale = CellRest[cell];
            float volumeScale = CellScale[cell] * Copies[copy].Weight;
            int r = copy * Lattice.TetsPerCell;

            // Kuhn split around the 0-7 diagonal, as in Lattice.TetCorners (the same for both parities).
            SolveTet(ref x0, ref x1, ref x3, ref x7, w0, w1, w3, w7, 0, restScale, volumeScale, r);
            SolveTet(ref x0, ref x1, ref x7, ref x5, w0, w1, w7, w5, 1, restScale, volumeScale, r + 1);
            SolveTet(ref x0, ref x2, ref x7, ref x3, w0, w2, w7, w3, 2, restScale, volumeScale, r + 2);
            SolveTet(ref x0, ref x2, ref x6, ref x7, w0, w2, w6, w7, 3, restScale, volumeScale, r + 3);
            SolveTet(ref x0, ref x4, ref x5, ref x7, w0, w4, w5, w7, 4, restScale, volumeScale, r + 4);
            SolveTet(ref x0, ref x4, ref x7, ref x6, w0, w4, w7, w6, 5, restScale, volumeScale, r + 5);

            Positions[n0] = x0;
            Positions[n1] = x1;
            Positions[n2] = x2;
            Positions[n3] = x3;
            Positions[n4] = x4;
            Positions[n5] = x5;
            Positions[n6] = x6;
            Positions[n7] = x7;
        }

        private void SolveTet(ref float3 x0, ref float3 x1, ref float3 x2, ref float3 x3,
            float w0, float w1, float w2, float w3, int tet, float3x3 restScale, float volumeScale, int rotationIndex)
        {
            if (w0 + w1 + w2 + w3 <= 0.0f)
            {
                return;
            }
            float3x3 inverseRest = math.mul(TetInverseRest[tet], restScale);
            float volume = TetVolume[tet] * volumeScale;
            float3x3 f = math.mul(new float3x3(x1 - x0, x2 - x0, x3 - x0), inverseRest);
            float3x3 restT = math.transpose(inverseRest);

            // Rotation of F: one step of Mueller et al. (2016) per substep, warm-started from the
            // previous substep. The projection must use the updated rotation: with a lagged one a
            // stiff tetrahedron is driven to the old rotation while the next substep starts from
            // the new one, and the alternating target oscillates. The per-step correction is small,
            // so the quaternion is advanced to first order (q += 0.5 (0, w) q) instead of through sin/cos.
            quaternion rotation = TetRotation[rotationIndex];
            float3x3 rot = new float3x3(rotation);
            float3 omega = math.cross(rot.c0, f.c0) + math.cross(rot.c1, f.c1) + math.cross(rot.c2, f.c2);
            omega *= 1.0f / (math.abs(math.dot(rot.c0, f.c0) + math.dot(rot.c1, f.c1) + math.dot(rot.c2, f.c2)) + 1e-9f);
            float angleSq = math.lengthsq(omega);
            if (angleSq > 1e-14f)
            {
                omega *= math.min(1.0f, math.rsqrt(angleSq) * 0.5f);
                float4 q = rotation.value;
                float4 dq = 0.5f * new float4(
                    omega.x * q.w + omega.y * q.z - omega.z * q.y,
                    omega.y * q.w + omega.z * q.x - omega.x * q.z,
                    omega.z * q.w + omega.x * q.y - omega.y * q.x,
                    -(omega.x * q.x + omega.y * q.y + omega.z * q.z));
                rotation = new quaternion(math.normalize(q + dq));
                TetRotation[rotationIndex] = rotation;
                rot = new float3x3(rotation);
            }

            // Volume: C_H = det(F) - 1, dC/dF = cof(F).
            float3x3 cof = new float3x3(math.cross(f.c1, f.c2), math.cross(f.c2, f.c0), math.cross(f.c0, f.c1));
            float cH = math.dot(f.c0, cof.c0) - 1.0f;
            float3x3 gH = math.mul(cof, restT);
            float3 gH0 = -(gH.c0 + gH.c1 + gH.c2);

            // Deviatoric: C_D = |F - R|_F with R held fixed, so dC/dx = G / C_D for G the gradient
            // of |F - R|^2 / 2. Solving for mu = lambda_D / C_D instead of lambda_D multiplies the
            // second row of the XPBD system by C_D and removes the square root and the division.
            float3x3 diff = f - rot;
            float cD2 = math.dot(diff.c0, diff.c0) + math.dot(diff.c1, diff.c1) + math.dot(diff.c2, diff.c2);
            float3x3 gD = math.mul(diff, restT);
            float3 gD0 = -(gD.c0 + gD.c1 + gD.c2);

            float alphaH = HydrostaticCompliance / volume;
            float alphaD = DeviatoricCompliance / volume;
            float a11 = w0 * math.lengthsq(gH0) + w1 * math.lengthsq(gH.c0) + w2 * math.lengthsq(gH.c1) +
                        w3 * math.lengthsq(gH.c2) + alphaH;
            float a22 = w0 * math.lengthsq(gD0) + w1 * math.lengthsq(gD.c0) + w2 * math.lengthsq(gD.c1) +
                        w3 * math.lengthsq(gD.c2) + alphaD * cD2;
            float a12 = w0 * math.dot(gH0, gD0) + w1 * math.dot(gH.c0, gD.c0) + w2 * math.dot(gH.c1, gD.c1) +
                        w3 * math.dot(gH.c2, gD.c2);
            float det = a11 * a22 - a12 * a12;
            if (!(det > 0.0f) || cD2 <= 1e-14f)
            {
                // No deviatoric strain (or a degenerate system): project the volume constraint alone.
                if (!(a11 > 0.0f))
                {
                    return;
                }
                a12 = 0.0f;
                a22 = 1.0f;
                det = a11;
                cD2 = 0.0f;
            }
            // Both constraints solved together as one 2x2 system in (lambda_H, mu).
            float inverseDet = 1.0f / det;
            float dH = (-cH * a22 + cD2 * a12) * inverseDet;
            float dD = (-cD2 * a11 + cH * a12) * inverseDet;

            x0 += w0 * (gH0 * dH + gD0 * dD);
            x1 += w1 * (gH.c0 * dH + gD.c0 * dD);
            x2 += w2 * (gH.c1 * dH + gD.c1 * dD);
            x3 += w3 * (gH.c2 * dH + gD.c2 * dD);
        }
    }
}
