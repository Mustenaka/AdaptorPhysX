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
        /// <summary>Push of every node out of folds of its own piece in this substep (see <see cref="FoldContactJob"/>).</summary>
        [ReadOnly] public NativeArray<float3> FoldPush;
        public bool Folds;
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
        /// <summary>
        /// The partners' copies are refreshed between the substeps (see <see cref="SubstepGroup"/>): they
        /// hold each partner's solved positions of this substep, and the partner resolves its share of the
        /// same overlap in this substep, so no share is carried over.
        /// </summary>
        public bool PartnersSynced;

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
        /// <summary>A node this close to a sticky collider touches it (TODO MED-17).</summary>
        public float TouchDistance;
        /// <summary>
        /// Direction of the rest SDF gradient at each node (rest space), the first slot (copy * 8 + corner) of each node
        /// and the rotations of the tetrahedra, which turn the direction into the world (TODO MED-17).
        /// </summary>
        [ReadOnly] public NativeArray<float3> NodeOutward;
        [ReadOnly] public NativeArray<int> NodeSlot;
        [ReadOnly] public NativeArray<quaternion> TetRotation;

        public void Execute(int i)
        {
            int kin = KinematicId[i];
            if (Finish && kin < 0)
            {
                float3 x = Positions[i];
                // A node NodeSdf outside its own surface may enter a collider before the surface touches it (TODO MED-17),
                // by that distance along the collider's normal: the surface lies NodeSdf away along the rest SDF gradient,
                // so a node outside an edge or a corner, NodeSdf away from it diagonally, enters a floor by the distance to
                // the face. Nodes inside their surface keep the margin: the surface cells hold the surface there
                // (SolveCellsJob), and a larger margin would keep the layer between the node and the collider from
                // compressing.
                float outside = math.max(NodeSdf[i], 0.0f);
                float3 outward = float3.zero;
                bool turned = false;
                if (Contacts)
                {
                    x += ContactCorrection(i);
                }
                if (Folds)
                {
                    x += FoldPush[i];
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
                    float margin = Margin;
                    if (outside > 0.0f && d < Margin + (shape.Stickiness > 0.0f ? shape.StickDistance : 0.0f))
                    {
                        if (!turned)
                        {
                            // The rotation of the first tetrahedron of the node's first copy; the solve wrote it.
                            outward = math.mul(TetRotation[NodeSlot[i] / Lattice.CornerCount * Lattice.TetsPerCell], NodeOutward[i]);
                            turned = true;
                        }
                        float facing = -math.dot(outward, n);
                        if (facing < 0.0f && d < 0.0f && shape.Thickness() < 2.0f * outside)
                        {
                            // The collider's nearest face looks away from the tissue: the node went past the middle of a
                            // collider thinner than it may enter (a jaw), whose far face is nearer. Pushing it out there would
                            // push it through.
                            continue;
                        }
                        margin -= outside * math.saturate(facing);
                    }
                    if (d >= margin)
                    {
                        // Adhesion: a node that touched a sticky collider is pulled back towards its
                        // surface until it is further away than the stick distance. Touching counts from
                        // TouchDistance on: the surface cells may hold the surface out of the collider first.
                        if (!(shape.Stickiness > 0.0f) || d >= margin + shape.StickDistance)
                        {
                            continue;
                        }
                        if (Stuck[i] == 0)
                        {
                            held |= d < margin + TouchDistance;
                            continue;
                        }
                        pushed = (d - margin) * shape.Stickiness;
                        x -= n * pushed;
                        held = true;
                    }
                    else
                    {
                        pushed = margin - d;
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
                        // The contact point is on the surface, as far out of the node as it lies outside it.
                        AddImpulse(i, s, x + n * (Margin - margin), x - before, shape.A1, n);
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
        internal static float3 MaterialPointMotion(ColliderShape shape, float3 x)
        {
            float3 local = math.mul(math.conjugate(shape.R1), x - shape.A1);
            return x - (shape.A0 + math.mul(shape.R0, local));
        }

        /// <summary>
        /// Momentum m dx / h the node received from shape <paramref name="s"/>, and the shape's normal
        /// <paramref name="n"/> there (HAP-4), added to the node's block.
        /// </summary>
        private void AddImpulse(int i, int s, float3 x, float3 dx, float3 origin, float3 n)
        {
            float w = Weights[i];
            if (!(w > 0.0f))
            {
                return;
            }
            int index = i / SoftBodySimulation.NodeBatch * Shapes.Length + s;
            ColliderImpulse impulse = Impulses[index];
            float size = math.length(dx);
            impulse.Add(x, origin, dx * (1.0f / (w * Dt)), size);
            impulse.AddNormal(n, size);
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
            // Time since the partners' copies were taken: none when they are refreshed between the
            // substeps, otherwise from the start of the step.
            float tau = PartnersSynced ? 0.0f : FinishTime * StepDt;
            float depth = AdaptorPhysX.BodyContacts.Penetration(x, NodeSdf[i], i * AdaptorPhysX.BodyContacts.MaxCandidates,
                count, tau, PartnerCandidate, PartnerCells, PartnerCorner, PartnerVelocity, PartnerSdf,
                PartnerWeight, out float3 n, out float partnerWeight, out float3 partnerVelocity, out float friction);
            if (!PartnersSynced)
            {
                // The partner resolves its share of the overlap in its own step.
                depth -= PartnerShare[i];
            }
            if (!(depth > 0.0f))
            {
                return x;
            }
            float share = wi / (wi + partnerWeight);
            float push = math.min(depth * share, MaxBodyPush);
            if (!PartnersSynced)
            {
                PartnerShare[i] += push * (1.0f - share) / share;
            }
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

    /// <summary>
    /// Push out of folds, before the node job of every substep: each outer node is held outside the cells of
    /// its own piece that lie far from it at rest (see <see cref="FindFoldCandidatesJob"/>). As in the contact
    /// between bodies (<see cref="BodyContacts"/>), their rest SDF is linear on each deformed tetrahedron; the
    /// node is pushed along its gradient until that surface and the node's own (its rest SDF away from it)
    /// touch, and takes its mass-weighted share. The corners of the cell are pushed back by their own fold
    /// contacts, so the share is relaxed. Reads the solved positions, which the node job then moves: each node
    /// writes only its own push (Jacobi). Runs over the nodes with candidates only (<see cref="FoldNodesJob"/>):
    /// a body without folds would otherwise pass over all its nodes every substep.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FoldContactJob : IJobParallelForDefer
    {
        /// <summary>Nodes with fold candidates in this step.</summary>
        [ReadOnly] public NativeArray<int> Nodes;
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<float> NodeSdf;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int> Candidate;
        [ReadOnly] public NativeArray<int> CandidateCount;
        /// <summary>Largest push per substep.</summary>
        public float MaxPush;
        public float Relaxation;
        [NativeDisableParallelForRestriction] public NativeArray<float3> Push;

        public void Execute(int index)
        {
            int i = Nodes[index];
            int count = CandidateCount[i];
            float wi = Weights[i];
            Push[i] = float3.zero;
            if (count == 0 || wi <= 0.0f)
            {
                return;
            }
            float3 x = Positions[i];
            float best = 0.0f, partnerWeight = 0.0f;
            float3 normal = float3.zero;
            for (int k = 0; k < count; k++)
            {
                int q = Candidate[i * Contacts.MaxFoldCells + k];
                int b = q * Lattice.CornerCount;
                float3 lo = Positions[CopyNodes[b]], hi = lo;
                for (int c = 1; c < 8; c++)
                {
                    float3 p = Positions[CopyNodes[b + c]];
                    lo = math.min(lo, p);
                    hi = math.max(hi, p);
                }
                if (math.any(x < lo) || math.any(x > hi))
                {
                    continue;
                }
                int mask = Copies[q].Mask;
                for (int t = 0; t < Lattice.TetsPerCell; t++)
                {
                    int4 tc = BodyContacts.Tet(t);
                    if (((mask >> tc.x) & (mask >> tc.y) & (mask >> tc.z) & (mask >> tc.w) & 1) == 0)
                    {
                        continue;
                    }
                    int4 n = new int4(CopyNodes[b + tc.x], CopyNodes[b + tc.y], CopyNodes[b + tc.z], CopyNodes[b + tc.w]);
                    float3 p0 = Positions[n.x];
                    float3x3 m = new float3x3(Positions[n.y] - p0, Positions[n.z] - p0, Positions[n.w] - p0);
                    if (!(math.abs(math.determinant(m)) > 1e-18f))
                    {
                        continue;
                    }
                    float3x3 inverse = math.inverse(m);
                    float3 bc = math.mul(inverse, x - p0);
                    float b0 = 1.0f - math.csum(bc);
                    const float eps = -1e-5f;
                    if (b0 < eps || math.any(bc < eps))
                    {
                        continue;
                    }
                    float s0 = NodeSdf[n.x];
                    float3 ds = new float3(NodeSdf[n.y], NodeSdf[n.z], NodeSdf[n.w]) - s0;
                    float3 gradient = math.mul(math.transpose(inverse), ds);
                    float g = math.length(gradient);
                    if (g < 0.5f)
                    {
                        // On the medial axis of the SDF the depth would be far too large (see BodyContacts).
                        break;
                    }
                    // The rest SDF itself rather than phi / |grad phi| as between bodies: the gradient differs from one
                    // tetrahedron to the next, and with the node about 1.5 mm from the surface phi / |grad phi| jumped
                    // by 0.1 mm across their faces, which kept nodes moving to and fro between two tetrahedra.
                    float depth = -(s0 + math.dot(bc, ds)) - NodeSdf[i];
                    if (depth > best)
                    {
                        best = depth;
                        normal = gradient / g;
                        float4 w = new float4(b0, bc);
                        partnerWeight = w.x * w.x * Weights[n.x] + w.y * w.y * Weights[n.y] +
                                        w.z * w.z * Weights[n.z] + w.w * w.w * Weights[n.w];
                    }
                    // A point lies in one tetrahedron of a cell.
                    break;
                }
            }
            if (best > 0.0f)
            {
                Push[i] = normal * (Relaxation * math.min(best * wi / (wi + partnerWeight), MaxPush));
            }
        }
    }

    /// <summary>The nodes with fold candidates in this step (see <see cref="FoldContactJob"/>); every push starts at zero.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FoldNodesJob : IJob
    {
        [ReadOnly] public NativeArray<int> CandidateCount;
        public NativeList<int> Nodes;
        public NativeArray<float3> Push;

        public void Execute()
        {
            Nodes.Clear();
            for (int i = 0; i < CandidateCount.Length; i++)
            {
                Push[i] = float3.zero;
                if (CandidateCount[i] > 0)
                {
                    Nodes.Add(i);
                }
            }
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

        /// <summary>b where c is set, a elsewhere.</summary>
        public static Lanes3 Select(Lanes3 a, Lanes3 b, bool4 c) =>
            new Lanes3(math.select(a.X, b.X, c), math.select(a.Y, b.Y, c), math.select(a.Z, b.Z, c));

        public float3 Lane(int i) => new float3(X[i], Y[i], Z[i]);
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

        /// <summary>
        /// Forces on the kinematic frames (HAP-2): the impulse grad C lambda / dt the constraints exert on
        /// each kinematic node (weight 0, so it does not move) is summed per (slice, frame). A slice is
        /// solved by one thread in a fixed order, so the sums need no atomics.
        /// </summary>
        public bool AccumulateFrames;
        [ReadOnly] public NativeArray<int> KinematicId;
        /// <summary>Torque reference point of each frame.</summary>
        [ReadOnly] public NativeArray<float3> FrameOrigins;
        /// <summary>Slices * FrameCount sums of the momentum the frames gave the tissue.</summary>
        [NativeDisableParallelForRestriction] public NativeArray<ColliderImpulse> FrameImpulses;
        public float InverseDt;
        /// <summary>Number of kinematic frames: the stride of FrameImpulses.</summary>
        public int FrameCount;

        /// <summary>
        /// Contact of the surface cells with the colliders after the cells of each slice (TODO MED-17, see
        /// <see cref="SolveSurfaceContacts"/>). The slice's momentum goes to block ImpulseBlockOffset + slice of the
        /// collider impulse sums, after the node jobs' blocks.
        /// </summary>
        public bool SurfaceContacts;
        [ReadOnly] public NativeArray<float3> Previous;
        [ReadOnly] public NativeArray<float> NodeSdf;
        [ReadOnly] public NativeArray<ColliderShape> Shapes;
        [ReadOnly] public NativeArray<float> Fields;
        [ReadOnly] public NativeArray<int> SurfaceCells;
        [ReadOnly] public NativeArray<int> SliceSurfaceStart;
        [NativeDisableParallelForRestriction] public NativeArray<ColliderImpulse> Impulses;
        public int ImpulseBlockOffset;
        public float Margin;
        /// <summary>Colliders thinner than this take part (<see cref="SoftBodySimulation.ThinColliderCells"/>).</summary>
        public float ThinCollider;
        /// <summary>Time of the colliders at the end and at the start of the substep, 0 to 1 over the step.</summary>
        public float ShapeTime;
        public float ShapeTimePrevious;

        public void Execute(int index)
        {
            int sliceIndex = SliceOffset + index;
            int4 slice = Slices[sliceIndex];
            for (int g = slice.x; g < slice.x + slice.y; g++)
            {
                int4 cells = SliceGroups[g];
                int4 copies = new int4(CellCopyRange[cells.x].y, CellCopyRange[cells.y].y,
                    CellCopyRange[cells.z].y, CellCopyRange[cells.w].y);
                if (math.all(copies == 1))
                {
                    SolveGroup(cells, sliceIndex);
                    continue;
                }
                // A cut cell has several copies: solve the four cells one by one.
                SolveCell(cells.x, sliceIndex);
                SolveCell(cells.y, sliceIndex);
                SolveCell(cells.z, sliceIndex);
                SolveCell(cells.w, sliceIndex);
            }
            for (int c = slice.z; c < slice.z + slice.w; c++)
            {
                SolveCell(SliceCells[c], sliceIndex);
            }
            if (SurfaceContacts)
            {
                SolveSurfaceContacts(sliceIndex);
            }
        }

        /// <summary>
        /// Pushes the surface cells of the slice out of the thin colliders. A cell is tried against a collider when the
        /// collider comes within the cell's bounding sphere; then each of its copies whose corners are all real: the point p
        /// of the collider nearest a point one cell inside the surface from the copy's centre (along -grad phi there) is
        /// located in the copy (u with x(u) = sum w_k(u) x_k = p, by Newton's method). If it lies in the copy, the rest
        /// SDF there, phi = sum w_k(u) phi_k (the contact SDF of the corners, <see cref="BodyContacts"/>), must put it at
        /// least the margin outside, phi / |grad phi| >= margin, where the surface faces the collider (grad phi against
        /// the collider's normal at p). The material at u moves away from the collider along -grad phi by one XPBD correction
        /// of that constraint, with friction against the collider's motion as at the nodes: dlambda = dx / sum w_k^2 m_k^-1
        /// and corner k moves by w_k m_k^-1 dlambda. The trilinear interpolation is the one the rendered skin uses. The
        /// corners are those of the slice, which no other thread writes.
        /// </summary>
        private void SolveSurfaceContacts(int slice)
        {
            for (int e = SliceSurfaceStart[slice]; e < SliceSurfaceStart[slice + 1]; e++)
            {
                int cell = SurfaceCells[e];
                int2 range = CellCopyRange[cell];
                float3 lo = new float3(float.MaxValue), hi = new float3(float.MinValue);
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    for (int k = 0; k < Lattice.CornerCount; k++)
                    {
                        float3 x = Positions[CopyNodes[q * Lattice.CornerCount + k]];
                        lo = math.min(lo, x);
                        hi = math.max(hi, x);
                    }
                }
                float3 center = 0.5f * (lo + hi);
                float reach = 0.5f * math.length(hi - lo) + math.max(Margin, 0.0f);
                for (int s = 0; s < Shapes.Length; s++)
                {
                    ColliderShape shape = Shapes[s];
                    float t = shape.DynamicBody > 0 ? 1.0f : ShapeTime;
                    if (shape.Thickness() >= ThinCollider || shape.BoundDistance(center, t) >= reach)
                    {
                        continue;
                    }
                    for (int q = range.x; q < range.x + range.y; q++)
                    {
                        // A cut copy's virtual corners carry the SDF of the original surface across the cut face.
                        if (Copies[q].Mask == 0xFF)
                        {
                            SurfaceContact(q * Lattice.CornerCount, shape, s, t, slice);
                        }
                    }
                }
            }
        }

        private void SurfaceContact(int b, ColliderShape shape, int s, float t, int slice)
        {
            // The deepest part of the collider in the cell: its point nearest a point a cell inside the surface from the
            // copy's centre (the tip of an instrument pressing in, the floor under the cell).
            float3 centre = float3.zero, centreU = float3.zero;
            var centreJacobian = float3x3.zero;
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                float3 corner = Positions[CopyNodes[b + k]];
                float3 dw = Geometry.TrilinearGradient(k, new float3(0.5f));
                centre += corner;
                centreU += dw * NodeSdf[CopyNodes[b + k]];
                centreJacobian.c0 += corner * dw.x;
                centreJacobian.c1 += corner * dw.y;
                centreJacobian.c2 += corner * dw.z;
            }
            centre *= 1.0f / Lattice.CornerCount;
            if (!(math.abs(math.determinant(centreJacobian)) > 0.0f))
            {
                return;
            }
            float3 inward = -math.normalizesafe(math.mul(math.transpose(math.inverse(centreJacobian)), centreU));
            // One cell: the length of the deformed cell along its first axis.
            float3 probe = centre + inward * math.length(centreJacobian.c0);
            float dc = shape.Distance(probe, t, Fields, out float3 nc);
            float3 p = probe - dc * nc;
            if (!Locate(b, p, out float3 u, out float3x3 jacobian) || math.any(u < 0.0f) || math.any(u > 1.0f))
            {
                return;
            }
            float phi = 0.0f, w2 = 0.0f, free = 0.0f;
            float3 phiU = float3.zero, previous = float3.zero;
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                int n = CopyNodes[b + k];
                float wk = Geometry.TrilinearWeight(k, u);
                float sdf = NodeSdf[n];
                float m = Weights[n];
                phi += wk * sdf;
                phiU += Geometry.TrilinearGradient(k, u) * sdf;
                previous += wk * Previous[n];
                w2 += wk * wk * m;
                free += m > 0.0f ? wk : 0.0f;
            }
            float3 gradient = math.mul(math.transpose(math.inverse(jacobian)), phiU);
            float g = math.length(gradient);
            if (g < 0.5f || !(w2 > 0.0f))
            {
                // Medial axis of the SDF, where phi / |grad phi| overstates the depth (see BodyContacts).
                return;
            }
            float depth = Margin - phi / g;
            float3 normal = gradient / g;
            if (!(depth > 0.0f) || math.dot(normal, nc) >= 0.0f)
            {
                // Only where the surface faces the collider: in tissue thinner than about two cells the SDF of the far
                // side would push the tissue through the collider.
                return;
            }
            float3 dx = -normal * depth;
            float3 relative = (p + dx - previous) - (shape.DynamicBody > 0
                ? NodeStepJob.MaterialPointMotion(shape, p)
                : shape.Displacement(ShapeTimePrevious, ShapeTime));
            float3 tangent = relative - math.dot(relative, normal) * normal;
            float slip = math.length(tangent);
            float limit = shape.Friction * depth;
            dx -= slip <= limit ? tangent : tangent * (limit / slip);
            float3 dlambda = dx / w2;
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                int n = CopyNodes[b + k];
                Positions[n] += Geometry.TrilinearWeight(k, u) * Weights[n] * dlambda;
            }
            // The momentum the corners gained: sum m_k (w_k m_k^-1 dlambda) / h over the free ones.
            float3 impulse = dlambda * (free * InverseDt);
            float size = math.length(dx);
            int index = (ImpulseBlockOffset + slice) * Shapes.Length + s;
            ColliderImpulse sum = Impulses[index];
            sum.Add(p, shape.A1, impulse, size);
            sum.AddNormal(normal, size);
            Impulses[index] = sum;
        }

        /// <summary>
        /// Position <paramref name="u"/> in the copy with corners at b of world point <paramref name="target"/>, inverting
        /// the trilinear interpolation by Newton's method, and the Jacobian dx/du there; false when it does not converge.
        /// </summary>
        private bool Locate(int b, float3 target, out float3 u, out float3x3 jacobian)
        {
            u = new float3(0.5f);
            jacobian = float3x3.zero;
            for (int iteration = 0; iteration < 8; iteration++)
            {
                float3 x = float3.zero;
                jacobian = float3x3.zero;
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    float3 corner = Positions[CopyNodes[b + k]];
                    float3 gradient = Geometry.TrilinearGradient(k, u);
                    x += Geometry.TrilinearWeight(k, u) * corner;
                    jacobian.c0 += corner * gradient.x;
                    jacobian.c1 += corner * gradient.y;
                    jacobian.c2 += corner * gradient.z;
                }
                float3 r = target - x;
                float det = math.determinant(jacobian);
                if (!(math.abs(det) > 0.0f))
                {
                    return false;
                }
                float3 step = math.mul(math.inverse(jacobian), r);
                if (math.cmax(math.abs(step)) < 1e-5f)
                {
                    return true;
                }
                u = math.clamp(u + step, -1.0f, 2.0f);
            }
            return false;
        }

        private void SolveCell(int cell, int slice)
        {
            int2 range = CellCopyRange[cell];
            for (int q = range.x; q < range.x + range.y; q++)
            {
                SolveCopy(cell, q, slice);
            }
        }

        /// <summary>
        /// Adds the constraint impulse <paramref name="d"/> / dt on a corner of weight <paramref name="w"/>
        /// to its frame, if it is kinematic.
        /// </summary>
        private void AddFrameImpulse(int node, float w, float3 x, float3 d, int slice)
        {
            int frame = KinematicId[node];
            if (frame < 0 || w != 0.0f)
            {
                return;
            }
            // The frame gives the tissue the opposite of what the constraints exert on its node.
            float3 impulse = -d * InverseDt;
            int index = slice * FrameCount + frame;
            ColliderImpulse sum = FrameImpulses[index];
            sum.Add(x, FrameOrigins[frame], impulse, math.length(impulse));
            FrameImpulses[index] = sum;
        }

        private void AddFrameImpulses(int4 n, float4 w, Lanes3 x, Lanes3 d, int slice)
        {
            AddFrameImpulse(n.x, w.x, x.Lane(0), d.Lane(0), slice);
            AddFrameImpulse(n.y, w.y, x.Lane(1), d.Lane(1), slice);
            AddFrameImpulse(n.z, w.z, x.Lane(2), d.Lane(2), slice);
            AddFrameImpulse(n.w, w.w, x.Lane(3), d.Lane(3), slice);
        }

        /// <summary>The six tetrahedra of four uncut cells without a shared node, one cell per lane.</summary>
        private void SolveGroup(int4 cells, int slice)
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
            // Constraint impulses on the corners of zero weight, summed over the six tetrahedra.
            bool frames = AccumulateFrames && math.any((w0 == 0.0f) | (w1 == 0.0f) | (w2 == 0.0f) | (w3 == 0.0f) |
                                                       (w4 == 0.0f) | (w5 == 0.0f) | (w6 == 0.0f) | (w7 == 0.0f));
            Lanes3 d0 = default, d1 = default, d2 = default, d3 = default;
            Lanes3 d4 = default, d5 = default, d6 = default, d7 = default;

            SolveTets(ref x0, ref x1, ref x3, ref x7, w0, w1, w3, w7, 0, restScale, volumeScale, r, frames, ref d0, ref d1, ref d3, ref d7);
            SolveTets(ref x0, ref x1, ref x7, ref x5, w0, w1, w7, w5, 1, restScale, volumeScale, r + 1, frames, ref d0, ref d1, ref d7, ref d5);
            SolveTets(ref x0, ref x2, ref x7, ref x3, w0, w2, w7, w3, 2, restScale, volumeScale, r + 2, frames, ref d0, ref d2, ref d7, ref d3);
            SolveTets(ref x0, ref x2, ref x6, ref x7, w0, w2, w6, w7, 3, restScale, volumeScale, r + 3, frames, ref d0, ref d2, ref d6, ref d7);
            SolveTets(ref x0, ref x4, ref x5, ref x7, w0, w4, w5, w7, 4, restScale, volumeScale, r + 4, frames, ref d0, ref d4, ref d5, ref d7);
            SolveTets(ref x0, ref x4, ref x7, ref x6, w0, w4, w7, w6, 5, restScale, volumeScale, r + 5, frames, ref d0, ref d4, ref d7, ref d6);

            Store(n0, x0);
            Store(n1, x1);
            Store(n2, x2);
            Store(n3, x3);
            Store(n4, x4);
            Store(n5, x5);
            Store(n6, x6);
            Store(n7, x7);
            if (frames)
            {
                AddFrameImpulses(n0, w0, x0, d0, slice);
                AddFrameImpulses(n1, w1, x1, d1, slice);
                AddFrameImpulses(n2, w2, x2, d2, slice);
                AddFrameImpulses(n3, w3, x3, d3, slice);
                AddFrameImpulses(n4, w4, x4, d4, slice);
                AddFrameImpulses(n5, w5, x5, d5, slice);
                AddFrameImpulses(n6, w6, x6, d6, slice);
                AddFrameImpulses(n7, w7, x7, d7, slice);
            }
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

        /// <summary>
        /// Lane form of <see cref="SolveTet"/>: the same constraints, branches turned into selects. With
        /// <paramref name="frames"/>, grad C lambda of the corners of zero weight is added to d0..d3.
        /// </summary>
        private void SolveTets(ref Lanes3 x0, ref Lanes3 x1, ref Lanes3 x2, ref Lanes3 x3,
            float4 w0, float4 w1, float4 w2, float4 w3, int tet, LaneMatrix restScale, float4 volumeScale, int4 rotationIndex,
            bool frames, ref Lanes3 d0, ref Lanes3 d1, ref Lanes3 d2, ref Lanes3 d3)
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

            Lanes3 g0 = gH0 * dH + gD0 * dD;
            Lanes3 g1 = gH.C0 * dH + gD.C0 * dD;
            Lanes3 g2 = gH.C1 * dH + gD.C1 * dD;
            Lanes3 g3 = gH.C2 * dH + gD.C2 * dD;
            x0 += g0 * w0;
            x1 += g1 * w1;
            x2 += g2 * w2;
            x3 += g3 * w3;
            if (frames)
            {
                // As in the scalar path, a tetrahedron without a free corner exerts nothing.
                bool4 free = w0 + w1 + w2 + w3 > 0.0f;
                d0 = Lanes3.Select(d0, d0 + g0, free & (w0 == 0.0f));
                d1 = Lanes3.Select(d1, d1 + g1, free & (w1 == 0.0f));
                d2 = Lanes3.Select(d2, d2 + g2, free & (w2 == 0.0f));
                d3 = Lanes3.Select(d3, d3 + g3, free & (w3 == 0.0f));
            }
        }

        /// <summary>
        /// The six tetrahedra of one cell copy. The eight corners are read once, updated in
        /// registers and written back once: consecutive tetrahedra share the 0-7 diagonal, and
        /// passing their positions through memory stalls on store-to-load forwarding.
        /// </summary>
        private void SolveCopy(int cell, int copy, int slice)
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
            bool frames = AccumulateFrames && (w0 == 0.0f || w1 == 0.0f || w2 == 0.0f || w3 == 0.0f ||
                                               w4 == 0.0f || w5 == 0.0f || w6 == 0.0f || w7 == 0.0f);
            float3 d0 = float3.zero, d1 = float3.zero, d2 = float3.zero, d3 = float3.zero;
            float3 d4 = float3.zero, d5 = float3.zero, d6 = float3.zero, d7 = float3.zero;

            // Kuhn split around the 0-7 diagonal, as in Lattice.TetCorners (the same for both parities).
            SolveTet(ref x0, ref x1, ref x3, ref x7, w0, w1, w3, w7, 0, restScale, volumeScale, r, frames, ref d0, ref d1, ref d3, ref d7);
            SolveTet(ref x0, ref x1, ref x7, ref x5, w0, w1, w7, w5, 1, restScale, volumeScale, r + 1, frames, ref d0, ref d1, ref d7, ref d5);
            SolveTet(ref x0, ref x2, ref x7, ref x3, w0, w2, w7, w3, 2, restScale, volumeScale, r + 2, frames, ref d0, ref d2, ref d7, ref d3);
            SolveTet(ref x0, ref x2, ref x6, ref x7, w0, w2, w6, w7, 3, restScale, volumeScale, r + 3, frames, ref d0, ref d2, ref d6, ref d7);
            SolveTet(ref x0, ref x4, ref x5, ref x7, w0, w4, w5, w7, 4, restScale, volumeScale, r + 4, frames, ref d0, ref d4, ref d5, ref d7);
            SolveTet(ref x0, ref x4, ref x7, ref x6, w0, w4, w7, w6, 5, restScale, volumeScale, r + 5, frames, ref d0, ref d4, ref d7, ref d6);

            Positions[n0] = x0;
            Positions[n1] = x1;
            Positions[n2] = x2;
            Positions[n3] = x3;
            Positions[n4] = x4;
            Positions[n5] = x5;
            Positions[n6] = x6;
            Positions[n7] = x7;
            if (frames)
            {
                AddFrameImpulse(n0, w0, x0, d0, slice);
                AddFrameImpulse(n1, w1, x1, d1, slice);
                AddFrameImpulse(n2, w2, x2, d2, slice);
                AddFrameImpulse(n3, w3, x3, d3, slice);
                AddFrameImpulse(n4, w4, x4, d4, slice);
                AddFrameImpulse(n5, w5, x5, d5, slice);
                AddFrameImpulse(n6, w6, x6, d6, slice);
                AddFrameImpulse(n7, w7, x7, d7, slice);
            }
        }

        /// <summary>
        /// One tetrahedron. With <paramref name="frames"/>, grad C lambda of the corners of zero weight
        /// is added to d0..d3.
        /// </summary>
        private void SolveTet(ref float3 x0, ref float3 x1, ref float3 x2, ref float3 x3,
            float w0, float w1, float w2, float w3, int tet, float3x3 restScale, float volumeScale, int rotationIndex,
            bool frames, ref float3 d0, ref float3 d1, ref float3 d2, ref float3 d3)
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

            float3 g0 = gH0 * dH + gD0 * dD;
            float3 g1 = gH.c0 * dH + gD.c0 * dD;
            float3 g2 = gH.c1 * dH + gD.c1 * dD;
            float3 g3 = gH.c2 * dH + gD.c2 * dD;
            x0 += w0 * g0;
            x1 += w1 * g1;
            x2 += w2 * g2;
            x3 += w3 * g3;
            if (frames)
            {
                d0 += w0 == 0.0f ? g0 : float3.zero;
                d1 += w1 == 0.0f ? g1 : float3.zero;
                d2 += w2 == 0.0f ? g2 : float3.zero;
                d3 += w3 == 0.0f ? g3 : float3.zero;
            }
        }
    }
}
