using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Rebuilds cell copies and node sharing from the severed-edge masks
    /// (virtual node algorithm, Molino et al. 2004, on a hexahedral lattice).
    ///
    /// Every connected corner component of a cell becomes one copy that keeps all
    /// eight nodes; corners outside its component are virtual nodes. Two copies of
    /// neighbouring cells are glued, i.e. share all nodes of their common grid
    /// points, when both own material at one of those points. Severing only ever
    /// removes connectivity, so each new copy derives from exactly one old copy and
    /// each new node inherits the state of exactly one old node.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct TopologyJob : IJob
    {
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int> GridToCell;
        [ReadOnly] public NativeArray<float> CellFill;
        [ReadOnly] public NativeArray<ushort> Severed;
        [ReadOnly] public NativeArray<byte> ComponentCounts;
        [ReadOnly] public NativeArray<byte> ComponentMasks;
        [ReadOnly] public NativeArray<int3> Directions;
        [ReadOnly] public NativeArray<int> PairStart;
        [ReadOnly] public NativeArray<int> Pairs;
        public int3 Dims;
        public float3 Origin;
        public float CellSize;
        public float CellMass;

        public bool HasOld;
        [ReadOnly] public NativeArray<int2> OldRange;
        [ReadOnly] public NativeArray<CellCopy> OldCopies;
        [ReadOnly] public NativeArray<int> OldCopyNodes;

        public NativeArray<int2> NewRange;
        public NativeList<CellCopy> NewCopies;
        public NativeList<int> NewCopyNodes;
        /// <summary>Old copy each new copy derives from, or -1.</summary>
        public NativeList<int> NewCopyParent;
        /// <summary>Old node each new node inherits from, or -1 for a fresh node at rest.</summary>
        public NativeList<int> NodeParent;
        public NativeList<float3> NodeRest;
        public NativeList<float> NodeMass;

        public void Execute()
        {
            NewCopies.Clear();
            NewCopyNodes.Clear();
            NodeParent.Clear();
            NodeRest.Clear();
            NodeMass.Clear();

            NewCopyParent.Clear();
            NativeList<int> parentCopy = NewCopyParent;
            for (int c = 0; c < CellCoord.Length; c++)
            {
                int severed = Severed[c];
                int count = ComponentCounts[severed];
                NewRange[c] = new int2(NewCopies.Length, count);
                for (int i = 0; i < count; i++)
                {
                    byte mask = ComponentMasks[severed * Lattice.CornerCount + i];
                    int parent = -1;
                    if (HasOld)
                    {
                        int2 old = OldRange[c];
                        for (int q = old.x; q < old.x + old.y; q++)
                        {
                            if ((OldCopies[q].Mask & mask) == mask)
                            {
                                parent = q;
                                break;
                            }
                        }
                    }
                    NewCopies.Add(new CellCopy
                    {
                        Cell = c,
                        Mask = mask,
                        Weight = CellFill[c] * math.countbits((uint)mask) / 8.0f,
                    });
                    parentCopy.Add(parent);
                }
            }

            int slotCount = NewCopies.Length * Lattice.CornerCount;
            var root = new NativeArray<int>(slotCount, Allocator.Temp);
            for (int s = 0; s < slotCount; s++)
            {
                root[s] = s;
            }

            for (int c = 0; c < CellCoord.Length; c++)
            {
                int3 coord = CellCoord[c];
                int2 rc = NewRange[c];
                for (int d = 0; d < Lattice.NeighborDirections; d++)
                {
                    int3 nc = coord + Directions[d];
                    if (math.any(nc < 0) || math.any(nc >= Dims))
                    {
                        continue;
                    }
                    int n = GridToCell[nc.x + Dims.x * (nc.y + Dims.y * nc.z)];
                    if (n < 0)
                    {
                        continue;
                    }
                    int2 rn = NewRange[n];
                    int p0 = PairStart[d], p1 = PairStart[d + 1];
                    for (int qa = rc.x; qa < rc.x + rc.y; qa++)
                    {
                        int ma = NewCopies[qa].Mask;
                        for (int qb = rn.x; qb < rn.x + rn.y; qb++)
                        {
                            int mb = NewCopies[qb].Mask;
                            bool glue = false;
                            for (int p = p0; p < p1 && !glue; p++)
                            {
                                glue = ((ma >> Pairs[2 * p]) & 1) != 0 && ((mb >> Pairs[2 * p + 1]) & 1) != 0;
                            }
                            if (!glue)
                            {
                                continue;
                            }
                            for (int p = p0; p < p1; p++)
                            {
                                Union(root, qa * 8 + Pairs[2 * p], qb * 8 + Pairs[2 * p + 1]);
                            }
                        }
                    }
                }
            }

            var nodeOf = new NativeArray<int>(slotCount, Allocator.Temp);
            for (int s = 0; s < slotCount; s++)
            {
                nodeOf[s] = -1;
            }
            NewCopyNodes.ResizeUninitialized(slotCount);
            for (int s = 0; s < slotCount; s++)
            {
                int r = Find(root, s);
                int q = s / Lattice.CornerCount;
                int k = s % Lattice.CornerCount;
                if (nodeOf[r] < 0)
                {
                    nodeOf[r] = NodeParent.Length;
                    int parent = HasOld && parentCopy[q] >= 0 ? OldCopyNodes[parentCopy[q] * 8 + k] : -1;
                    int3 gridPoint = CellCoord[NewCopies[q].Cell] + Lattice.CornerOffset(k);
                    NodeParent.Add(parent);
                    NodeRest.Add(Origin + (float3)gridPoint * CellSize);
                    NodeMass.Add(0.0f);
                }
                int node = nodeOf[r];
                NewCopyNodes[s] = node;
                NodeMass[node] += CellMass * NewCopies[q].Weight / Lattice.CornerCount;
            }
        }

        private static int Find(NativeArray<int> root, int x)
        {
            while (root[x] != x)
            {
                root[x] = root[root[x]];
                x = root[x];
            }
            return x;
        }

        private static void Union(NativeArray<int> root, int a, int b)
        {
            a = Find(root, a);
            b = Find(root, b);
            if (a < b)
            {
                root[b] = a;
            }
            else if (b < a)
            {
                root[a] = b;
            }
        }
    }

    /// <summary>Marks nodes that own material and labels the connected pieces of real corners.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct PieceJob : IJob
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        public NativeArray<byte> NodeReal;
        public NativeArray<int> PieceId;

        public void Execute()
        {
            var root = new NativeArray<int>(PieceId.Length, Allocator.Temp);
            for (int i = 0; i < root.Length; i++)
            {
                root[i] = i;
            }
            for (int q = 0; q < Copies.Length; q++)
            {
                int mask = Copies[q].Mask;
                int first = -1;
                for (int k = 0; k < 8; k++)
                {
                    if (((mask >> k) & 1) == 0)
                    {
                        continue;
                    }
                    int n = CopyNodes[q * 8 + k];
                    NodeReal[n] = 1;
                    if (first < 0)
                    {
                        first = n;
                    }
                    else
                    {
                        int a = Find(root, n), b = Find(root, first);
                        root[math.max(a, b)] = math.min(a, b);
                    }
                }
            }
            for (int i = 0; i < PieceId.Length; i++)
            {
                PieceId[i] = NodeReal[i] != 0 ? Find(root, i) : -1;
            }
        }

        private static int Find(NativeArray<int> root, int x)
        {
            while (root[x] != x)
            {
                root[x] = root[root[x]];
                x = root[x];
            }
            return x;
        }
    }

    /// <summary>Gathers node state for the new topology from the inherited old nodes.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RemapNodesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> NodeParent;
        [ReadOnly] public NativeArray<float3> NodeRest;
        [ReadOnly] public NativeArray<float> NodeMass;
        [ReadOnly] public NativeArray<float3> OldPositions;
        [ReadOnly] public NativeArray<float3> OldPrevious;
        [ReadOnly] public NativeArray<float3> OldVelocities;
        [ReadOnly] public NativeArray<int> OldKinematicId;
        [ReadOnly] public NativeArray<float3> OldKinematicLocal;
        [ReadOnly] public NativeArray<sbyte> OldStrokeSide;
        [ReadOnly] public NativeArray<float> OldStrokeTravel;
        [ReadOnly] public NativeArray<float> OldStrokeDistance;
        public float4x4 RestToWorld;

        public NativeArray<float3> Positions;
        public NativeArray<float3> Previous;
        public NativeArray<float3> Velocities;
        public NativeArray<float> InverseMass;
        public NativeArray<int> KinematicId;
        public NativeArray<float3> KinematicLocal;
        public NativeArray<sbyte> StrokeSide;
        public NativeArray<float> StrokeTravel;
        public NativeArray<float> StrokeDistance;

        public void Execute(int i)
        {
            int p = NodeParent[i];
            InverseMass[i] = NodeMass[i] > 0.0f ? 1.0f / NodeMass[i] : 0.0f;
            if (p < 0)
            {
                float3 x = math.transform(RestToWorld, NodeRest[i]);
                Positions[i] = x;
                Previous[i] = x;
                Velocities[i] = float3.zero;
                KinematicId[i] = -1;
                KinematicLocal[i] = float3.zero;
                StrokeSide[i] = 0;
                StrokeTravel[i] = 0.0f;
                StrokeDistance[i] = 0.0f;
                return;
            }
            Positions[i] = OldPositions[p];
            Previous[i] = OldPrevious[p];
            Velocities[i] = OldVelocities[p];
            KinematicId[i] = OldKinematicId[p];
            KinematicLocal[i] = OldKinematicLocal[p];
            StrokeSide[i] = OldStrokeSide[p];
            StrokeTravel[i] = OldStrokeTravel[p];
            StrokeDistance[i] = OldStrokeDistance[p];
        }
    }

    /// <summary>
    /// Severs every intact lattice edge crossed by the swept blade quad
    /// (a0,b0,b1,a1), and the edges outside the surface beyond the tip or heel of a
    /// blade that has left the material through it (<see cref="OutsideCrossing"/>), and records
    /// Hermite data (edge parameter and rest-space normal) on all cells sharing the edge.
    /// Within a stroke, nodes are labelled with the side of the blade they were on when the
    /// blade passed them, and an edge whose ends both carry a label is severed exactly when
    /// the labels differ.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CutJob : IJob
    {
        /// <summary>Edge parameter range at each end where an on-plane endpoint is tested directly.</summary>
        private const float EndpointTolerance = 1e-3f;

        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int> GridToCell;
        [ReadOnly] public NativeArray<float> PointSdf;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int> EdgeCorners;
        public int3 Dims;
        public float3 A0, B0, A1, B1;
        /// <summary>Previous blade position without the overlap with the previous sweep.</summary>
        public float3 SweptA0, SweptB0;

        /// <summary>
        /// Per node: side of the blade when the current stroke passed it (+1, -1, or 0 for not
        /// yet), and the blade travel of the stroke and the node's distance from the blade at that moment.
        /// </summary>
        public NativeArray<sbyte> StrokeSide;
        public NativeArray<float> StrokeTravel;
        public NativeArray<float> StrokeDistance;
        /// <summary>Blade travel of the stroke up to this sweep.</summary>
        public float Travel;
        /// <summary>Blade travel after which a label is renewed, so a stroke may come back over the same tissue.</summary>
        public float LabelTravel;
        /// <summary>Distance from the blade within which passed nodes are labelled.</summary>
        public float Band;
        /// <summary>+1 or -1: orientation of this sweep's blade normal relative to the stroke, so the labels keep their meaning when the blade reverses.</summary>
        public float Orientation;

        public NativeArray<ushort> Severed;
        public NativeArray<float> EdgeT;
        public NativeArray<float3> EdgeNormal;
        public NativeArray<int> SeveredCount;

        public void Execute()
        {
            float3 lo = math.min(math.min(A0, B0), math.min(A1, B1));
            float3 hi = math.max(math.max(A0, B0), math.max(A1, B1));
            float3 n1 = math.cross(B0 - A0, B1 - A0);
            float3 n2 = math.cross(B1 - A0, A1 - A0);
            float3 bladeNormal = math.normalizesafe(n1 + n2);
            if (math.lengthsq(bladeNormal) < 0.5f)
            {
                return;
            }

            // Label the nodes near the blade first, then test the edges of the same cells: an edge whose
            // ends were labelled in different sweeps may have moved off the blade by now.
            var cut = new NativeList<int>(256, Allocator.Temp);
            for (int c = 0; c < CellCoord.Length; c++)
            {
                int2 range = CellCopyRange[c];
                float3 cmin = new float3(float.MaxValue), cmax = new float3(float.MinValue);
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    for (int k = 0; k < 8; k++)
                    {
                        float3 x = Positions[CopyNodes[q * 8 + k]];
                        cmin = math.min(cmin, x);
                        cmax = math.max(cmax, x);
                    }
                }
                if (math.any(cmax < lo - Band) || math.any(cmin > hi + Band))
                {
                    continue;
                }
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = Copies[q].Mask;
                    for (int k = 0; k < 8; k++)
                    {
                        if (((mask >> k) & 1) != 0)
                        {
                            Label(CopyNodes[q * 8 + k], n1, n2);
                        }
                    }
                }
                cut.Add(c);
            }

            int severedCount = 0;
            for (int i = 0; i < cut.Length; i++)
            {
                int c = cut[i];
                int2 range = CellCopyRange[c];
                LabelBetweenSides(c, range, bladeNormal);
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((Severed[c] & (1 << e)) != 0)
                    {
                        continue;
                    }
                    int k0 = EdgeCorners[2 * e], k1 = EdgeCorners[2 * e + 1];
                    int owner = -1;
                    for (int q = range.x; q < range.x + range.y && owner < 0; q++)
                    {
                        int mask = Copies[q].Mask;
                        if (((mask >> k0) & 1) != 0 && ((mask >> k1) & 1) != 0)
                        {
                            owner = q;
                        }
                    }
                    if (owner < 0)
                    {
                        continue;
                    }
                    int np = CopyNodes[owner * 8 + k0], nr = CopyNodes[owner * 8 + k1];
                    float3 p = Positions[np];
                    float3 r = Positions[nr];
                    int sp = Side(np), sr = Side(nr);
                    float t = Crossing(p, r, A0, B0, B1, n1);
                    if (t < 0.0f)
                    {
                        t = Crossing(p, r, A0, B1, A1, n2);
                    }
                    if (sp != 0 && sr != 0)
                    {
                        // Both ends were passed by this stroke: the sides they were on then decide.
                        // Tissue that moves across the blade plane during the stroke would otherwise
                        // leave an edge whose ends were on different sides when the blade passed
                        // each of them, but which never crosses the blade when it is tested.
                        if (sp == sr)
                        {
                            continue;
                        }
                        t = t < 0.0f ? LabelCrossing(np, nr) : t;
                    }
                    else if (t < 0.0f)
                    {
                        t = OutsideCrossing(c, k0, k1, p, r, n1, n2);
                    }
                    if (t < 0.0f)
                    {
                        continue;
                    }
                    t = math.clamp(t, 0.05f, 0.95f);
                    float3 restNormal = RestNormal(owner, k0, k1, t, bladeNormal);
                    SeverShared(c, e, t, restNormal);
                    severedCount++;
                }
            }
            cut.Dispose();
            SeveredCount[0] = severedCount;
        }

        /// <summary>
        /// Labels, by its current side, an unlabelled node joined by intact edges of the cell to nodes
        /// of both labels. Tissue the blade drags along can leave such a node outside the band
        /// whenever its projection lies on the quad; through it the two sides would stay joined.
        /// Nodes next to one label only stay unlabelled, so a partial incision does not grow past
        /// the blade.
        /// </summary>
        private void LabelBetweenSides(int cell, int2 range, float3 bladeNormal)
        {
            int severed = Severed[cell];
            for (int q = range.x; q < range.x + range.y; q++)
            {
                int mask = Copies[q].Mask;
                for (int k = 0; k < 8; k++)
                {
                    int node = CopyNodes[q * 8 + k];
                    if (((mask >> k) & 1) == 0 || Side(node) != 0)
                    {
                        continue;
                    }
                    bool plus = false, minus = false;
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        int k0 = EdgeCorners[2 * e], k1 = EdgeCorners[2 * e + 1];
                        int other = k0 == k ? k1 : k1 == k ? k0 : -1;
                        if (other < 0 || (severed & (1 << e)) != 0 || ((mask >> other) & 1) == 0)
                        {
                            continue;
                        }
                        int side = Side(CopyNodes[q * 8 + other]);
                        plus |= side > 0;
                        minus |= side < 0;
                    }
                    if (plus && minus)
                    {
                        float d = math.dot(Positions[node] - A0, bladeNormal) * Orientation;
                        StrokeSide[node] = (sbyte)(d >= 0.0f ? 1 : -1);
                        StrokeTravel[node] = Travel;
                        StrokeDistance[node] = d;
                    }
                }
            }
        }

        /// <summary>The node's label of this stroke, or 0 when the blade has not passed it or the label is old.</summary>
        private int Side(int node) =>
            StrokeSide[node] != 0 && Travel - StrokeTravel[node] <= LabelTravel ? StrokeSide[node] : 0;

        /// <summary>Labels a node within <see cref="Band"/> of the blade whose projection lies on the swept quad.</summary>
        private void Label(int node, float3 n1, float3 n2)
        {
            if (Side(node) != 0)
            {
                return;
            }
            float3 x = Positions[node];
            for (int half = 0; half < 2; half++)
            {
                float3 n = half == 0 ? n1 : n2;
                float length = math.length(n);
                if (length <= 0.0f || !Geometry.PointInTriangle(x, A0, half == 0 ? B0 : B1, half == 0 ? B1 : A1, n))
                {
                    continue;
                }
                float d = math.dot(x - A0, n) / length * Orientation;
                if (math.abs(d) <= Band)
                {
                    // An end on the blade counts as the positive side, as in Crossing.
                    StrokeSide[node] = (sbyte)(d >= 0.0f ? 1 : -1);
                    StrokeTravel[node] = Travel;
                    StrokeDistance[node] = d;
                }
                return;
            }
        }

        /// <summary>
        /// Parameter of the blade between the ends of an edge labelled on opposite sides, from their
        /// distances to the blade when it passed them. The ends may have been passed by different
        /// sweeps, and the edge may have moved off the blade since: the edge midpoint would put the
        /// cut surface up to half a cell off the blade path.
        /// </summary>
        private float LabelCrossing(int p, int r)
        {
            float dp = math.abs(StrokeDistance[p]), dr = math.abs(StrokeDistance[r]);
            return dp + dr > 0.0f ? dp / (dp + dr) : 0.5f;
        }

        /// <summary>
        /// Segment parameter where p-r crosses triangle abc (normal n), or -1. An endpoint on the plane
        /// counts as the positive side, so a node on the blade keeps the edges to its positive neighbours
        /// and a blade through a layer of nodes separates the body.
        /// </summary>
        private static float Crossing(float3 p, float3 r, float3 a, float3 b, float3 c, float3 n)
        {
            float dp = math.dot(p - a, n), dr = math.dot(r - a, n);
            if ((dp >= 0.0f) == (dr >= 0.0f))
            {
                return -1.0f;
            }
            float t = Geometry.SegmentTriangle(p, r, a, b, c);
            if (t >= 0.0f)
            {
                return t;
            }
            // SegmentTriangle can reject a crossing at an endpoint by rounding of t; away from the
            // endpoints its result stands.
            float s = dp / (dp - dr);
            if (s > EndpointTolerance && s < 1.0f - EndpointTolerance)
            {
                return -1.0f;
            }
            return Geometry.PointInTriangle(math.lerp(p, r, s), a, b, c, n) ? s : -1.0f;
        }

        /// <summary>
        /// For an edge with both ends outside the surface, which carries no material: the parameter
        /// where it crosses the plane of a blade triangle when the swept quad comes at least as close
        /// to that point as the surface does (rest distance), i.e. the blade has left the material
        /// through the surface beyond its tip or heel; otherwise -1. The lattice nodes of boundary
        /// cells lie up to about one cell outside the surface, and the cell splits only when these
        /// edges are severed too.
        /// </summary>
        private float OutsideCrossing(int cell, int k0, int k1, float3 p, float3 r, float3 n1, float3 n2)
        {
            int3 pointDims = Dims + 1;
            int3 g0 = CellCoord[cell] + Lattice.CornerOffset(k0);
            int3 g1 = CellCoord[cell] + Lattice.CornerOffset(k1);
            float sdf0 = PointSdf[g0.x + pointDims.x * (g0.y + pointDims.y * g0.z)];
            float sdf1 = PointSdf[g1.x + pointDims.x * (g1.y + pointDims.y * g1.z)];
            if (sdf0 <= 0.0f || sdf1 <= 0.0f)
            {
                return -1.0f;
            }
            for (int half = 0; half < 2; half++)
            {
                float3 n = half == 0 ? n1 : n2;
                float dp = math.dot(p - A0, n), dr = math.dot(r - A0, n);
                if ((dp >= 0.0f) == (dr >= 0.0f))
                {
                    continue;
                }
                float s = dp / (dp - dr);
                float3 x = math.lerp(p, r, s);
                // Only beyond the tip or heel, and only in the part swept in this step (without the
                // overlap with the previous sweep). Tissue that moves across the plane between two
                // steps would otherwise be cut on both sides and leave single-node fragments.
                float3 normal = n1 + n2;
                if (math.dot(math.cross(SweptB0 - SweptA0, x - SweptA0), normal) < 0.0f ||
                    math.dot(math.cross(B1 - A1, x - A1), normal) > 0.0f)
                {
                    continue;
                }
                float reach = float.MaxValue;
                if (math.lengthsq(n1) > 0.0f)
                {
                    reach = math.distance(x, Geometry.ClosestPointOnTriangle(x, A0, B0, B1));
                }
                if (math.lengthsq(n2) > 0.0f)
                {
                    reach = math.min(reach, math.distance(x, Geometry.ClosestPointOnTriangle(x, A0, B1, A1)));
                }
                if (reach <= math.lerp(sdf0, sdf1, s))
                {
                    return s;
                }
            }
            return -1.0f;
        }

        /// <summary>Maps the world blade normal into rest space with the cell's deformation Jacobian.</summary>
        private float3 RestNormal(int copy, int k0, int k1, float t, float3 worldNormal)
        {
            float3 u = math.lerp((float3)Lattice.CornerOffset(k0), (float3)Lattice.CornerOffset(k1), t);
            float3x3 j = float3x3.zero;
            for (int k = 0; k < 8; k++)
            {
                float3 x = Positions[CopyNodes[copy * 8 + k]];
                float3 g = Geometry.TrilinearGradient(k, u);
                j.c0 += x * g.x;
                j.c1 += x * g.y;
                j.c2 += x * g.z;
            }
            float3 n = math.normalizesafe(math.mul(math.transpose(j), worldNormal));
            return math.lengthsq(n) > 0.5f ? n : worldNormal;
        }

        private void SeverShared(int cell, int edge, float t, float3 restNormal)
        {
            int axis = edge / 4;
            int start = EdgeCorners[2 * edge];
            int3 gridPoint = CellCoord[cell] + Lattice.CornerOffset(start);
            int axisB = (axis + 1) % 3, axisC = (axis + 2) % 3;
            for (int ob = 0; ob < 2; ob++)
            {
                for (int oc = 0; oc < 2; oc++)
                {
                    int3 offset = int3.zero;
                    offset[axisB] = ob;
                    offset[axisC] = oc;
                    int3 nc = gridPoint - offset;
                    if (math.any(nc < 0) || math.any(nc >= Dims))
                    {
                        continue;
                    }
                    int n = GridToCell[nc.x + Dims.x * (nc.y + Dims.y * nc.z)];
                    if (n < 0)
                    {
                        continue;
                    }
                    int localStart = Lattice.CornerIndex(offset);
                    int localEdge = -1;
                    for (int e = axis * 4; e < axis * 4 + 4; e++)
                    {
                        if (EdgeCorners[2 * e] == localStart)
                        {
                            localEdge = e;
                        }
                    }
                    Severed[n] = (ushort)(Severed[n] | (1 << localEdge));
                    EdgeT[n * 12 + localEdge] = t;
                    EdgeNormal[n * 12 + localEdge] = restNormal;
                }
            }
        }
    }
}
