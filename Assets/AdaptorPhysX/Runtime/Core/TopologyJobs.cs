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
    [BurstCompile(CompileSynchronously = true)]
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

            var parentCopy = new NativeList<int>(CellCoord.Length, Allocator.Temp);
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

    /// <summary>Gathers node state for the new topology from the inherited old nodes.</summary>
    [BurstCompile(CompileSynchronously = true)]
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
        public float4x4 RestToWorld;

        public NativeArray<float3> Positions;
        public NativeArray<float3> Previous;
        public NativeArray<float3> Velocities;
        public NativeArray<float> InverseMass;
        public NativeArray<int> KinematicId;
        public NativeArray<float3> KinematicLocal;

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
                return;
            }
            Positions[i] = OldPositions[p];
            Previous[i] = OldPrevious[p];
            Velocities[i] = OldVelocities[p];
            KinematicId[i] = OldKinematicId[p];
            KinematicLocal[i] = OldKinematicLocal[p];
        }
    }

    /// <summary>
    /// Severs every intact lattice edge crossed by the swept blade quad
    /// (a0,b0,b1,a1) and records Hermite data (edge parameter and rest-space
    /// normal) on all cells sharing the edge.
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    internal struct CutJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int> GridToCell;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int> EdgeCorners;
        public int3 Dims;
        public float3 A0, B0, A1, B1;

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

            int severedCount = 0;
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
                if (math.any(cmax < lo) || math.any(cmin > hi))
                {
                    continue;
                }

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
                    float3 p = Positions[CopyNodes[owner * 8 + k0]];
                    float3 r = Positions[CopyNodes[owner * 8 + k1]];
                    float t = Geometry.SegmentTriangle(p, r, A0, B0, B1);
                    if (t < 0.0f)
                    {
                        t = Geometry.SegmentTriangle(p, r, A0, B1, A1);
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
            SeveredCount[0] = severedCount;
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
