using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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
        /// <summary>
        /// Least node mass. A corner of a single nearly empty cell (fill 1/8, the voxelizer's least) weighs 1/64 of a
        /// cell, against up to a whole cell for the other corners of its tetrahedra; with 16 substeps such corners
        /// moved back and forth between substeps once the body was deformed, with 32 substeps or this floor they do
        /// not (TODO BUG-13).
        /// </summary>
        public float MinNodeMass;

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
        /// <summary>First slot (copy * 8 + corner) of every node: nodes are numbered in the order of their first slots.</summary>
        public NativeList<int> FirstSlot;

        public void Execute()
        {
            NewCopies.Clear();
            NewCopyNodes.Clear();
            NodeParent.Clear();
            NodeRest.Clear();
            NodeMass.Clear();
            FirstSlot.Clear();

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
                    FirstSlot.Add(s);
                }
                int node = nodeOf[r];
                NewCopyNodes[s] = node;
                NodeMass[node] += CellMass * NewCopies[q].Weight / Lattice.CornerCount;
            }
            for (int i = 0; i < NodeMass.Length; i++)
            {
                NodeMass[i] = math.max(NodeMass[i], MinNodeMass);
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

    /// <summary>
    /// Updates the topology built by <see cref="TopologyJob"/> after edges were only severed since, with the same result
    /// as rebuilding it (TODO PERF-5). Glue only joins the slots of one grid point, so only the nodes at the corners of the
    /// cells whose severed edges changed can split, and severing never joins anything: the copies of the changed cells
    /// are rebuilt from their components, the slots at each of their corners are united again from the copies of the
    /// cells around that grid point alone, and everything else is copied. Copies keep their order, and nodes the order
    /// of their first slots, as in the rebuild; masses are summed over the slots in the same order. The copies of the
    /// unchanged cells and their nodes are written by <see cref="RemapCopyNodesJob"/>, the slots of the split nodes by
    /// <see cref="SetSplitSlotsJob"/>. <see cref="Result"/>[0] is 0 when an edge was restored (<see cref="HealStrokeJob"/>)
    /// or a split node has no old node; the topology must then be rebuilt.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct TopologyUpdateJob : IJob
    {
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int> GridToCell;
        [ReadOnly] public NativeArray<float> CellFill;
        [ReadOnly] public NativeArray<ushort> Severed;
        /// <summary>Severed edges the old topology was built from.</summary>
        [ReadOnly] public NativeArray<ushort> OldSevered;
        [ReadOnly] public NativeArray<byte> ComponentCounts;
        [ReadOnly] public NativeArray<byte> ComponentMasks;
        [ReadOnly] public NativeArray<int3> Directions;
        [ReadOnly] public NativeArray<int> PairStart;
        [ReadOnly] public NativeArray<int> Pairs;
        public int3 Dims;
        public float CellMass;
        public float MinNodeMass;

        [ReadOnly] public NativeArray<int2> OldRange;
        [ReadOnly] public NativeArray<CellCopy> OldCopies;
        [ReadOnly] public NativeArray<int> OldCopyNodes;
        [ReadOnly] public NativeArray<float3> OldRest;
        [ReadOnly] public NativeArray<float> OldNodeMass;
        [ReadOnly] public NativeArray<int> OldFirstSlot;

        public NativeArray<int2> NewRange;
        public NativeList<CellCopy> NewCopies;
        public NativeList<int> NewCopyNodes;
        public NativeList<int> NewCopyParent;
        public NativeList<int> NodeParent;
        public NativeList<float3> NodeRest;
        public NativeList<float> NodeMass;
        public NativeList<int> FirstSlot;
        /// <summary>Per cell: 1 when its severed edges changed.</summary>
        public NativeArray<byte> CellChanged;
        /// <summary>Per old node: its new node, or -1 when it was at a corner of a changed cell (see <see cref="SplitSlots"/>).</summary>
        public NativeArray<int> OldToNew;
        /// <summary>(slot, new node) of every slot at the corners of the changed cells.</summary>
        public NativeList<int2> SplitSlots;
        /// <summary>[0]: 1 when updated, 0 when the topology must be rebuilt.</summary>
        public NativeArray<int> Result;

        /// <summary>A node at a corner of a changed cell: its first slot, old node, mass and slots (in Slots, from SlotStart).</summary>
        private struct SplitNode : System.IComparable<SplitNode>
        {
            public int FirstSlot;
            public int Parent;
            public float Mass;
            public int SlotStart;
            public int SlotCount;

            public int CompareTo(SplitNode other) => FirstSlot.CompareTo(other.FirstSlot);
        }

        public void Execute()
        {
            Result[0] = 0;
            NewCopies.Clear();
            NewCopyParent.Clear();
            NodeParent.Clear();
            NodeRest.Clear();
            NodeMass.Clear();
            FirstSlot.Clear();
            SplitSlots.Clear();

            var changed = new NativeList<int>(64, Allocator.Temp);
            for (int c = 0; c < CellCoord.Length; c++)
            {
                int before = OldSevered[c], now = Severed[c];
                CellChanged[c] = (byte)(before != now ? 1 : 0);
                if (before == now)
                {
                    continue;
                }
                if ((before & ~now) != 0)
                {
                    return;
                }
                changed.Add(c);
            }

            // Copies in the order of TopologyJob; those of an unchanged cell are its old ones.
            int copyCount = OldCopies.Length;
            for (int i = 0; i < changed.Length; i++)
            {
                copyCount += ComponentCounts[Severed[changed[i]]] - OldRange[changed[i]].y;
            }
            NewCopies.ResizeUninitialized(copyCount);
            NewCopyParent.ResizeUninitialized(copyCount);
            NativeArray<CellCopy> copies = NewCopies.AsArray();
            NativeArray<int> copyParent = NewCopyParent.AsArray();
            int copy = 0;
            for (int c = 0; c < CellCoord.Length; c++)
            {
                int2 old = OldRange[c];
                if (CellChanged[c] == 0)
                {
                    NewRange[c] = new int2(copy, old.y);
                    for (int q = old.x; q < old.x + old.y; q++)
                    {
                        copies[copy] = OldCopies[q];
                        copyParent[copy++] = q;
                    }
                    continue;
                }
                int severed = Severed[c];
                int count = ComponentCounts[severed];
                NewRange[c] = new int2(copy, count);
                for (int i = 0; i < count; i++)
                {
                    byte mask = ComponentMasks[severed * Lattice.CornerCount + i];
                    int parent = -1;
                    for (int q = old.x; q < old.x + old.y && parent < 0; q++)
                    {
                        if ((OldCopies[q].Mask & mask) == mask)
                        {
                            parent = q;
                        }
                    }
                    copies[copy] = new CellCopy
                    {
                        Cell = c,
                        Mask = mask,
                        Weight = CellFill[c] * math.countbits((uint)mask) / 8.0f,
                    };
                    copyParent[copy++] = parent;
                }
            }

            int3 pointDims = Dims + 1;
            var points = new NativeList<int>(changed.Length * Lattice.CornerCount, Allocator.Temp);
            for (int i = 0; i < changed.Length; i++)
            {
                int3 coord = CellCoord[changed[i]];
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int3 p = coord + Lattice.CornerOffset(k);
                    points.Add(p.x + pointDims.x * (p.y + pointDims.y * p.z));
                }
            }
            points.Sort();

            var split = new NativeArray<byte>(OldNodeMass.Length, Allocator.Temp);
            int splitCount = 0;
            var nodes = new NativeList<SplitNode>(points.Length, Allocator.Temp);
            var nodeSlots = new NativeList<int>(points.Length * 2, Allocator.Temp);
            // Per grid point: the cell having it as corner k, the slots of their copies and the union of those slots.
            var cellAt = new FixedList64Bytes<int>();
            var slotStart = new FixedList64Bytes<int>();
            var slots = new FixedList512Bytes<int>();
            var root = new FixedList512Bytes<int>();
            for (int i = 0; i < points.Length; i++)
            {
                if (i > 0 && points[i] == points[i - 1])
                {
                    continue;
                }
                int index = points[i];
                var point = new int3(index % pointDims.x, index / pointDims.x % pointDims.y, index / (pointDims.x * pointDims.y));
                cellAt.Clear();
                slotStart.Clear();
                slots.Clear();
                root.Clear();
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int3 cc = point - Lattice.CornerOffset(k);
                    int cell = math.any(cc < 0) || math.any(cc >= Dims) ? -1 : GridToCell[cc.x + Dims.x * (cc.y + Dims.y * cc.z)];
                    cellAt.Add(cell);
                    slotStart.Add(slots.Length);
                    if (cell < 0)
                    {
                        continue;
                    }
                    int2 range = NewRange[cell];
                    for (int q = range.x; q < range.x + range.y; q++)
                    {
                        root.Add(slots.Length);
                        slots.Add(q * Lattice.CornerCount + k);
                    }
                    int2 old = OldRange[cell];
                    for (int q = old.x; q < old.x + old.y; q++)
                    {
                        int node = OldCopyNodes[q * Lattice.CornerCount + k];
                        splitCount += split[node] == 0 ? 1 : 0;
                        split[node] = 1;
                    }
                }

                for (int ka = 0; ka < Lattice.CornerCount; ka++)
                {
                    for (int kb = ka + 1; kb < Lattice.CornerCount; kb++)
                    {
                        if (cellAt[ka] < 0 || cellAt[kb] < 0)
                        {
                            continue;
                        }
                        // The cell with the point at corner kb lies at offset(ka) - offset(kb) from the one with it at ka;
                        // TopologyJob glues them from the cell the direction is positive from.
                        int3 direction = Lattice.CornerOffset(ka) - Lattice.CornerOffset(kb);
                        int lower = ka, upper = kb;
                        if (!Positive(direction))
                        {
                            direction = -direction;
                            lower = kb;
                            upper = ka;
                        }
                        int d = 0;
                        while (math.any(Directions[d] != direction))
                        {
                            d++;
                        }
                        int2 rl = NewRange[cellAt[lower]], ru = NewRange[cellAt[upper]];
                        for (int qa = rl.x; qa < rl.x + rl.y; qa++)
                        {
                            int ma = copies[qa].Mask;
                            for (int qb = ru.x; qb < ru.x + ru.y; qb++)
                            {
                                int mb = copies[qb].Mask;
                                bool glue = false;
                                for (int p = PairStart[d]; p < PairStart[d + 1] && !glue; p++)
                                {
                                    glue = ((ma >> Pairs[2 * p]) & 1) != 0 && ((mb >> Pairs[2 * p + 1]) & 1) != 0;
                                }
                                if (glue)
                                {
                                    Union(ref root, slotStart[lower] + qa - rl.x, slotStart[upper] + qb - ru.x);
                                }
                            }
                        }
                    }
                }

                for (int r = 0; r < slots.Length; r++)
                {
                    if (Find(ref root, r) != r)
                    {
                        continue;
                    }
                    // Slots in increasing order, the order TopologyJob visits them in.
                    int start = nodeSlots.Length;
                    for (int s = 0; s < slots.Length; s++)
                    {
                        if (Find(ref root, s) != r)
                        {
                            continue;
                        }
                        int slot = slots[s];
                        int j = nodeSlots.Length;
                        nodeSlots.Add(slot);
                        while (j > start && nodeSlots[j - 1] > slot)
                        {
                            nodeSlots[j] = nodeSlots[j - 1];
                            j--;
                        }
                        nodeSlots[j] = slot;
                    }
                    float mass = 0.0f;
                    for (int s = start; s < nodeSlots.Length; s++)
                    {
                        mass += CellMass * copies[nodeSlots[s] / Lattice.CornerCount].Weight / Lattice.CornerCount;
                    }
                    int first = nodeSlots[start];
                    int parentCopy = copyParent[first / Lattice.CornerCount];
                    if (parentCopy < 0)
                    {
                        return;
                    }
                    nodes.Add(new SplitNode
                    {
                        FirstSlot = first,
                        Parent = OldCopyNodes[parentCopy * Lattice.CornerCount + first % Lattice.CornerCount],
                        Mass = math.max(mass, MinNodeMass),
                        SlotStart = start,
                        SlotCount = nodeSlots.Length - start,
                    });
                }
            }
            nodes.Sort();

            // Nodes in the order of their first slots: the old nodes elsewhere keep theirs, moved with their copies.
            int nodeCount = OldNodeMass.Length - splitCount + nodes.Length;
            NodeParent.ResizeUninitialized(nodeCount);
            NodeRest.ResizeUninitialized(nodeCount);
            NodeMass.ResizeUninitialized(nodeCount);
            FirstSlot.ResizeUninitialized(nodeCount);
            int id = 0, next = 0;
            for (int o = 0; o < OldNodeMass.Length; o++)
            {
                if (split[o] != 0)
                {
                    OldToNew[o] = -1;
                    continue;
                }
                int oldFirst = OldFirstSlot[o];
                int oldCopy = oldFirst / Lattice.CornerCount;
                int cell = OldCopies[oldCopy].Cell;
                int first = (oldCopy - OldRange[cell].x + NewRange[cell].x) * Lattice.CornerCount + oldFirst % Lattice.CornerCount;
                while (next < nodes.Length && nodes[next].FirstSlot < first)
                {
                    SetSplitNode(id++, nodes[next++], nodeSlots);
                }
                OldToNew[o] = id;
                NodeParent[id] = o;
                NodeRest[id] = OldRest[o];
                NodeMass[id] = OldNodeMass[o];
                FirstSlot[id++] = first;
            }
            while (next < nodes.Length)
            {
                SetSplitNode(id++, nodes[next++], nodeSlots);
            }
            NewCopyNodes.ResizeUninitialized(copyCount * Lattice.CornerCount);
            Result[0] = 1;
        }

        private void SetSplitNode(int id, SplitNode node, NativeList<int> nodeSlots)
        {
            NodeParent[id] = node.Parent;
            NodeRest[id] = OldRest[node.Parent];
            NodeMass[id] = node.Mass;
            FirstSlot[id] = node.FirstSlot;
            for (int s = node.SlotStart; s < node.SlotStart + node.SlotCount; s++)
            {
                SplitSlots.Add(new int2(nodeSlots[s], id));
            }
        }

        /// <summary>Whether <paramref name="d"/> is one of the 13 directions of <see cref="LatticeTables.Directions"/>.</summary>
        private static bool Positive(int3 d) => d.z > 0 || (d.z == 0 && (d.y > 0 || (d.y == 0 && d.x > 0)));

        private static int Find(ref FixedList512Bytes<int> root, int x)
        {
            while (root[x] != x)
            {
                root[x] = root[root[x]];
                x = root[x];
            }
            return x;
        }

        private static void Union(ref FixedList512Bytes<int> root, int a, int b)
        {
            a = Find(ref root, a);
            b = Find(ref root, b);
            root[math.max(a, b)] = math.min(a, b);
        }
    }

    /// <summary>
    /// Nodes of the copies <see cref="TopologyUpdateJob"/> kept: the new nodes of their old ones. Slots at the corners of
    /// the changed cells are -1 here and set by <see cref="SetSplitSlotsJob"/>.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RemapCopyNodesJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyParent;
        [ReadOnly] public NativeArray<byte> CellChanged;
        [ReadOnly] public NativeArray<int> OldCopyNodes;
        [ReadOnly] public NativeArray<int> OldToNew;
        [NativeDisableParallelForRestriction] public NativeArray<int> CopyNodes;

        public void Execute(int q)
        {
            bool changed = CellChanged[Copies[q].Cell] != 0;
            int parent = CopyParent[q];
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                CopyNodes[q * Lattice.CornerCount + k] = changed ? -1 : OldToNew[OldCopyNodes[parent * Lattice.CornerCount + k]];
            }
        }
    }

    /// <summary>Sets the slots of the nodes at the corners of the changed cells (<see cref="TopologyUpdateJob.SplitSlots"/>).</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct SetSplitSlotsJob : IJob
    {
        [ReadOnly] public NativeArray<int2> Slots;
        public NativeArray<int> CopyNodes;

        public void Execute()
        {
            for (int i = 0; i < Slots.Length; i++)
            {
                CopyNodes[Slots[i].x] = Slots[i].y;
            }
        }
    }

    /// <summary>
    /// Tet rotations of the new copies: those of the copy each derives from, so the polar decomposition stays warm across
    /// cuts, or <see cref="Initial"/>.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RemapRotationsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> CopyParent;
        /// <summary>Rotations of the old copies; empty when there are none.</summary>
        [ReadOnly] public NativeArray<quaternion> OldRotations;
        public quaternion Initial;
        [NativeDisableParallelForRestriction] public NativeArray<quaternion> Rotations;

        public void Execute(int q)
        {
            int parent = CopyParent[q];
            for (int t = 0; t < Lattice.TetsPerCell; t++)
            {
                Rotations[q * Lattice.TetsPerCell + t] = parent >= 0 && OldRotations.Length > 0
                    ? OldRotations[parent * Lattice.TetsPerCell + t]
                    : Initial;
            }
        }
    }

    /// <summary>
    /// Marks nodes that own material and joins the real corners of every copy into a piece, copies in parallel. Roots
    /// are linked by compare-and-swap, always the larger index under the smaller, so every piece ends with its least
    /// node as its root whatever the order of the joins, as with a sequential union (<see cref="PieceLabelJob"/>).
    /// <see cref="Root"/> starts with every node its own root.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal unsafe struct PieceUnionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [NativeDisableParallelForRestriction] public NativeArray<byte> NodeReal;
        [NativeDisableParallelForRestriction] public NativeArray<int> Root;

        public void Execute(int q)
        {
            var root = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(Root);
            int mask = Copies[q].Mask;
            int first = -1;
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                if (((mask >> k) & 1) == 0)
                {
                    continue;
                }
                int n = CopyNodes[q * Lattice.CornerCount + k];
                NodeReal[n] = 1;
                if (first < 0)
                {
                    first = n;
                }
                else
                {
                    Union(root, first, n);
                }
            }
        }

        private static void Union(int* root, int a, int b)
        {
            while (true)
            {
                a = Find(root, a);
                b = Find(root, b);
                if (a == b)
                {
                    return;
                }
                if (a < b)
                {
                    int t = a;
                    a = b;
                    b = t;
                }
                if (Interlocked.CompareExchange(ref root[a], b, a) == a)
                {
                    return;
                }
            }
        }

        /// <summary>Root of <paramref name="x"/>, halving the path: entries other than roots only ever move to an ancestor.</summary>
        private static int Find(int* root, int x)
        {
            while (true)
            {
                int parent = Volatile.Read(ref root[x]);
                if (parent == x)
                {
                    return x;
                }
                int grand = Volatile.Read(ref root[parent]);
                if (grand != parent)
                {
                    Volatile.Write(ref root[x], grand);
                }
                x = grand;
            }
        }
    }

    /// <summary>Labels every real node with the least node of its piece (<see cref="PieceUnionJob"/>), -1 for virtual-only nodes.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct PieceLabelJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<byte> NodeReal;
        [ReadOnly] public NativeArray<int> Root;
        public NativeArray<int> PieceId;

        public void Execute(int i)
        {
            int x = i;
            while (Root[x] != x)
            {
                x = Root[x];
            }
            PieceId[i] = NodeReal[i] != 0 ? x : -1;
        }
    }

    /// <summary>Every node its own root, for <see cref="PieceUnionJob"/>.</summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct PieceRootsJob : IJobParallelFor
    {
        public NativeArray<int> Root;

        public void Execute(int i) => Root[i] = i;
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
    /// Severs every intact lattice edge crossed by the swept blade surface: the quad of this step
    /// (a0,b0,b1,a1) and the tail quad re-sweeping the end of the previous step's path; the edges
    /// outside the surface beyond the tip or heel of a blade that has left the material through it
    /// (<see cref="OutsideCrossing"/>); and records Hermite data (edge parameter and rest-space
    /// normal) on all cells sharing the edge. Within a stroke, nodes are labelled with the side of
    /// the blade they were on when the blade passed them, and an edge whose ends both carry a label
    /// is severed exactly when the labels differ.
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
        /// <summary>Per cell: 1 when its copies come within <see cref="Band"/> of the swept blade's bounds (<see cref="CutCellsJob"/>).</summary>
        [ReadOnly] public NativeArray<byte> Near;
        public int3 Dims;
        /// <summary>
        /// Blade edge (heel A, tip B) at the start of the tail, at the previous step and at this step.
        /// The tail quad (TailA, TailB) to (A0, B0) lies on the previous step's path, the quad
        /// (A0, B0) to (A1, B1) is this step's; each has its own plane.
        /// </summary>
        public float3 TailA, TailB, A0, B0, A1, B1;

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
        /// <summary>The same for the tail quad.</summary>
        public float TailOrientation;

        public NativeArray<ushort> Severed;
        public NativeArray<float> EdgeT;
        public NativeArray<float3> EdgeNormal;
        public NativeArray<int> SeveredCount;
        /// <summary>Severed edges when the stroke began: the cut surfaces of earlier strokes.</summary>
        [ReadOnly] public NativeArray<ushort> PriorSevered;
        /// <summary>Edges the stroke keeps intact (<see cref="KeepChips"/>); they are not cut again in the stroke.</summary>
        public NativeArray<ushort> Kept;

        /// <summary>Normals of the swept triangles (see <see cref="Triangle"/>), not normalized.</summary>
        private float3 _n0, _n1, _n2, _n3;
        private float3 _tailNormal, _bladeNormal;
        /// <summary>Direction the blade moves in this step.</summary>
        private float3 _motion;

        public void Execute()
        {
            _n0 = math.cross(TailB - TailA, B0 - TailA);
            _n1 = math.cross(B0 - TailA, A0 - TailA);
            _n2 = math.cross(B0 - A0, B1 - A0);
            _n3 = math.cross(B1 - A0, A1 - A0);
            _bladeNormal = math.normalizesafe(_n2 + _n3);
            if (math.lengthsq(_bladeNormal) < 0.5f)
            {
                return;
            }
            _tailNormal = math.normalizesafe(_n0 + _n1, _bladeNormal);
            _motion = math.normalizesafe((A1 + B1) - (A0 + B0));

            // Label the nodes near the blade first, then test the edges of the same cells: an edge whose
            // ends were labelled in different sweeps may have moved off the blade by now.
            var cut = new NativeList<int>(256, Allocator.Temp);
            for (int c = 0; c < CellCoord.Length; c++)
            {
                if (Near[c] == 0)
                {
                    continue;
                }
                int2 range = CellCopyRange[c];
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = Copies[q].Mask;
                    for (int k = 0; k < 8; k++)
                    {
                        if (((mask >> k) & 1) != 0)
                        {
                            Label(CopyNodes[q * 8 + k]);
                        }
                    }
                }
                cut.Add(c);
            }

            int severedCount = 0;
            var fresh = new NativeList<int3>(64, Allocator.Temp);
            for (int i = 0; i < cut.Length; i++)
            {
                int c = cut[i];
                int2 range = CellCopyRange[c];
                LabelBetweenSides(c, range);
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if (((Severed[c] | Kept[c]) & (1 << e)) != 0)
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
                    // This step's quad first, then the tail.
                    float t = -1.0f;
                    int crossed = -1;
                    for (int j = 2; j < 2 + TriangleCount && t < 0.0f; j++)
                    {
                        crossed = j % TriangleCount;
                        Triangle(crossed, out float3 ta, out float3 tb, out float3 tc, out float3 tn);
                        t = Crossing(p, r, ta, tb, tc, tn);
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
                        if (t < 0.0f)
                        {
                            t = LabelCrossing(np, nr);
                            crossed = Nearest(math.lerp(p, r, t));
                        }
                    }
                    else if (t < 0.0f)
                    {
                        t = OutsideCrossing(c, k0, k1, p, r);
                        crossed = 2;
                    }
                    if (t < 0.0f)
                    {
                        continue;
                    }
                    t = math.clamp(t, 0.05f, 0.95f);
                    float3 restNormal = RestNormal(owner, k0, k1, t, crossed < 2 ? _tailNormal : _bladeNormal);
                    SeverShared(c, e, t, restNormal);
                    fresh.Add(new int3(c, e, owner));
                    severedCount++;
                }
            }
            cut.Dispose();
            severedCount -= KeepChips(fresh);
            fresh.Dispose();
            SeveredCount[0] = severedCount;
        }

        /// <summary>
        /// Labels, by its current side, an unlabelled node joined by intact edges of the cell to nodes
        /// of both labels. Tissue the blade drags along can leave such a node outside the band, or
        /// where it does not project onto the swept surface; through it the two sides would stay
        /// joined. Nodes next to one label only stay unlabelled, so a partial incision does not grow
        /// past the blade. A node ahead of the blade waits until the blade passes it: the plane of
        /// this sweep does not tell its side where the path turns before reaching it.
        /// </summary>
        private void LabelBetweenSides(int cell, int2 range)
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
                    float3 x = Positions[node];
                    if (!plus || !minus || math.dot(x - 0.5f * (A1 + B1), _motion) > 0.0f)
                    {
                        continue;
                    }
                    if (!ProjectedDistance(x, out float d))
                    {
                        d = SignedDistance(x, Nearest(x));
                    }
                    StrokeSide[node] = (sbyte)(d >= 0.0f ? 1 : -1);
                    StrokeTravel[node] = Travel;
                    StrokeDistance[node] = d;
                }
            }
        }

        /// <summary>The node's label of this stroke, or 0 when the blade has not passed it or the label is old.</summary>
        private int Side(int node) =>
            StrokeSide[node] != 0 && Travel - StrokeTravel[node] <= LabelTravel ? StrokeSide[node] : 0;

        /// <summary>
        /// Labels a node within <see cref="Band"/> of the blade whose projection lies on the swept
        /// surface, by the nearest triangle it projects onto. Where the path turns, the planes of the
        /// tail and of this step's quad can put a node on different sides; the nearer one is the
        /// side of the path.
        /// </summary>
        private void Label(int node)
        {
            if (Side(node) != 0)
            {
                return;
            }
            if (ProjectedDistance(Positions[node], out float d) && math.abs(d) <= Band)
            {
                // An end on the blade counts as the positive side, as in Crossing.
                StrokeSide[node] = (sbyte)(d >= 0.0f ? 1 : -1);
                StrokeTravel[node] = Travel;
                StrokeDistance[node] = d;
            }
        }

        private const int TriangleCount = 4;

        /// <summary>Swept triangle i: 0 and 1 make the tail quad, 2 and 3 this step's quad.</summary>
        private void Triangle(int i, out float3 a, out float3 b, out float3 c, out float3 n)
        {
            switch (i)
            {
                case 0:
                    a = TailA; b = TailB; c = B0; n = _n0;
                    break;
                case 1:
                    a = TailA; b = B0; c = A0; n = _n1;
                    break;
                case 2:
                    a = A0; b = B0; c = B1; n = _n2;
                    break;
                default:
                    a = A0; b = B1; c = A1; n = _n3;
                    break;
            }
        }

        /// <summary>
        /// Signed distance of x from the plane of the nearest swept triangle that x projects onto;
        /// false when it projects onto none.
        /// </summary>
        private bool ProjectedDistance(float3 x, out float d)
        {
            d = float.MaxValue;
            bool found = false;
            for (int i = 0; i < TriangleCount; i++)
            {
                Triangle(i, out float3 a, out float3 b, out float3 c, out float3 n);
                if (math.lengthsq(n) <= 0.0f || !Geometry.PointInTriangle(x, a, b, c, n))
                {
                    continue;
                }
                float di = SignedDistance(x, i);
                d = math.abs(di) < math.abs(d) ? di : d;
                found = true;
            }
            return found;
        }

        /// <summary>Distance of x from the plane of swept triangle i, signed by the stroke's orientation.</summary>
        private float SignedDistance(float3 x, int i)
        {
            Triangle(i, out float3 a, out _, out _, out float3 n);
            float length = math.length(n);
            return length > 0.0f ? math.dot(x - a, n) / length * (i < 2 ? TailOrientation : Orientation) : 0.0f;
        }

        /// <summary>The swept triangle nearest to x.</summary>
        private int Nearest(float3 x)
        {
            int nearest = 2;
            float best = float.MaxValue;
            for (int i = 0; i < TriangleCount; i++)
            {
                Triangle(i, out float3 a, out float3 b, out float3 c, out float3 n);
                if (math.lengthsq(n) <= 0.0f)
                {
                    continue;
                }
                float distance = math.distancesq(x, Geometry.ClosestPointOnTriangle(x, a, b, c));
                if (distance < best)
                {
                    best = distance;
                    nearest = i;
                }
            }
            return nearest;
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
        private float OutsideCrossing(int cell, int k0, int k1, float3 p, float3 r)
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
            float3 n1 = _n2, n2 = _n3;
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
                // Only beyond the tip or heel, and only in the part swept in this step (this step's
                // quad, without the tail). Tissue that moves across the plane between two steps would
                // otherwise be cut on both sides and leave single-node fragments.
                float3 normal = n1 + n2;
                if (math.dot(math.cross(B0 - A0, x - A0), normal) < 0.0f ||
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

        /// <summary>
        /// Fewest real nodes of a part a later stroke may cut off next to an earlier cut surface: twice the least
        /// piece that is rendered (<see cref="SurfaceMesh.MinVisiblePieceNodes"/>). A sliver shaved off a wall is one
        /// layer of nodes and a cell or two long; a strip cut off along the whole incision has many more.
        /// </summary>
        private const int ChipNodes = 2 * SurfaceMesh.MinVisiblePieceNodes;

        /// <summary>
        /// Restores the edges this sweep severed around a part of fewer than <see cref="ChipNodes"/> real nodes
        /// next to the cut surface of an earlier stroke, and keeps them for the rest of the stroke; returns their
        /// number. A blade drawn again along an incision, or beside it, passes through its walls where they have
        /// moved since and shaves off slivers thinner than a cell. The lattice holds such a sliver as a few nodes
        /// with mostly virtual corners: it flaps about or is too small to render, and the planes and copies around
        /// it left holes and stretched skin in the walls (TODO BUG-15). Parts cut off by the stroke alone are left alone.
        /// </summary>
        private int KeepChips(NativeList<int3> fresh)
        {
            if (fresh.Length == 0)
            {
                return 0;
            }
            // Per node: 1 in a chip, 2 in a part that is no chip.
            var marks = new NativeArray<byte>(Positions.Length, Allocator.Temp);
            var part = new FixedList512Bytes<int4>();
            bool any = false;
            for (int i = 0; i < fresh.Length; i++)
            {
                int3 f = fresh[i];
                for (int end = 0; end < 2; end++)
                {
                    int k = EdgeCorners[2 * f.y + end];
                    int node = CopyNodes[f.z * 8 + k];
                    if (marks[node] != 0)
                    {
                        continue;
                    }
                    bool chip = FloodPart(node, CellCoord[f.x] + Lattice.CornerOffset(k), ref part);
                    for (int j = 0; j < part.Length; j++)
                    {
                        marks[part[j].x] = (byte)(chip ? 1 : 2);
                    }
                    any |= chip;
                }
            }
            int restored = 0;
            if (any)
            {
                for (int i = 0; i < fresh.Length; i++)
                {
                    int3 f = fresh[i];
                    int np = CopyNodes[f.z * 8 + EdgeCorners[2 * f.y]];
                    int nr = CopyNodes[f.z * 8 + EdgeCorners[2 * f.y + 1]];
                    if (marks[np] == 1 || marks[nr] == 1)
                    {
                        KeepShared(f.x, f.y);
                        restored++;
                    }
                }
            }
            marks.Dispose();
            return restored;
        }

        /// <summary>
        /// Collects the real nodes joined to <paramref name="node"/> (at grid point <paramref name="point"/>) by
        /// intact edges, (node, grid point) per entry, until there are <see cref="ChipNodes"/>. True when the part
        /// has fewer nodes and one of them ends an edge severed before the stroke.
        /// </summary>
        private bool FloodPart(int node, int3 point, ref FixedList512Bytes<int4> part)
        {
            part.Clear();
            part.Add(new int4(node, point));
            bool prior = false;
            for (int head = 0; head < part.Length; head++)
            {
                int n = part[head].x;
                int3 p = part[head].yzw;
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int3 cc = p - Lattice.CornerOffset(k);
                    if (math.any(cc < 0) || math.any(cc >= Dims))
                    {
                        continue;
                    }
                    int cell = GridToCell[cc.x + Dims.x * (cc.y + Dims.y * cc.z)];
                    if (cell < 0)
                    {
                        continue;
                    }
                    int2 range = CellCopyRange[cell];
                    int copy = -1;
                    for (int q = range.x; q < range.x + range.y && copy < 0; q++)
                    {
                        if (((Copies[q].Mask >> k) & 1) != 0 && CopyNodes[q * 8 + k] == n)
                        {
                            copy = q;
                        }
                    }
                    if (copy < 0)
                    {
                        continue;
                    }
                    int mask = Copies[copy].Mask;
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        int k0 = EdgeCorners[2 * e], k1 = EdgeCorners[2 * e + 1];
                        int other = k0 == k ? k1 : k1 == k ? k0 : -1;
                        if (other < 0)
                        {
                            continue;
                        }
                        prior |= (PriorSevered[cell] & (1 << e)) != 0;
                        if ((Severed[cell] & (1 << e)) != 0 || ((mask >> other) & 1) == 0)
                        {
                            continue;
                        }
                        int m = CopyNodes[copy * 8 + other];
                        bool seen = false;
                        for (int j = 0; j < part.Length && !seen; j++)
                        {
                            seen = part[j].x == m;
                        }
                        if (seen)
                        {
                            continue;
                        }
                        part.Add(new int4(m, cc + Lattice.CornerOffset(other)));
                        if (part.Length >= ChipNodes)
                        {
                            return false;
                        }
                    }
                }
            }
            return prior;
        }

        private void SeverShared(int cell, int edge, float t, float3 restNormal) => MarkShared(cell, edge, false, t, restNormal);

        /// <summary>Restores a severed edge on all cells sharing it and keeps it for the rest of the stroke.</summary>
        private void KeepShared(int cell, int edge) => MarkShared(cell, edge, true, 0.0f, float3.zero);

        private void MarkShared(int cell, int edge, bool keep, float t, float3 restNormal)
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
                    if (keep)
                    {
                        Severed[n] = (ushort)(Severed[n] & ~(1 << localEdge));
                        Kept[n] = (ushort)(Kept[n] | (1 << localEdge));
                        continue;
                    }
                    Severed[n] = (ushort)(Severed[n] | (1 << localEdge));
                    EdgeT[n * 12 + localEdge] = t;
                    EdgeNormal[n * 12 + localEdge] = restNormal;
                }
            }
        }
    }

    /// <summary>
    /// Marks the cells whose copies come within <see cref="Band"/> of the bounding box of the blade's tail and quad, the
    /// cells <see cref="CutJob"/> labels and cuts: the test reads every node of every cell, in parallel here.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CutCellsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<int> CopyNodes;
        public float3 TailA, TailB, A0, B0, A1, B1;
        public float Band;
        public NativeArray<byte> Near;

        public void Execute(int c)
        {
            float3 lo = math.min(math.min(math.min(TailA, TailB), math.min(A0, B0)), math.min(A1, B1));
            float3 hi = math.max(math.max(math.max(TailA, TailB), math.max(A0, B0)), math.max(A1, B1));
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
            Near[c] = (byte)(math.any(cmax < lo - Band) || math.any(cmin > hi + Band) ? 0 : 1);
        }
    }

    /// <summary>
    /// At the end of a stroke, restores the edges it severed alongside the cut surface of an earlier stroke without
    /// separating anything: a severed edge whose ends are in one piece, with a node at an end lying between its cut
    /// point and the Hermite point of an edge that was severed before the stroke and shares that node, the two less than
    /// <see cref="MaxGap"/> apart along the normal of the earlier edge and the normals within <see cref="MinAlignment"/>.
    /// A blade drawn beside an incision whose walls have moved since slices into a wall a fraction of a cell from it and
    /// leaves flaps of the wall attached along one side; they turned and swung apart, and the wall showed slits
    /// (TODO BUG-16). Only the end of the stroke tells: a strip cut off along the whole incision is such a flap until the
    /// blade has passed its far end. A node between the two cut points rules out a stroke that continues an earlier cut
    /// in its own plane, and one that starts from an incision at a right angle.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct HealStrokeJob : IJob
    {
        [ReadOnly] public NativeArray<int3> CellCoord;
        [ReadOnly] public NativeArray<int> GridToCell;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<int> PieceId;
        [ReadOnly] public NativeArray<int> EdgeCorners;
        /// <summary>Severed edges when the stroke began.</summary>
        [ReadOnly] public NativeArray<ushort> StrokeStartSevered;
        [ReadOnly] public NativeArray<float> EdgeT;
        [ReadOnly] public NativeArray<float3> EdgeNormal;
        public int3 Dims;
        /// <summary>Largest distance between the two cuts, in cells.</summary>
        public float MaxGap;
        /// <summary>Least |cos| between the normals of the two cuts.</summary>
        public float MinAlignment;
        public NativeArray<ushort> Severed;
        /// <summary>[0]: number of restored edges.</summary>
        public NativeArray<int> Healed;

        public void Execute()
        {
            int healed = 0;
            for (int c = 0; c < CellCoord.Length; c++)
            {
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    // Edges restored from a cell sharing them are no longer severed here.
                    if (((Severed[c] & ~StrokeStartSevered[c]) & (1 << e)) == 0 || Separates(c, e) || !Alongside(c, e))
                    {
                        continue;
                    }
                    RestoreShared(c, e);
                    healed++;
                }
            }
            Healed[0] = healed;
        }

        /// <summary>Whether the copies owning the two corners of the edge belong to different pieces.</summary>
        private bool Separates(int cell, int edge)
        {
            int2 range = CellCopyRange[cell];
            int k0 = EdgeCorners[2 * edge], k1 = EdgeCorners[2 * edge + 1];
            int p0 = -1, p1 = -1;
            for (int q = range.x; q < range.x + range.y; q++)
            {
                int mask = Copies[q].Mask;
                if (((mask >> k0) & 1) != 0)
                {
                    p0 = PieceId[CopyNodes[q * 8 + k0]];
                }
                if (((mask >> k1) & 1) != 0)
                {
                    p1 = PieceId[CopyNodes[q * 8 + k1]];
                }
            }
            return p0 != p1;
        }

        private bool Alongside(int cell, int edge)
        {
            int3 coord = CellCoord[cell];
            int k0 = EdgeCorners[2 * edge], k1 = EdgeCorners[2 * edge + 1];
            // Positions in cells of the rest lattice.
            float3 x = (float3)coord + math.lerp((float3)Lattice.CornerOffset(k0), Lattice.CornerOffset(k1),
                EdgeT[cell * Lattice.EdgeCount + edge]);
            float3 normal = EdgeNormal[cell * Lattice.EdgeCount + edge];
            for (int end = 0; end < 2; end++)
            {
                int3 node = coord + Lattice.CornerOffset(end == 0 ? k0 : k1);
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    int3 cc = node - Lattice.CornerOffset(k);
                    if (math.any(cc < 0) || math.any(cc >= Dims))
                    {
                        continue;
                    }
                    int other = GridToCell[cc.x + Dims.x * (cc.y + Dims.y * cc.z)];
                    if (other < 0 || StrokeStartSevered[other] == 0)
                    {
                        continue;
                    }
                    for (int f = 0; f < Lattice.EdgeCount; f++)
                    {
                        int a = EdgeCorners[2 * f], b = EdgeCorners[2 * f + 1];
                        if ((a != k && b != k) || (StrokeStartSevered[other] & (1 << f)) == 0)
                        {
                            continue;
                        }
                        float3 earlier = EdgeNormal[other * Lattice.EdgeCount + f];
                        if (math.abs(math.dot(earlier, normal)) < MinAlignment)
                        {
                            continue;
                        }
                        float3 hermite = (float3)cc + math.lerp((float3)Lattice.CornerOffset(a), Lattice.CornerOffset(b),
                            EdgeT[other * Lattice.EdgeCount + f]);
                        float toCut = math.dot(earlier, x - (float3)node);
                        float toEarlier = math.dot(earlier, hermite - (float3)node);
                        if (toCut * toEarlier < 0.0f && math.abs(toCut - toEarlier) < MaxGap)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private void RestoreShared(int cell, int edge)
        {
            int axis = edge / 4;
            int3 start = CellCoord[cell] + Lattice.CornerOffset(EdgeCorners[2 * edge]);
            int axisB = (axis + 1) % 3, axisC = (axis + 2) % 3;
            for (int ob = 0; ob < 2; ob++)
            {
                for (int oc = 0; oc < 2; oc++)
                {
                    int3 offset = int3.zero;
                    offset[axisB] = ob;
                    offset[axisC] = oc;
                    int3 nc = start - offset;
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
                    for (int e = axis * 4; e < axis * 4 + 4; e++)
                    {
                        if (EdgeCorners[2 * e] == localStart)
                        {
                            Severed[n] = (ushort)(Severed[n] & ~(1 << e));
                        }
                    }
                }
            }
        }
    }
}
