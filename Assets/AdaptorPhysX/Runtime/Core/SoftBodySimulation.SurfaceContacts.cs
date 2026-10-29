using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Contact of the surface with the colliders (TODO MED-17). The colliders push the lattice nodes, each node allowed into
    /// a collider by as far as it lies outside its own surface (<see cref="NodeStepJob"/>), and the cells at the surface:
    /// in each, the deepest point of the collider is held outside the surface the rest SDF describes there, interpolated
    /// trilinearly as the rendered skin is. The point is found again in every substep, so an instrument thinner than a
    /// cell, which passes between the nodes, presses the surface in instead of entering it. The cells are pushed out after
    /// the cells of each slice (<see cref="SolveCellsJob"/>), and the momentum they gain adds to the forces on the
    /// colliders (HAP-1).
    /// </summary>
    public sealed partial class SoftBodySimulation
    {
        /// <summary>Cells at the surface, slice by slice in the solver's order: _surfaceCells[_sliceSurfaceStart[s] ..).</summary>
        private NativeArray<int> _surfaceCells;
        private NativeArray<int> _sliceSurfaceStart;

        /// <summary>
        /// Cells that take part in the surface contact: those with a corner outside the surface or less than a cell inside
        /// it, where the rest SDF gradient at the centre is at least <see cref="MinSurfaceGradient"/> long.
        /// </summary>
        public int SurfaceCellCount => _surfaceCells.IsCreated ? _surfaceCells.Length : 0;

        /// <summary>Shortest rest SDF gradient at the centre of a surface cell (1 for a resolved distance field).</summary>
        public const float MinSurfaceGradient = 0.8f;

        /// <summary>Lists the surface cells slice by slice.</summary>
        private void CreateSurfaceCells(VoxelGrid grid)
        {
            var cells = new List<int>();
            var sliceStart = new int[_slices.Length + 1];
            var sliceCells = new List<int>();
            for (int s = 0; s < _slices.Length; s++)
            {
                sliceStart[s] = cells.Count;
                int4 slice = _slices[s];
                sliceCells.Clear();
                for (int g = slice.x; g < slice.x + slice.y; g++)
                {
                    int4 group = _sliceGroups[g];
                    sliceCells.Add(group.x);
                    sliceCells.Add(group.y);
                    sliceCells.Add(group.z);
                    sliceCells.Add(group.w);
                }
                for (int c = slice.z; c < slice.z + slice.w; c++)
                {
                    sliceCells.Add(_sliceCells[c]);
                }
                sliceCells.Sort();
                foreach (int c in sliceCells)
                {
                    float highest = float.MinValue;
                    float3 gradient = float3.zero;
                    for (int k = 0; k < Lattice.CornerCount; k++)
                    {
                        float sdf = grid.Sdf[grid.PointIndex(grid.Cells[c] + Lattice.CornerOffset(k))];
                        highest = math.max(highest, sdf);
                        gradient += Geometry.TrilinearGradient(k, new float3(0.5f)) * sdf;
                    }
                    // Where the SDF of two faces meets in a cell (tissue thinner than about two cells, the medial plane of
                    // an edge), its gradient is short and the surface it describes is not the visible one.
                    if (highest >= -grid.CellSize && math.length(gradient) >= MinSurfaceGradient * grid.CellSize)
                    {
                        cells.Add(c);
                    }
                }
            }
            sliceStart[_slices.Length] = cells.Count;
            _surfaceCells = new NativeArray<int>(cells.ToArray(), Allocator.Persistent);
            _sliceSurfaceStart = new NativeArray<int>(sliceStart, Allocator.Persistent);
        }

        private void DisposeSurfaceCells()
        {
            DisposeIfCreated(ref _surfaceCells);
            DisposeIfCreated(ref _sliceSurfaceStart);
            DisposeIfCreated(ref _noSurfaceImpulses);
        }

        /// <summary>Placeholder for the solve's impulse sums in a step without surface contacts (the frames may use the other one).</summary>
        private NativeArray<ColliderImpulse> _noSurfaceImpulses;

        /// <summary>
        /// Sets up the surface contacts of the constraint solve for a step: the momentum of slice s goes to block
        /// <paramref name="nodeBlocks"/> + s of the collider impulse sums.
        /// </summary>
        private void ConfigureSurfaceContacts(ref SolveCellsJob solve, NativeArray<float> fields, int nodeBlocks)
        {
            if (!_noSurfaceImpulses.IsCreated)
            {
                _noSurfaceImpulses = new NativeArray<ColliderImpulse>(0, Allocator.Persistent);
            }
            solve.SurfaceContacts = SurfaceContactsActive;
            solve.Previous = Previous;
            solve.NodeSdf = _nodeSdf;
            solve.Shapes = _shapes;
            solve.Fields = fields;
            solve.SurfaceCells = _surfaceCells;
            solve.SliceSurfaceStart = _sliceSurfaceStart;
            solve.Impulses = solve.SurfaceContacts ? _impulsePartial : _noSurfaceImpulses;
            solve.ImpulseBlockOffset = nodeBlocks;
            solve.Margin = Parameters.CollisionMargin;
            solve.ThinCollider = ThinColliderCells * CellSize;
        }

        /// <summary>
        /// Colliders thinner than this many cells (<see cref="ColliderShape.Thickness"/>) take part in the surface contact.
        /// The nodes alone let a sphere of radius r between four nodes of a face into the surface by r - sqrt(r^2 - h^2 / 2),
        /// more than a tenth of a cell below 2.5 cells; for larger colliders the nodes hold the surface, and the surface cells,
        /// which hold it at one point per cell, make the contact of a large sphere stiffer than Hertz's.
        /// </summary>
        public const float ThinColliderCells = 2.5f;

        /// <summary>Whether the step solves surface contacts: they are enabled, and a collider is thin.</summary>
        private bool SurfaceContactsActive
        {
            get
            {
                if (!Parameters.SurfaceContact || !_shapes.IsCreated || SurfaceCellCount == 0)
                {
                    return false;
                }
                for (int s = 0; s < _shapes.Length; s++)
                {
                    if (_shapes[s].Thickness() < ThinColliderCells * CellSize)
                    {
                        return true;
                    }
                }
                return false;
            }
        }
    }
}
