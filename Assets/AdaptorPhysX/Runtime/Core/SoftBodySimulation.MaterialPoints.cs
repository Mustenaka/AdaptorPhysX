using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// A point of the material embedded in a cell copy (TODO SUT-1): it moves with the eight nodes of the copy by the
    /// trilinear interpolation the rendered skin uses, and keeps to its side of a cut through its cell. Anchors of
    /// sutures, attachments and contacts that must stay on one side of an incision are material points.
    /// </summary>
    public struct MaterialPoint
    {
        /// <summary>Cell copy the point is embedded in; -1 for a removed point.</summary>
        public int Copy;
        /// <summary>Position in the copy's cell, 0 to 1 along each axis of the rest lattice.</summary>
        public float3 U;
    }

    public sealed partial class SoftBodySimulation
    {
        /// <summary>Distance outside its cell, in cells, at which a position still counts as inside (<see cref="AddMaterialPoint(float3)"/>).</summary>
        public const float MaterialPointTolerance = 1e-3f;

        /// <summary>Material points by handle; removed points keep their handle with copy -1.</summary>
        private NativeList<MaterialPoint> _materialPoints;

        /// <summary>Number of material point handles, removed points included.</summary>
        public int MaterialPointCount => _materialPoints.Length;

        internal NativeArray<MaterialPoint> MaterialPoints => _materialPoints.AsArray();

        /// <summary>Adds a material point in copy <paramref name="copy"/> at <paramref name="u"/> (clamped to the cell); returns its handle.</summary>
        public int AddMaterialPoint(int copy, float3 u)
        {
            CompleteStep();
            if (copy < 0 || copy >= CopyCount)
            {
                throw new System.ArgumentOutOfRangeException(nameof(copy));
            }
            _materialPoints.Add(new MaterialPoint { Copy = copy, U = math.saturate(u) });
            return _materialPoints.Length - 1;
        }

        /// <summary>
        /// Adds a material point at world position <paramref name="position"/> in the current shape; returns its handle,
        /// or -1 when no cell copy holds the position. Of the copies whose deformed cell holds it, the point goes to the one
        /// with the most material there (the largest sum of the trilinear weights of its real corners): next to an
        /// incision, the copy on the side the position is on.
        /// </summary>
        public int AddMaterialPoint(float3 position)
        {
            CompleteStep();
            using var result = new NativeArray<MaterialPoint>(1, Allocator.TempJob);
            new FindMaterialPointJob
            {
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                Positions = Positions,
                Target = position,
                Tolerance = MaterialPointTolerance,
                Result = result,
            }.Run();
            if (result[0].Copy < 0)
            {
                return -1;
            }
            _materialPoints.Add(result[0]);
            return _materialPoints.Length - 1;
        }

        /// <summary>Removes a material point; its handle is not reused. False when there was none.</summary>
        public bool RemoveMaterialPoint(int point)
        {
            CompleteStep();
            if (!IsMaterialPoint(point))
            {
                return false;
            }
            _materialPoints[point] = new MaterialPoint { Copy = -1 };
            return true;
        }

        public bool IsMaterialPoint(int point) => point >= 0 && point < _materialPoints.Length && _materialPoints[point].Copy >= 0;

        public MaterialPoint GetMaterialPoint(int point)
        {
            CompleteStep();
            return _materialPoints[point];
        }

        /// <summary>Current world position of a material point.</summary>
        public float3 MaterialPointPosition(int point) => InterpolateNodes(point, Positions);

        /// <summary>Current velocity of a material point.</summary>
        public float3 MaterialPointVelocity(int point) => InterpolateNodes(point, Velocities);

        /// <summary>Rest-space position of a material point.</summary>
        public float3 MaterialPointRestPosition(int point)
        {
            MaterialPoint p = GetMaterialPoint(point);
            return Grid.Origin + ((float3)CellCoord[Copies[p.Copy].Cell] + p.U) * CellSize;
        }

        /// <summary>Piece of a material point (<see cref="PieceId"/>): that of the real corners of its copy.</summary>
        public int MaterialPointPiece(int point)
        {
            MaterialPoint p = GetMaterialPoint(point);
            int mask = Copies[p.Copy].Mask;
            return PieceId[CopyNodes[p.Copy * Lattice.CornerCount + math.tzcnt(mask)]];
        }

        private float3 InterpolateNodes(int point, NativeArray<float3> values)
        {
            MaterialPoint p = GetMaterialPoint(point);
            float3 x = float3.zero;
            for (int k = 0; k < Lattice.CornerCount; k++)
            {
                x += Geometry.TrilinearWeight(k, p.U) * values[CopyNodes[p.Copy * Lattice.CornerCount + k]];
            }
            return x;
        }

        /// <summary>Moves the material points to the copies of the new topology; the old copies are still in place.</summary>
        private void RemapMaterialPoints(NativeArray<int2> newRange, NativeList<CellCopy> newCopies)
        {
            if (_materialPoints.Length == 0)
            {
                return;
            }
            new RemapMaterialPointsJob
            {
                OldCopies = Copies.AsArray(),
                NewRange = newRange,
                NewCopies = newCopies.AsArray(),
                Severed = Severed,
                EdgeT = EdgeT,
                EdgeNormal = EdgeNormal,
                EdgeCorners = _tables.EdgeCorners,
                Points = _materialPoints.AsArray(),
            }.Run();
        }
    }

    /// <summary>
    /// The copy whose deformed cell holds <see cref="Target"/> and has the most material there, with the target's
    /// position in the cell (inverse of the trilinear interpolation, by Newton's method).
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FindMaterialPointJob : IJob
    {
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<float3> Positions;
        public float3 Target;
        /// <summary>Distance outside the cell, in cells, still accepted.</summary>
        public float Tolerance;
        /// <summary>[0]: the point found, copy -1 when none.</summary>
        public NativeArray<MaterialPoint> Result;

        public void Execute()
        {
            var best = new MaterialPoint { Copy = -1 };
            float bestMaterial = -1.0f;
            for (int q = 0; q < Copies.Length; q++)
            {
                int b = q * Lattice.CornerCount;
                float3 lo = Positions[CopyNodes[b]], hi = lo;
                for (int k = 1; k < Lattice.CornerCount; k++)
                {
                    lo = math.min(lo, Positions[CopyNodes[b + k]]);
                    hi = math.max(hi, Positions[CopyNodes[b + k]]);
                }
                float3 margin = (hi - lo) * Tolerance;
                if (math.any(Target < lo - margin) || math.any(Target > hi + margin) || !Invert(b, math.cmax(hi - lo), out float3 u) ||
                    math.any(u < -Tolerance) || math.any(u > 1.0f + Tolerance))
                {
                    continue;
                }
                u = math.saturate(u);
                int mask = Copies[q].Mask;
                float material = 0.0f;
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    material += ((mask >> k) & 1) != 0 ? Geometry.TrilinearWeight(k, u) : 0.0f;
                }
                if (material > bestMaterial)
                {
                    bestMaterial = material;
                    best = new MaterialPoint { Copy = q, U = u };
                }
            }
            Result[0] = best;
        }

        private bool Invert(int b, float size, out float3 u)
        {
            u = new float3(0.5f);
            float3 r = float3.zero;
            for (int iteration = 0; iteration < 20; iteration++)
            {
                float3 x = float3.zero;
                float3x3 j = float3x3.zero;
                for (int k = 0; k < Lattice.CornerCount; k++)
                {
                    float3 p = Positions[CopyNodes[b + k]];
                    float3 g = Geometry.TrilinearGradient(k, u);
                    x += Geometry.TrilinearWeight(k, u) * p;
                    j.c0 += p * g.x;
                    j.c1 += p * g.y;
                    j.c2 += p * g.z;
                }
                r = Target - x;
                if (math.length(r) <= 1e-5f * size)
                {
                    return true;
                }
                if (math.abs(math.determinant(j)) <= 1e-12f * size * size * size)
                {
                    return false;
                }
                u = math.clamp(u + math.mul(math.inverse(j), r), -1.0f, 2.0f);
            }
            return false;
        }
    }

    /// <summary>
    /// Moves every material point to a copy of the new topology in its cell: the copy that took over the old copy's
    /// corners, or, where a cut split the old copy, the part on the point's side of the cut. The side is that of the
    /// nearest severed edge between the parts, from the edge's cut point and rest normal (<see cref="SoftBodySimulation.EdgeT"/>,
    /// <see cref="SoftBodySimulation.EdgeNormal"/>); where that does not tell, the part with the most material at the
    /// point. The material alone puts a point near the cut into the larger part, e.g. a point cut off with one corner.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct RemapMaterialPointsJob : IJob
    {
        [ReadOnly] public NativeArray<CellCopy> OldCopies;
        [ReadOnly] public NativeArray<int2> NewRange;
        [ReadOnly] public NativeArray<CellCopy> NewCopies;
        [ReadOnly] public NativeArray<ushort> Severed;
        [ReadOnly] public NativeArray<float> EdgeT;
        [ReadOnly] public NativeArray<float3> EdgeNormal;
        [ReadOnly] public NativeArray<int> EdgeCorners;
        public NativeArray<MaterialPoint> Points;

        public void Execute()
        {
            for (int i = 0; i < Points.Length; i++)
            {
                MaterialPoint point = Points[i];
                if (point.Copy < 0)
                {
                    continue;
                }
                CellCopy old = OldCopies[point.Copy];
                int2 range = NewRange[old.Cell];
                // Every corner of the old copy is in exactly one new copy, so there is at least one.
                int best = -1;
                bool bestInside = false;
                float bestMaterial = -1.0f;
                for (int q = range.x; q < range.x + range.y; q++)
                {
                    int mask = NewCopies[q].Mask;
                    if ((mask & old.Mask) == 0)
                    {
                        continue;
                    }
                    bool inside = Inside(old.Cell, mask, old.Mask, point.U);
                    float material = 0.0f;
                    for (int k = 0; k < Lattice.CornerCount; k++)
                    {
                        material += ((mask >> k) & 1) != 0 ? Geometry.TrilinearWeight(k, point.U) : 0.0f;
                    }
                    if (best < 0 || (inside && !bestInside) || (inside == bestInside && material > bestMaterial))
                    {
                        best = q;
                        bestInside = inside;
                        bestMaterial = material;
                    }
                }
                point.Copy = best;
                Points[i] = point;
            }
        }

        /// <summary>
        /// Whether <paramref name="u"/> is on the side of part <paramref name="mask"/> at the nearest severed edge between
        /// it and another part of <paramref name="oldMask"/>; false when no edge lies between them.
        /// </summary>
        private bool Inside(int cell, int mask, int oldMask, float3 u)
        {
            int severed = Severed[cell];
            float nearest = float.MaxValue;
            bool inside = false;
            for (int e = 0; e < Lattice.EdgeCount; e++)
            {
                int a = EdgeCorners[2 * e], b = EdgeCorners[2 * e + 1];
                bool inA = ((mask >> a) & 1) != 0, inB = ((mask >> b) & 1) != 0;
                int outer = inA ? b : a;
                if ((severed & (1 << e)) == 0 || inA == inB || ((oldMask >> outer) & 1) == 0)
                {
                    continue;
                }
                float3 pa = Lattice.CornerOffset(a), pb = Lattice.CornerOffset(b);
                float3 h = math.lerp(pa, pb, EdgeT[cell * Lattice.EdgeCount + e]);
                float distance = math.distancesq(u, h);
                if (distance < nearest)
                {
                    nearest = distance;
                    float3 n = EdgeNormal[cell * Lattice.EdgeCount + e];
                    inside = (math.dot(n, u - h) >= 0.0f) == (math.dot(n, (inA ? pa : pb) - h) >= 0.0f);
                }
            }
            return inside;
        }
    }
}
