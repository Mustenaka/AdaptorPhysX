using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>A stitch (<see cref="SoftBodySimulation.AddStitch"/>).</summary>
    internal struct StitchState
    {
        /// <summary>Material points at the two ends.</summary>
        public int PointA;
        public int PointB;
        /// <summary>Length of the stitch at the start and the end of the step.</summary>
        public float StartLength;
        public float Length;
        /// <summary>Length the stitch is drawn to, and the speed at which it gets there, m/s.</summary>
        public float Target;
        public float Speed;
        /// <summary>Length of the material between the two points without stress (<see cref="SoftBodySimulation.StitchNaturalLength"/>).</summary>
        public float NaturalLength;
        /// <summary>Compliance, m/N; 0 is rigid.</summary>
        public float Compliance;
        /// <summary>0 once removed.</summary>
        public int Active;
    }

    /// <summary>The two nodes of one grid point on the two sides of an incision (see <see cref="CutFacePairsJob"/>).</summary>
    internal struct CutFacePair
    {
        /// <summary>The node of the copy that has material at the grid point.</summary>
        public int Owner;
        /// <summary>The node there of the copy on the other side of the cut.</summary>
        public int Other;
        /// <summary>The copies of the owner and the other node in the cell: their rotations turn the normal into the world.</summary>
        public int Copy;
        public int OtherCopy;
        /// <summary>Rest normal of the cut, from the owner's side to the other side.</summary>
        public float3 Normal;
    }

    /// <summary>
    /// Simplified sutures without a thread (TODO SUT-5): a stitch is a distance constraint between two material points (SUT-1)
    /// on the two sides of an incision, solved with the dynamic attachments (<see cref="AttachmentJob"/>). Like a thread it
    /// only pulls. Its length starts at the distance of the points and is drawn at a given speed to a target, by default the
    /// length the material between the points has without stress: where the tissue is pre-strained, the distance of the
    /// points before the cut leaves the incision open, as the tissue between a point and the wall contracts. A shorter
    /// target presses the walls into each other. The walls of an incision within one piece are kept from passing through
    /// each other by the cut-face contact (<see cref="CutFaceContactJob"/>), part of the self-collision.
    /// </summary>
    public sealed partial class SoftBodySimulation
    {
        private NativeList<StitchState> _stitches;
        /// <summary>Per stitch: the impulse of the thread in the last step, N s (the sum of -lambda / h).</summary>
        private NativeArray<float> _stitchImpulses;
        private bool _stitchesActive;
        private NativeList<CutFacePair> _cutFacePairs;

        /// <summary>
        /// Adds a stitch between material points <paramref name="pointA"/> and <paramref name="pointB"/>; returns its handle.
        /// Its length starts at the points' distance and is drawn by <paramref name="speed"/> m/s to the length of the
        /// material between them without stress (<see cref="StitchNaturalLength"/>; <see cref="SetStitchTarget"/> changes
        /// it). <paramref name="compliance"/> is in m/N (0 rigid).
        /// </summary>
        public int AddStitch(int pointA, int pointB, float speed, float compliance = 0.0f)
        {
            CompleteStep();
            if (!IsMaterialPoint(pointA) || !IsMaterialPoint(pointB) || pointA == pointB)
            {
                throw new System.ArgumentException("A stitch needs two material points.");
            }
            float length = math.distance(MaterialPointPosition(pointA), MaterialPointPosition(pointB));
            float natural = NaturalLength(MaterialPointRestPosition(pointA), MaterialPointRestPosition(pointB));
            _stitches.Add(new StitchState
            {
                PointA = pointA,
                PointB = pointB,
                StartLength = length,
                Length = length,
                Target = natural,
                Speed = math.max(speed, 0.0f),
                NaturalLength = natural,
                Compliance = math.max(compliance, 0.0f),
                Active = 1,
            });
            return _stitches.Length - 1;
        }

        /// <summary>Changes the length a stitch is drawn to, and optionally the speed, m/s.</summary>
        public void SetStitchTarget(int stitch, float length, float speed = -1.0f)
        {
            CompleteStep();
            StitchState state = _stitches[stitch];
            state.Target = math.max(length, 0.0f);
            state.Speed = speed >= 0.0f ? speed : state.Speed;
            _stitches[stitch] = state;
        }

        /// <summary>Cuts a stitch; its handle is not used again.</summary>
        public void RemoveStitch(int stitch)
        {
            CompleteStep();
            if (!IsStitch(stitch))
            {
                return;
            }
            StitchState state = _stitches[stitch];
            state.Active = 0;
            _stitches[stitch] = state;
        }

        public bool IsStitch(int stitch) => stitch >= 0 && stitch < _stitches.Length && _stitches[stitch].Active != 0;

        /// <summary>
        /// Length of the material between a stitch's points without stress, its default target: the straight segment between
        /// them at rest, through the rest shapes of the cells (pre-strain, skin layer, plastic flow) when the stitch was added.
        /// Without pre-strain, the distance of the points before the cut.
        /// </summary>
        public float StitchNaturalLength(int stitch)
        {
            CompleteStep();
            return _stitches[stitch].NaturalLength;
        }

        /// <summary>Length of a stitch at the end of the last step; the points are at most this far apart.</summary>
        public float StitchLength(int stitch)
        {
            CompleteStep();
            return _stitches[stitch].Length;
        }

        /// <summary>Force of a stitch's thread in the last step, averaged over the step, N; 0 when it was slack.</summary>
        public float StitchTension(int stitch)
        {
            CompleteStep();
            return IsStitch(stitch) && _stitchImpulses.IsCreated && stitch < _stitchImpulses.Length && _impulseDt > 0.0f
                ? _stitchImpulses[stitch] / _impulseDt
                : 0.0f;
        }

        /// <summary>Whether the length of a stitch still changes towards its target.</summary>
        public bool StitchesDrawing
        {
            get
            {
                CompleteStep();
                for (int i = 0; i < _stitches.Length; i++)
                {
                    if (_stitches[i].Active != 0 && _stitches[i].Length != _stitches[i].Target)
                    {
                        return true;
                    }
                }
                return false;
            }
        }

        /// <summary>Node pairs of the cut-face contact in the current topology (<see cref="CutFacePairsJob"/>).</summary>
        public int CutFacePairCount => _cutFacePairs.Length;

        /// <summary>
        /// Length of the straight segment between two rest positions in the rest shapes of the cells it passes through
        /// (x to S x, see <see cref="UpdateCellRest"/>), in the world: sampled at 32 points, unscaled outside the cells.
        /// </summary>
        private float NaturalLength(float3 a, float3 b)
        {
            const int Samples = 32;
            var toWorld = new float3x3(RestToWorld);
            float3 step = (b - a) / Samples;
            float length = 0.0f;
            for (int i = 0; i < Samples; i++)
            {
                int3 p = (int3)math.floor((a + (i + 0.5f) * step - Grid.Origin) / CellSize);
                int cell = math.all(p >= 0 & p < Grid.Dims) ? GridToCell[p.x + Grid.Dims.x * (p.y + Grid.Dims.y * p.z)] : -1;
                // The cells keep the inverse of their rest shape.
                float3x3 shape = cell >= 0 ? math.inverse(_cellRest[cell]) : float3x3.identity;
                length += math.length(math.mul(toWorld, math.mul(shape, step)));
            }
            return length;
        }

        private void CreateSutures()
        {
            _stitches = new NativeList<StitchState>(0, Allocator.Persistent);
            _cutFacePairs = new NativeList<CutFacePair>(0, Allocator.Persistent);
        }

        private void DisposeSutures()
        {
            if (_stitches.IsCreated) _stitches.Dispose();
            if (_cutFacePairs.IsCreated) _cutFacePairs.Dispose();
            DisposeIfCreated(ref _stitchImpulses);
        }

        /// <summary>Before a step of <paramref name="dt"/>: the stitches are drawn towards their targets, the impulses start at zero.</summary>
        private void BeginStitches(float dt)
        {
            _stitchesActive = false;
            for (int i = 0; i < _stitches.Length; i++)
            {
                StitchState s = _stitches[i];
                if (s.Active == 0)
                {
                    continue;
                }
                s.StartLength = s.Length;
                float draw = s.Speed * dt;
                s.Length = s.Length > s.Target ? math.max(s.Target, s.Length - draw) : math.min(s.Target, s.Length + draw);
                _stitches[i] = s;
                _stitchesActive |= IsMaterialPoint(s.PointA) && IsMaterialPoint(s.PointB);
            }
            if (!_stitchImpulses.IsCreated || _stitchImpulses.Length != _stitches.Length)
            {
                DisposeIfCreated(ref _stitchImpulses);
                _stitchImpulses = new NativeArray<float>(_stitches.Length, Allocator.Persistent);
            }
            else
            {
                for (int i = 0; i < _stitchImpulses.Length; i++)
                {
                    _stitchImpulses[i] = 0.0f;
                }
            }
        }

        /// <summary>
        /// Node pairs of the cut-face contact, for the current topology and pieces. Edges a stroke still in progress severed
        /// are left out: the blade lies between their walls, and pushing them apart changes what it cuts next.
        /// <see cref="EndStroke"/> adds them.
        /// </summary>
        private void RebuildCutFacePairs()
        {
            new CutFacePairsJob
            {
                CellCopyRange = CellCopyRange,
                Copies = Copies.AsArray(),
                CopyNodes = CopyNodes.AsArray(),
                Severed = Severed,
                StrokeStartSevered = StrokeStartSevered,
                InStroke = _stroke != 0 && !_strokeEnded,
                EdgeNormal = EdgeNormal,
                EdgeCorners = _tables.EdgeCorners,
                PieceId = PieceId,
                Pairs = _cutFacePairs,
            }.Run();
        }

        /// <summary>Schedules the cut-face contact of a substep after <paramref name="handle"/>.</summary>
        private JobHandle ScheduleCutFaceContacts(JobHandle handle)
        {
            if (!_step.Contacts || _cutFacePairs.Length == 0)
            {
                return handle;
            }
            return new CutFaceContactJob
            {
                Positions = Positions,
                Weights = _weights,
                Pairs = _cutFacePairs.AsArray(),
                TetRotation = _tetRotation,
                MaxDistance = 2.0f * CellSize,
            }.Schedule(handle);
        }
    }

    /// <summary>
    /// The pairs of nodes of the cut-face contact: wherever a severed edge of a cell separates two of its copies of the same
    /// piece (an incision that does not go through), the two nodes at each end of the edge, the owner's (the copy with
    /// material there) and the other copy's, which lies behind its cut face. At rest they coincide; when the other side's
    /// cut face moves into the owner's side, its node there moves past the owner's node along the cut normal. Pairs that
    /// several edges give are listed once, with the sum of the edges' normals. Separated pieces have the piece contact
    /// (DESIGN 3.1) instead. With <see cref="InStroke"/>, edges severed since the stroke began are left out.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CutFacePairsJob : IJob
    {
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<CellCopy> Copies;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<ushort> Severed;
        [ReadOnly] public NativeArray<ushort> StrokeStartSevered;
        /// <summary>A stroke is in progress (<see cref="SoftBodySimulation.EndStroke"/>).</summary>
        public bool InStroke;
        [ReadOnly] public NativeArray<float3> EdgeNormal;
        [ReadOnly] public NativeArray<int> EdgeCorners;
        [ReadOnly] public NativeArray<int> PieceId;
        public NativeList<CutFacePair> Pairs;

        public void Execute()
        {
            Pairs.Clear();
            var index = new NativeParallelHashMap<long, int>(256, Allocator.Temp);
            for (int c = 0; c < CellCopyRange.Length; c++)
            {
                int2 range = CellCopyRange[c];
                int severed = InStroke ? Severed[c] & StrokeStartSevered[c] : Severed[c];
                if (range.y < 2 || severed == 0)
                {
                    continue;
                }
                for (int e = 0; e < Lattice.EdgeCount; e++)
                {
                    if ((severed & (1 << e)) == 0)
                    {
                        continue;
                    }
                    int a = EdgeCorners[2 * e], b = EdgeCorners[2 * e + 1];
                    int qa = CopyWith(range, a), qb = CopyWith(range, b);
                    if (qa < 0 || qb < 0 || qa == qb)
                    {
                        continue;
                    }
                    int na = CopyNodes[qa * Lattice.CornerCount + a], nb = CopyNodes[qb * Lattice.CornerCount + b];
                    if (PieceId[na] != PieceId[nb])
                    {
                        continue;
                    }
                    // The edge runs from corner a along its axis.
                    float3 n = EdgeNormal[c * Lattice.EdgeCount + e];
                    n = n[e / 4] < 0.0f ? -n : n;
                    Add(ref index, na, CopyNodes[qb * Lattice.CornerCount + a], qa, qb, n);
                    Add(ref index, nb, CopyNodes[qa * Lattice.CornerCount + b], qb, qa, -n);
                }
            }
            for (int i = 0; i < Pairs.Length; i++)
            {
                CutFacePair pair = Pairs[i];
                pair.Normal = math.normalizesafe(pair.Normal);
                Pairs[i] = pair;
            }
            index.Dispose();
        }

        private int CopyWith(int2 range, int corner)
        {
            for (int q = range.x; q < range.x + range.y; q++)
            {
                if (((Copies[q].Mask >> corner) & 1) != 0)
                {
                    return q;
                }
            }
            return -1;
        }

        private void Add(ref NativeParallelHashMap<long, int> index, int owner, int other, int copy, int otherCopy, float3 normal)
        {
            // Near the end of an incision the copies share their nodes again.
            if (owner == other)
            {
                return;
            }
            long key = ((long)owner << 32) | (uint)other;
            if (index.TryGetValue(key, out int i))
            {
                CutFacePair pair = Pairs[i];
                pair.Normal += normal;
                Pairs[i] = pair;
                return;
            }
            index.Add(key, Pairs.Length);
            Pairs.Add(new CutFacePair { Owner = owner, Other = other, Copy = copy, OtherCopy = otherCopy, Normal = normal });
        }
    }

    /// <summary>
    /// The cut-face contact of a substep, after the constraint solve and the attachments, in order: the other node of a
    /// pair (<see cref="CutFacePairsJob"/>) must not be behind the owner along the cut normal n, n . (x_other - x_owner) >= 0;
    /// where it is, both move apart along n by their inverse masses. n is the rest normal turned by the rotations of the
    /// two copies (their first tetrahedra), averaged. Pairs whose copies turn it by more than 120 degrees apart (a cell
    /// squeezed inside out, whose rotation has flipped) and pairs further apart than <see cref="MaxDistance"/> (a flap
    /// folded away) are skipped.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CutFaceContactJob : IJob
    {
        public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float> Weights;
        [ReadOnly] public NativeArray<CutFacePair> Pairs;
        [ReadOnly] public NativeArray<quaternion> TetRotation;
        public float MaxDistance;

        public void Execute()
        {
            for (int i = 0; i < Pairs.Length; i++)
            {
                CutFacePair p = Pairs[i];
                float wOwner = Weights[p.Owner], wOther = Weights[p.Other];
                float3 d = Positions[p.Other] - Positions[p.Owner];
                if (wOwner + wOther <= 0.0f || math.lengthsq(d) > MaxDistance * MaxDistance)
                {
                    continue;
                }
                float3 n = math.mul(TetRotation[p.Copy * Lattice.TetsPerCell], p.Normal) +
                           math.mul(TetRotation[p.OtherCopy * Lattice.TetsPerCell], p.Normal);
                float length = math.length(n);
                if (length < 1.0f)
                {
                    continue;
                }
                n /= length;
                float c = math.dot(d, n);
                if (c >= 0.0f)
                {
                    continue;
                }
                float push = -c / (wOwner + wOther);
                Positions[p.Owner] -= wOwner * push * n;
                Positions[p.Other] += wOther * push * n;
            }
        }
    }
}
