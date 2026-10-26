using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Momentum the colliders gave the nodes during a step, summed per block of nodes so that the
    /// node job needs no atomics; <see cref="ReduceImpulseJob"/> adds the blocks in a fixed order.
    /// </summary>
    public struct ColliderImpulse
    {
        /// <summary>Sum of m dx / h over nodes and substeps (N s), on the tissue.</summary>
        public float3 Linear;
        /// <summary>Sum of r x (m dx / h) about the shape's reference point (N m s).</summary>
        public float3 Angular;
        /// <summary>Contact points weighted by |dx|, and the sum of the weights.</summary>
        public float3 Centroid;
        public float Weight;
        /// <summary>Node contacts summed over the substeps.</summary>
        public float Contacts;

        public void Add(float3 x, float3 origin, float3 impulse, float size)
        {
            Linear += impulse;
            Angular += math.cross(x - origin, impulse);
            Centroid += x * size;
            Weight += size;
            Contacts += 1.0f;
        }

        public static ColliderImpulse operator +(ColliderImpulse a, ColliderImpulse b) => new ColliderImpulse
        {
            Linear = a.Linear + b.Linear,
            Angular = a.Angular + b.Angular,
            Centroid = a.Centroid + b.Centroid,
            Weight = a.Weight + b.Weight,
            Contacts = a.Contacts + b.Contacts,
        };
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct ReduceImpulseJob : IJob
    {
        /// <summary>Blocks * shapes partial sums.</summary>
        [ReadOnly] public NativeArray<ColliderImpulse> Partial;
        public NativeArray<ColliderImpulse> Total;

        public void Execute()
        {
            int shapes = Total.Length;
            for (int s = 0; s < shapes; s++)
            {
                var sum = new ColliderImpulse();
                for (int b = s; b < Partial.Length; b += shapes)
                {
                    sum += Partial[b];
                }
                Total[s] = sum;
            }
        }
    }

    /// <summary>
    /// Plastic flow per cell: the part of the elastic strain above the yield strain creeps into the
    /// rest shape, which may also recover towards the original shape. The rest shape of a cell is
    /// P S X (S the pre-strain and skin scaling, P the plastic stretch, kept volume-preserving
    /// and within the maximum deformation), so the solver's inverse rest scaling becomes S^-1 P^-1.
    /// </summary>
    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct PlasticityJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<int2> CellCopyRange;
        [ReadOnly] public NativeArray<int> CopyNodes;
        [ReadOnly] public NativeArray<quaternion> TetRotation;
        [ReadOnly] public NativeArray<float3x3> CellRestBase;
        public NativeArray<float3x3> CellPlastic;
        public NativeArray<float3x3> CellRest;
        public float InverseCellSize;
        /// <summary>Elastic strain (Frobenius norm of U - I) below which nothing flows.</summary>
        public float Yield;
        /// <summary>Fraction of the strain above yield absorbed in this step.</summary>
        public float Creep;
        /// <summary>Fraction of the plastic deformation recovered in this step.</summary>
        public float Recovery;
        /// <summary>Largest Frobenius norm of P - I.</summary>
        public float MaxDeformation;

        public void Execute(int c)
        {
            int copy = CellCopyRange[c].x;
            int b = copy * Lattice.CornerCount;
            // Deformation gradient at the cell centre with respect to the lattice rest coordinates.
            float3x3 f = float3x3.zero;
            for (int k = 0; k < 8; k++)
            {
                float3 x = Positions[CopyNodes[b + k]];
                float3 g = Geometry.TrilinearGradient(k, new float3(0.5f)) * InverseCellSize;
                f += new float3x3(x * g.x, x * g.y, x * g.z);
            }
            float3x3 p = CellPlastic[c];
            float3x3 elastic = math.mul(f, CellRest[c]);
            float3x3 r = new float3x3(TetRotation[copy * Lattice.TetsPerCell]);
            float3x3 u = math.mul(math.transpose(r), elastic);
            float3x3 strain = 0.5f * (u + math.transpose(u)) - float3x3.identity;
            float e = Norm(strain);
            if (e > Yield && Creep > 0.0f)
            {
                p = math.mul(float3x3.identity + strain * (Creep * (1.0f - Yield / e)), p);
            }
            if (Recovery > 0.0f)
            {
                p += (float3x3.identity - p) * Recovery;
            }
            float det = math.determinant(p);
            if (!(det > 1e-3f) || !math.all(math.isfinite(p.c0 + p.c1 + p.c2)))
            {
                p = float3x3.identity;
                det = 1.0f;
            }
            p *= 1.0f / math.pow(det, 1.0f / 3.0f);
            float3x3 d = p - float3x3.identity;
            float size = Norm(d);
            if (size > MaxDeformation)
            {
                p = float3x3.identity + d * (MaxDeformation / size);
            }
            CellPlastic[c] = p;
            CellRest[c] = math.mul(CellRestBase[c], math.inverse(p));
        }

        private static float Norm(float3x3 m) => math.sqrt(math.dot(m.c0, m.c0) + math.dot(m.c1, m.c1) + math.dot(m.c2, m.c2));
    }
}
