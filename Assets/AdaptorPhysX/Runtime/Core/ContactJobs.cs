using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace AdaptorPhysX
{
    /// <summary>
    /// Node-level contact between separated pieces (and self-contact of folded tissue).
    /// Contacts are gathered once per frame from a spatial hash and resolved every substep.
    /// A pair may not come closer than min(rest distance, contact distance), so the two
    /// sides of a fresh cut start in contact instead of being pushed apart.
    /// </summary>
    internal static class Contacts
    {
        public const int MaxPerNode = 16;
        public const int BucketCount = 1 << 14;

        public static int Bucket(int3 c) =>
            (int)((uint)(c.x * 73856093 ^ c.y * 19349663 ^ c.z * 83492791) & (BucketCount - 1));
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct BuildContactHashJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<byte> NodeReal;
        public float InverseCellSize;
        /// <summary>BucketCount + 1 prefix offsets.</summary>
        public NativeArray<int> BucketStart;
        public NativeArray<int> Entries;

        public void Execute()
        {
            for (int b = 0; b < BucketStart.Length; b++)
            {
                BucketStart[b] = 0;
            }
            for (int i = 0; i < Positions.Length; i++)
            {
                if (NodeReal[i] != 0)
                {
                    BucketStart[Contacts.Bucket((int3)math.floor(Positions[i] * InverseCellSize)) + 1]++;
                }
            }
            for (int b = 1; b < BucketStart.Length; b++)
            {
                BucketStart[b] += BucketStart[b - 1];
            }
            var fill = new NativeArray<int>(Contacts.BucketCount, Allocator.Temp);
            for (int i = 0; i < Positions.Length; i++)
            {
                if (NodeReal[i] == 0)
                {
                    continue;
                }
                int bucket = Contacts.Bucket((int3)math.floor(Positions[i] * InverseCellSize));
                Entries[BucketStart[bucket] + fill[bucket]++] = i;
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct FindContactsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> RestPositions;
        [ReadOnly] public NativeArray<byte> NodeReal;
        [ReadOnly] public NativeArray<int> PieceId;
        [ReadOnly] public NativeArray<int> BucketStart;
        [ReadOnly] public NativeArray<int> Entries;
        public float InverseCellSize;
        public float SearchRadius;
        public float ContactDistance;
        /// <summary>Same-piece pairs closer than this at rest are lattice neighbours and skipped.</summary>
        public float NeighbourRestDistance;

        [NativeDisableParallelForRestriction] public NativeArray<int> Partner;
        [NativeDisableParallelForRestriction] public NativeArray<float> Distance;
        public NativeArray<int> Count;

        public void Execute(int i)
        {
            int count = 0;
            if (NodeReal[i] != 0)
            {
                float3 x = Positions[i];
                int3 c = (int3)math.floor(x * InverseCellSize);
                float searchSq = SearchRadius * SearchRadius;
                for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    int bucket = Contacts.Bucket(c + new int3(dx, dy, dz));
                    for (int e = BucketStart[bucket]; e < BucketStart[bucket + 1] && count < Contacts.MaxPerNode; e++)
                    {
                        int j = Entries[e];
                        if (j == i || math.distancesq(Positions[j], x) > searchSq)
                        {
                            continue;
                        }
                        float rest = math.distance(RestPositions[i], RestPositions[j]);
                        float d0;
                        if (PieceId[i] != PieceId[j])
                        {
                            d0 = math.min(rest, ContactDistance);
                        }
                        else if (rest > NeighbourRestDistance)
                        {
                            d0 = ContactDistance;
                        }
                        else
                        {
                            continue;
                        }
                        if (d0 <= 0.0f)
                        {
                            continue;
                        }
                        Partner[i * Contacts.MaxPerNode + count] = j;
                        Distance[i * Contacts.MaxPerNode + count] = d0;
                        count++;
                    }
                }
            }
            Count[i] = count;
        }
    }

    [BurstCompile(CompileSynchronously = true, DisableSafetyChecks = true)]
    internal struct CopyPositionsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Source;
        public NativeArray<float3> Destination;

        public void Execute(int i) => Destination[i] = Source[i];
    }
}
