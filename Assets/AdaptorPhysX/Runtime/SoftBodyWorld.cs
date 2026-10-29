using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Pairs nearby soft bodies for contact (TODO ECO-1). Bodies step in parallel jobs, so each
    /// one collides with copies of its partners' cells taken when a round of steps begins: the
    /// first <see cref="ApxSoftBody.Simulate"/> call, and every call by a body that already
    /// stepped in the current round. All bodies' jobs are complete at that point.
    /// </summary>
    internal static class SoftBodyWorld
    {
        private static readonly HashSet<ApxSoftBody> Stepped = new HashSet<ApxSoftBody>();

        /// <summary>Static state survives entering Play mode when domain reload is disabled.</summary>
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetStatics()
        {
            Stepped.Clear();
            DistanceFieldCache.Clear();
        }

        public static void BeginStep(ApxSoftBody body)
        {
            if (Stepped.Count == 0 || Stepped.Contains(body))
            {
                Stepped.Clear();
                PairBodies();
            }
            Stepped.Add(body);
        }

        public static void Remove(ApxSoftBody body) => Stepped.Remove(body);

        private static void PairBodies()
        {
            List<ApxSoftBody> bodies = ApxSoftBody.Active;
            foreach (ApxSoftBody body in bodies)
            {
                body.Simulation.ClearPartners();
            }
            float dt = Time.fixedDeltaTime;
            // Bodies in contact step together (SubstepGroup): connected components of the pairs.
            var root = new int[bodies.Count];
            for (int i = 0; i < root.Length; i++)
            {
                root[i] = i;
            }
            int Find(int i)
            {
                while (root[i] != i)
                {
                    i = root[i] = root[root[i]];
                }
                return i;
            }
            for (int a = 0; a < bodies.Count; a++)
            {
                ApxSoftBody first = bodies[a];
                if (!first.collideWithBodies)
                {
                    continue;
                }
                for (int b = a + 1; b < bodies.Count; b++)
                {
                    ApxSoftBody second = bodies[b];
                    if (!second.collideWithBodies || (first.collisionLayers & (1 << second.gameObject.layer)) == 0 ||
                        (second.collisionLayers & (1 << first.gameObject.layer)) == 0 ||
                        first.gameObject.scene.GetPhysicsScene() != second.gameObject.scene.GetPhysicsScene())
                    {
                        continue;
                    }
                    SoftBodySimulation x = first.Simulation, y = second.Simulation;
                    // Region each body takes the other's cells from: its own nodes plus the cell sizes and
                    // the distance both can travel during the step.
                    float margin = math.max(x.CellSize, y.CellSize) + (x.MaxSpeed + y.MaxSpeed) * dt;
                    float3 minX = x.BoundsMin - margin, maxX = x.BoundsMax + margin;
                    float3 minY = y.BoundsMin - margin, maxY = y.BoundsMax + margin;
                    if (math.any(maxX < y.BoundsMin) || math.any(minX > y.BoundsMax))
                    {
                        continue;
                    }
                    float friction = 0.5f * (first.friction + second.friction);
                    x.AddPartner(y, minX, maxX, friction);
                    y.AddPartner(x, minY, maxY, friction);
                    root[Find(a)] = Find(b);
                    // A moving body wakes a sleeping one it may touch; two resting bodies both sleep.
                    if (!first.IsSleeping && x.MaxSpeed > ApxSoftBody.SleepSpeed)
                    {
                        second.Wake();
                    }
                    if (!second.IsSleeping && y.MaxSpeed > ApxSoftBody.SleepSpeed)
                    {
                        first.Wake();
                    }
                }
            }
            var groups = new Dictionary<int, SubstepGroup>();
            for (int i = 0; i < bodies.Count; i++)
            {
                if (bodies[i].Simulation.PartnerCellCount == 0)
                {
                    continue;
                }
                int r = Find(i);
                if (!groups.TryGetValue(r, out SubstepGroup group))
                {
                    groups[r] = group = new SubstepGroup();
                }
                group.Add(bodies[i].Simulation);
            }
        }
    }
}
