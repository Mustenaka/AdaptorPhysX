using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>The cut one stroke made in a body (<see cref="ApxScorer.IncisionMade"/>).</summary>
    [Serializable]
    public sealed class ApxIncision
    {
        public string Body;
        public int Stroke;
        /// <summary>Simulated time of the body when the stroke ended, s.</summary>
        public float Time;
        public int SeveredEdges;
        /// <summary>
        /// Extent of the cut at the surface along its direction, and its deepest point below the surface, m. Both span the
        /// severed lattice edges: the cut ends between the last edge it severed and the next, so they are short by less
        /// than a cell.
        /// </summary>
        public float Length;
        public float Depth;
        /// <summary>Area of the cut surface of one side, inside the tissue, m^2.</summary>
        public float Area;
        /// <summary>Ends and centre of the cut at the surface, on the tissue as placed at rest (world, m).</summary>
        public Vector3 Start;
        public Vector3 End;
        public Vector3 Center;
    }

    /// <summary>A time during which the force on an instrument exceeded the limit (<see cref="ApxScorer.ForceExceeded"/>).</summary>
    [Serializable]
    public sealed class ApxForceEvent
    {
        public string Body;
        /// <summary>The collider's name, or "grab N" for grab handle N.</summary>
        public string Source;
        /// <summary>Simulated time of the first step above the limit, s, and how long the force stayed above it.</summary>
        public float Start;
        public float Duration;
        /// <summary>Largest force, N.</summary>
        public float Peak;
    }

    /// <summary>Scores of an <see cref="ApxScorer"/> (<see cref="ApxScorer.Report"/>).</summary>
    [Serializable]
    public sealed class ApxScoreReport
    {
        public float ForceLimit;
        public List<ApxIncision> Incisions = new List<ApxIncision>();
        /// <summary>Finished times above the force limit, and those still going on.</summary>
        public List<ApxForceEvent> ForceEvents = new List<ApxForceEvent>();
        /// <summary>Area of all cut surfaces (one side) of the observed bodies, m^2: the tissue damaged by cutting.</summary>
        public float CutArea;
        /// <summary>Material pieces of the observed bodies.</summary>
        public int Pieces;
        /// <summary>Total time above the force limit, s, and the largest force on an instrument, N.</summary>
        public float OverForceTime;
        public float PeakForce;

        public string ToJson() => JsonUtility.ToJson(this, true);
    }

    /// <summary>
    /// Scoring of a procedure (TODO MED-13) from the events of the observed bodies (ECO-10): every stroke that cut a body
    /// gives an incision, measured on the tissue at rest when the stroke ends (length at the surface, depth, area of the cut
    /// surface, position); every time the force on an instrument (a grab, or a collider of <see cref="instruments"/>)
    /// exceeds <see cref="forceLimit"/> gives a force event; the report adds the cut area and the pieces of the bodies.
    /// </summary>
    public sealed class ApxScorer : MonoBehaviour
    {
        [Tooltip("Bodies observed while enabled; empty: every ApxSoftBody active when the scorer is enabled.")]
        public ApxSoftBody[] bodies = new ApxSoftBody[0];
        [Tooltip("Colliders whose force on the tissue is scored, besides the grabs (supports such as a table are left out).")]
        public Collider[] instruments = new Collider[0];
        [Tooltip("Force on an instrument (collider or grab), N, above which it counts as excessive.")]
        [Min(0.0f)] public float forceLimit = 2.0f;

        /// <summary>A stroke that cut an observed body ended (a cut of stroke 0 at once): its incision.</summary>
        public event Action<ApxIncision> IncisionMade;
        /// <summary>The force on an instrument rose above the limit; Duration is 0 and Peak the first force.</summary>
        public event Action<ApxForceEvent> ForceExceeded;

        private readonly List<Observed> _observed = new List<Observed>();
        private readonly List<ApxIncision> _incisions = new List<ApxIncision>();
        private readonly List<ApxForceEvent> _forceEvents = new List<ApxForceEvent>();

        public IReadOnlyList<ApxIncision> Incisions => _incisions;

        /// <summary>An observed body: its severed edges at the last cut, the edges of its open strokes and its force events.</summary>
        private sealed class Observed
        {
            public ApxSoftBody Body;
            public ushort[] Severed;
            public readonly SortedDictionary<int, List<int2>> Strokes = new SortedDictionary<int, List<int2>>();
            public readonly Dictionary<string, ApxForceEvent> Forces = new Dictionary<string, ApxForceEvent>();
            public readonly HashSet<string> Touched = new HashSet<string>();
            public readonly List<string> Released = new List<string>();
            public Action<ApxSoftBody, ApxCutEvent> OnSevered;
            public Action<ApxSoftBody, int> OnStrokeEnded;
            public Action<ApxSoftBody> OnStep;
        }

        private void OnEnable()
        {
            foreach (ApxSoftBody body in bodies.Length > 0 ? bodies : ApxSoftBody.Active.ToArray())
            {
                Observe(body);
            }
        }

        private void OnDisable()
        {
            while (_observed.Count > 0)
            {
                Forget(_observed[0].Body);
            }
        }

        /// <summary>Starts observing a body; its cuts so far are not scored.</summary>
        public void Observe(ApxSoftBody body)
        {
            if (body == null || _observed.Exists(o => o.Body == body))
            {
                return;
            }
            var observed = new Observed { Body = body };
            observed.OnSevered = (b, cut) => Severed(observed, cut);
            observed.OnStrokeEnded = (b, stroke) => Finish(observed, stroke);
            observed.OnStep = b => MeasureForces(observed);
            body.Severed += observed.OnSevered;
            body.StrokeEnded += observed.OnStrokeEnded;
            body.StepCompleted += observed.OnStep;
            observed.Severed = body.Simulation != null ? body.Simulation.Severed.ToArray() : null;
            _observed.Add(observed);
        }

        public void Forget(ApxSoftBody body)
        {
            int index = _observed.FindIndex(o => o.Body == body);
            if (index < 0)
            {
                return;
            }
            Observed observed = _observed[index];
            if (body != null)
            {
                body.Severed -= observed.OnSevered;
                body.StrokeEnded -= observed.OnStrokeEnded;
                body.StepCompleted -= observed.OnStep;
            }
            _observed.RemoveAt(index);
        }

        /// <summary>Clears the incisions and force events.</summary>
        public void ResetScores()
        {
            _incisions.Clear();
            _forceEvents.Clear();
            foreach (Observed observed in _observed)
            {
                observed.Strokes.Clear();
                observed.Forces.Clear();
            }
        }

        /// <summary>The scores so far: incisions, force events (also those still going on) and the bodies' cut area and pieces.</summary>
        public ApxScoreReport Report()
        {
            var report = new ApxScoreReport { ForceLimit = forceLimit };
            report.Incisions.AddRange(_incisions);
            report.ForceEvents.AddRange(_forceEvents);
            foreach (Observed observed in _observed)
            {
                report.ForceEvents.AddRange(observed.Forces.Values);
                SoftBodySimulation sim = observed.Body != null ? observed.Body.Simulation : null;
                if (sim == null)
                {
                    continue;
                }
                var all = new List<int2>();
                for (int c = 0; c < sim.CellCount; c++)
                {
                    for (int e = 0; e < Lattice.EdgeCount; e++)
                    {
                        if ((sim.Severed[c] & (1 << e)) != 0)
                        {
                            all.Add(new int2(c, e));
                        }
                    }
                }
                report.CutArea += Measure(sim, all).Area;
                report.Pieces += sim.CountPieces();
            }
            foreach (ApxForceEvent force in report.ForceEvents)
            {
                report.OverForceTime += force.Duration;
                report.PeakForce = Mathf.Max(report.PeakForce, force.Peak);
            }
            return report;
        }

        /// <summary>Adds the edges severed since the last cut of the body to the stroke's incision.</summary>
        private void Severed(Observed observed, ApxCutEvent cut)
        {
            SoftBodySimulation sim = observed.Body.Simulation;
            if (observed.Severed == null || observed.Severed.Length != sim.CellCount)
            {
                observed.Severed = new ushort[sim.CellCount];
            }
            if (!observed.Strokes.TryGetValue(cut.Stroke, out List<int2> edges))
            {
                observed.Strokes[cut.Stroke] = edges = new List<int2>();
            }
            // Strokes that cut the body between the same two steps are not told apart: the edges go to the first.
            for (int c = 0; c < sim.CellCount; c++)
            {
                int added = sim.Severed[c] & ~observed.Severed[c];
                for (int e = 0; added != 0 && e < Lattice.EdgeCount; e++)
                {
                    if ((added & (1 << e)) != 0)
                    {
                        edges.Add(new int2(c, e));
                    }
                }
                observed.Severed[c] = sim.Severed[c];
            }
            if (cut.Stroke == 0)
            {
                Finish(observed, 0);
            }
        }

        /// <summary>Measures the incision of an ended stroke.</summary>
        private void Finish(Observed observed, int stroke)
        {
            if (!observed.Strokes.TryGetValue(stroke, out List<int2> edges))
            {
                return;
            }
            observed.Strokes.Remove(stroke);
            SoftBodySimulation sim = observed.Body.Simulation;
            ApxIncision incision = Measure(sim, edges);
            if (incision.SeveredEdges == 0)
            {
                return;
            }
            incision.Body = observed.Body.name;
            incision.Stroke = stroke;
            incision.Time = (float)sim.SimulatedTime;
            _incisions.Add(incision);
            IncisionMade?.Invoke(incision);
        }

        /// <summary>
        /// Measures severed edges (cell, edge) on the tissue at rest, each lattice edge once and only while still severed.
        /// The cut point of an edge lies at its <see cref="SoftBodySimulation.EdgeT"/>, its depth is the rest SDF of the
        /// edge's nodes (<see cref="SoftBodySimulation.NodeSdf"/>) interpolated there. Area: a plane of unit normal n is
        /// crossed by |n_a| A / h^2 edges along axis a, so each edge along the largest component of its cut normal adds
        /// h^2 / |n_a|, times the part of its cell layer inside the tissue, saturate(0.5 - sdf / h). Length: the extent,
        /// along their principal direction, of the cut points within half a cell of the surface.
        /// </summary>
        private static ApxIncision Measure(SoftBodySimulation sim, List<int2> edges)
        {
            float h = sim.CellSize;
            float scale = math.length(sim.RestToWorld.c0.xyz);
            var seen = new HashSet<long>();
            var trace = new List<float3>();
            var incision = new ApxIncision();
            foreach (int2 edge in edges)
            {
                int c = edge.x, e = edge.y;
                if ((sim.Severed[c] & (1 << e)) == 0)
                {
                    continue;
                }
                int a = Lattice.EdgeCorners[2 * e], b = Lattice.EdgeCorners[2 * e + 1], axis = e / 4;
                int3 pa = sim.CellCoord[c] + Lattice.CornerOffset(a);
                if (!seen.Add((long)sim.Grid.PointIndex(pa) * 3 + axis))
                {
                    continue;
                }
                float t = sim.EdgeT[c * Lattice.EdgeCount + e];
                int first = sim.CellCopyRange[c].x * Lattice.CornerCount;
                float sdf = math.lerp(sim.NodeSdf[sim.CopyNodes[first + a]], sim.NodeSdf[sim.CopyNodes[first + b]], t);
                float3 rest = sim.Grid.Origin + ((float3)pa + t * (float3)(Lattice.CornerOffset(b) - Lattice.CornerOffset(a))) * h;
                float3 n = math.abs(math.normalizesafe(sim.EdgeNormal[c * Lattice.EdgeCount + e]));
                if (n[axis] > 0.0f && n[axis] >= math.cmax(n))
                {
                    incision.Area += h * h / n[axis] * math.saturate(0.5f - sdf / h);
                }
                incision.Depth = math.max(incision.Depth, -sdf);
                incision.SeveredEdges++;
                if (math.abs(sdf) <= 0.5f * h)
                {
                    trace.Add(math.transform(sim.RestToWorld, rest));
                }
            }
            incision.Area *= scale * scale;
            incision.Depth *= scale;
            if (trace.Count > 0)
            {
                float3 center = float3.zero;
                foreach (float3 p in trace)
                {
                    center += p;
                }
                center /= trace.Count;
                float3 direction = PrincipalDirection(trace, center);
                float lo = float.MaxValue, hi = float.MinValue;
                foreach (float3 p in trace)
                {
                    float s = math.dot(p - center, direction);
                    lo = math.min(lo, s);
                    hi = math.max(hi, s);
                }
                incision.Length = hi - lo;
                incision.Start = center + lo * direction;
                incision.End = center + hi * direction;
                incision.Center = center;
            }
            return incision;
        }

        /// <summary>Direction of the largest spread of the points about their centre (power iteration on the covariance).</summary>
        private static float3 PrincipalDirection(List<float3> points, float3 center)
        {
            float3x3 covariance = float3x3.zero;
            foreach (float3 p in points)
            {
                float3 d = p - center;
                covariance += new float3x3(d * d.x, d * d.y, d * d.z);
            }
            float3 direction = math.normalize(new float3(1.0f, 0.7f, 0.4f));
            for (int i = 0; i < 32; i++)
            {
                float3 next = math.mul(covariance, direction);
                if (math.lengthsq(next) <= 1e-20f)
                {
                    break;
                }
                direction = math.normalize(next);
            }
            return direction;
        }

        /// <summary>After a step: starts, extends and ends the times above the force limit of the body's instruments.</summary>
        private void MeasureForces(Observed observed)
        {
            ApxSoftBody body = observed.Body;
            float time = (float)body.Simulation.SimulatedTime;
            observed.Touched.Clear();
            foreach (KeyValuePair<Collider, ApxWrench> pair in body.ColliderWrenches)
            {
                if (pair.Key != null && Array.IndexOf(instruments, pair.Key) >= 0)
                {
                    Force(observed, pair.Key.name, math.length(pair.Value.Force), time);
                }
            }
            foreach (int grab in body.GrabHandles)
            {
                if (body.TryGetGrabWrench(grab, out ApxWrench wrench))
                {
                    Force(observed, $"grab {grab}", math.length(wrench.Force), time);
                }
            }
            observed.Released.Clear();
            foreach (string source in observed.Forces.Keys)
            {
                if (!observed.Touched.Contains(source))
                {
                    observed.Released.Add(source);
                }
            }
            foreach (string source in observed.Released)
            {
                _forceEvents.Add(observed.Forces[source]);
                observed.Forces.Remove(source);
            }
        }

        private void Force(Observed observed, string source, float force, float time)
        {
            if (force <= forceLimit)
            {
                return;
            }
            observed.Touched.Add(source);
            if (observed.Forces.TryGetValue(source, out ApxForceEvent ongoing))
            {
                ongoing.Duration = time - ongoing.Start;
                ongoing.Peak = Mathf.Max(ongoing.Peak, force);
                return;
            }
            var started = new ApxForceEvent { Body = observed.Body.name, Source = source, Start = time, Peak = force };
            observed.Forces[source] = started;
            ForceExceeded?.Invoke(new ApxForceEvent { Body = started.Body, Source = source, Start = time, Peak = force });
        }
    }
}
