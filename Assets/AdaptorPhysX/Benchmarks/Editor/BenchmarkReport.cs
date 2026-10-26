using System;
using System.Collections.Generic;

namespace AdaptorPhysX.Benchmarks.Editor
{
    // JSON layout of the benchmark output (JsonUtility). Times in milliseconds unless the name says otherwise.

    [Serializable]
    public sealed class BenchmarkReport
    {
        public string schema;
        public string timestamp;
        public string gitCommit;
        public EnvironmentInfo environment;
        public SettingsInfo settings;
        public List<MeshInfo> meshes = new List<MeshInfo>();
        public List<CaseResult> cases = new List<CaseResult>();
    }

    [Serializable]
    public sealed class EnvironmentInfo
    {
        public string unityVersion;
        public string os;
        public string cpu;
        public int logicalProcessors;
        public int jobWorkers;
        public int systemMemoryMB;
        public bool editor;
        public bool batchMode;
        /// <summary>Managed code optimization of the editor (Debug or Release).</summary>
        public string codeOptimization;
        public bool burstEnabled;
        public bool jobsDebugger;
        public string leakDetection;
    }

    [Serializable]
    public sealed class SettingsInfo
    {
        public string[] meshes;
        public int[] resolutions;
        public double dt;
        public int substeps;
        public int warmupSteps;
        public int steps;
        public int dropSteps;
        /// <summary>Longest axis of every mesh after scaling.</summary>
        public double sizeM;
        public double youngsModulus;
        public double poissonRatio;
        public double density;
    }

    [Serializable]
    public sealed class MeshInfo
    {
        public string name;
        /// <summary>OBJ path relative to the project, or "procedural".</summary>
        public string source;
        /// <summary>ok, error, or skipped when there is neither a file nor a generator.</summary>
        public string status;
        public string error;
        public double loadMs;
        public int vertices;
        public int triangles;
        /// <summary>Edges with one triangle after welding equal positions; must be 0 for the voxelizer.</summary>
        public int boundaryEdges;
        public int nonManifoldEdges;
        public double[] sizeM;
    }

    [Serializable]
    public sealed class CaseResult
    {
        public string mesh;
        public int resolution;
        /// <summary>ok, skipped or error.</summary>
        public string status;
        public string error;
        public double cellSizeMm;
        public int[] dims;
        public int cells;
        public int copies;
        public int nodes;
        public int tets;
        public int renderVertices;
        public int renderTriangles;
        public InitResult init;
        /// <summary>No gravity and no colliders: the solver alone, as in DESIGN §6.</summary>
        public PhaseResult rest;
        /// <summary>Gravity on, falls a few millimetres onto a box collider and settles.</summary>
        public DropResult drop;
        /// <summary>A vertical blade sweeps through the middle of the settled body: full cut, 2 pieces expected.</summary>
        public CutResult cut;
        /// <summary>Steps after the cut, with the split topology.</summary>
        public PhaseResult afterCut;
    }

    [Serializable]
    public sealed class InitResult
    {
        public double totalMs;
        public double prepareMs;
        public double voxelizeMs;
        public double simulationMs;
        public double surfaceMs;
        /// <summary>Change of Unity's total allocated memory across Initialize (approximate in the editor).</summary>
        public double allocatedMB;
    }

    [Serializable]
    public sealed class PhaseResult
    {
        public int steps;
        /// <summary>Simulate + wait for the jobs (collider gathering included).</summary>
        public Stats step;
        /// <summary>Surface mesh update (skinning, render topology rebuild after a cut).</summary>
        public Stats render;
        /// <summary>Managed heap growth per step; -1 when a garbage collection ran during the phase.</summary>
        public double managedKBPerStep;
        /// <summary>False if any node position is NaN or infinite.</summary>
        public bool finite;
        /// <summary>Largest speed of a real node after the last step, in m/s.</summary>
        public double maxSpeed;
        /// <summary>FNV-1a of the node positions; equal inputs on the same machine and build give equal hashes.</summary>
        public string positionsHash;
    }

    [Serializable]
    public sealed class DropResult
    {
        public PhaseResult frames;
        /// <summary>Enclosed volume of the skin surface relative to the start.</summary>
        public double volumeRatio;
        /// <summary>Lowest real node below the floor top (positive = penetration).</summary>
        public double floorPenetrationMm;
    }

    [Serializable]
    public sealed class CutResult
    {
        public PhaseResult frames;
        /// <summary>Frames in which the blade severed at least one edge.</summary>
        public int cutFrames;
        public int severedEdges;
        /// <summary>Blade sweep on cutting frames: edge tests + simulation topology rebuild.</summary>
        public Stats sweep;
        /// <summary>Simulation topology rebuild alone on cutting frames.</summary>
        public Stats topology;
        /// <summary>Surface update on cutting frames (includes the render topology rebuild).</summary>
        public Stats renderRebuild;
        /// <summary>Sweep + step + render of the first cutting frame (includes first-use costs).</summary>
        public double firstCutFrameMs;
        public int pieces;
        public int[] largestPieceNodes;
        public int copies;
        public int nodes;
        public int cutTriangles;
    }

    [Serializable]
    public sealed class Stats
    {
        public int count;
        public double mean;
        public double min;
        public double p50;
        public double p95;
        public double p99;
        public double max;

        public static Stats Of(List<double> samples)
        {
            var stats = new Stats { count = samples.Count };
            if (samples.Count == 0)
            {
                return stats;
            }
            var sorted = new List<double>(samples);
            sorted.Sort();
            double sum = 0.0;
            foreach (double s in sorted)
            {
                sum += s;
            }
            stats.mean = Round(sum / sorted.Count);
            stats.min = Round(sorted[0]);
            stats.p50 = Round(Percentile(sorted, 0.50));
            stats.p95 = Round(Percentile(sorted, 0.95));
            stats.p99 = Round(Percentile(sorted, 0.99));
            stats.max = Round(sorted[sorted.Count - 1]);
            return stats;
        }

        public static double Round(double value) => Math.Round(value, 3);

        private static double Percentile(List<double> sorted, double p) =>
            sorted[Math.Min(sorted.Count - 1, Math.Max(0, (int)Math.Ceiling(p * sorted.Count) - 1))];
    }
}
