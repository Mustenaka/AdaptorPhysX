using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.Compilation;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.SceneManagement;
using Debug = UnityEngine.Debug;

namespace AdaptorPhysX.Benchmarks.Editor
{
    /// <summary>
    /// Measures initialization, stepping, cutting and rendering for a set of meshes and lattice
    /// resolutions in an empty scene, and writes the results as JSON (<see cref="BenchmarkReport"/>).
    /// Each case: rest (no gravity, no colliders), drop onto a box collider, a full cut through the
    /// middle with a vertical blade, and steps after the cut. Batch use (the project must not be open
    /// in another editor):
    /// Unity -batchmode -projectPath . -executeMethod AdaptorPhysX.Benchmarks.Editor.ApxBenchmark.RunBatch
    ///   [-apxBenchmarkOut &lt;file.json&gt;] [-apxMeshes box,bunny,liver,dragon] [-apxResolutions 16,32,48,64]
    ///   [-apxSubsteps 16] [-apxSteps 60] [-apxBaseline &lt;baseline.json&gt;|none] [-apxWriteBaseline]
    ///   [-apxFailOnBaselineDifference]
    /// The run is compared with the baseline (<see cref="BenchmarkBaseline"/>, default Benchmarks/baseline.json
    /// when it exists); -apxWriteBaseline replaces the baseline with this run instead.
    /// </summary>
    public static class ApxBenchmark
    {
        public const string Schema = "apx-benchmark/1";
        public const string ResultDirectory = "Benchmarks/Results";
        private const float Dt = 1.0f / 60.0f;
        private const int WarmupSteps = 10;

        public sealed class Settings
        {
            /// <summary>Names of <see cref="BenchmarkMeshes"/>: Benchmarks/Meshes/&lt;name&gt;.obj, or box / liver.</summary>
            public string[] Meshes = { "box", "liver", "bunny", "dragon" };
            public int[] Resolutions = { 16, 32, 48, 64 };
            public int Substeps = 16;
            /// <summary>Measured steps of the rest, cut and after-cut phases.</summary>
            public int Steps = 60;
            public int DropSteps = 90;
            /// <summary>Longest axis of every mesh, in metres.</summary>
            public float Size = 0.15f;
            /// <summary>Output file; defaults to Benchmarks/Results/apx-benchmark-&lt;time&gt;.json.</summary>
            public string Output;
            /// <summary>Baseline to compare with (or to write); "none" skips the comparison.</summary>
            public string Baseline = BenchmarkBaseline.DefaultPath;
            /// <summary>Writes this run as the new baseline instead of comparing with it.</summary>
            public bool WriteBaseline;
        }

        private struct FrameTime
        {
            public double Sweep, Step, Render, Topology;
            public int Severed;
            public SurfaceMesh.UpdateTimings Stages;
        }

        [MenuItem("AdaptorPhysX/Run Benchmarks")]
        public static void RunMenu()
        {
            if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                EditorUtility.RevealInFinder(RunKeepingScenes(new Settings()));
            }
        }

        [MenuItem("AdaptorPhysX/Run Benchmarks and Write Baseline")]
        public static void RunWriteBaselineMenu()
        {
            if (EditorUtility.DisplayDialog("AdaptorPhysX Benchmark",
                    $"Run the benchmark and replace {BenchmarkBaseline.DefaultPath} with its results?", "Run", "Cancel") &&
                EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                EditorUtility.RevealInFinder(RunKeepingScenes(new Settings { WriteBaseline = true }));
            }
        }

        [MenuItem("AdaptorPhysX/Run Benchmarks", true)]
        [MenuItem("AdaptorPhysX/Run Benchmarks and Write Baseline", true)]
        private static bool CanRun() => !EditorApplication.isPlayingOrWillChangePlaymode;

        /// <summary>
        /// <see cref="Run"/> in the open editor, then reopens the scenes that were open. Refuses to run
        /// with unsaved scene changes, which the benchmark scene would discard.
        /// </summary>
        public static string RunKeepingScenes(Settings settings)
        {
            for (int i = 0; i < SceneManager.sceneCount; i++)
            {
                if (SceneManager.GetSceneAt(i).isDirty)
                {
                    throw new InvalidOperationException("Save the open scenes first: the benchmark runs in a new empty scene.");
                }
            }
            SceneSetup[] setup = EditorSceneManager.GetSceneManagerSetup();
            try
            {
                return Run(settings, out _);
            }
            finally
            {
                EditorUtility.ClearProgressBar();
                if (setup.Length > 0 && Array.TrueForAll(setup, s => !string.IsNullOrEmpty(s.path)))
                {
                    EditorSceneManager.RestoreSceneManagerSetup(setup);
                }
                else
                {
                    EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects);
                }
            }
        }

        /// <summary>
        /// Schedules <see cref="RunKeepingScenes"/> for the next editor update and returns at once. For
        /// editor automation with a reply timeout, which may send a call that has not returned again.
        /// Uses <see cref="EditorApplication.update"/>: <see cref="EditorApplication.delayCall"/> waits for an
        /// inspector update, which does not come while the editor window is in the background.
        /// </summary>
        public static void ScheduleKeepingScenes(Settings settings)
        {
            void RunOnce()
            {
                EditorApplication.update -= RunOnce;
                try
                {
                    RunKeepingScenes(settings);
                }
                catch (Exception e)
                {
                    Debug.LogException(e);
                }
            }

            EditorApplication.update += RunOnce;
        }

        public static void RunBatch()
        {
            int code = 0;
            try
            {
                string[] args = Environment.GetCommandLineArgs();
                Run(ParseArguments(args), out BenchmarkReport report);
                code = report.cases.Any(c => c.status == "error") || report.meshes.Any(m => m.status == "error") ? 1 : 0;
                if (Array.IndexOf(args, "-apxFailOnBaselineDifference") >= 0 &&
                    (report.baseline == null || string.IsNullOrEmpty(report.baseline.path) || !report.baseline.Matches))
                {
                    code = 1;
                }
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                code = 1;
            }
            EditorApplication.Exit(code);
        }

        public static Settings ParseArguments(string[] args)
        {
            string Value(string name)
            {
                int i = Array.IndexOf(args, name);
                return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
            }

            var settings = new Settings { Output = Value("-apxBenchmarkOut") };
            if (Value("-apxMeshes") is string meshes)
            {
                settings.Meshes = meshes.Split(',');
            }
            if (Value("-apxResolutions") is string resolutions)
            {
                settings.Resolutions = resolutions.Split(',').Select(int.Parse).ToArray();
            }
            if (Value("-apxSubsteps") is string substeps)
            {
                settings.Substeps = int.Parse(substeps);
            }
            if (Value("-apxSteps") is string steps)
            {
                settings.Steps = int.Parse(steps);
            }
            if (Value("-apxBaseline") is string baseline)
            {
                settings.Baseline = baseline;
            }
            settings.WriteBaseline = Array.IndexOf(args, "-apxWriteBaseline") >= 0;
            return settings;
        }

        /// <summary>Runs every mesh at every resolution in a new empty scene; returns the full path of the JSON file.</summary>
        public static string Run(Settings settings, out BenchmarkReport report)
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            DateTime start = DateTime.Now;
            string path = Path.GetFullPath(settings.Output ??
                Path.Combine(ResultDirectory, $"apx-benchmark-{start:yyyyMMdd-HHmmss}.json"));
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            report = new BenchmarkReport
            {
                schema = Schema,
                timestamp = start.ToString("o"),
                gitCommit = GitCommit(),
                environment = DescribeEnvironment(),
                settings = new SettingsInfo
                {
                    meshes = settings.Meshes,
                    resolutions = settings.Resolutions,
                    dt = Math.Round(Dt, 6),
                    substeps = settings.Substeps,
                    warmupSteps = WarmupSteps,
                    steps = settings.Steps,
                    dropSteps = settings.DropSteps,
                    sizeM = Math.Round(settings.Size, 4),
                },
            };

            WarmUp(settings);
            int total = settings.Meshes.Length * settings.Resolutions.Length, done = 0;
            foreach (string name in settings.Meshes)
            {
                var info = new MeshInfo { name = name };
                Mesh mesh = null;
                try
                {
                    mesh = BenchmarkMeshes.Load(name, settings.Size, info);
                    info.status = mesh != null ? "ok" : "skipped";
                }
                catch (Exception e)
                {
                    Debug.LogException(e);
                    info.status = "error";
                    info.error = $"{e.GetType().Name}: {e.Message}";
                }
                report.meshes.Add(info);
                Debug.Log($"[ApxBenchmark] mesh {name}: {info.status} {info.source} vertices {info.vertices} " +
                          $"triangles {info.triangles} boundary edges {info.boundaryEdges} load {info.loadMs:0} ms");

                foreach (int resolution in settings.Resolutions)
                {
                    if (!Application.isBatchMode)
                    {
                        EditorUtility.DisplayProgressBar("AdaptorPhysX Benchmark", $"{name}, resolution {resolution}",
                            done / (float)total);
                    }
                    CaseResult result = mesh != null
                        ? RunCase(mesh, name, resolution, settings, report.settings)
                        : new CaseResult { mesh = name, resolution = resolution, status = "skipped" };
                    report.cases.Add(result);
                    LogCase(result);
                    // Written after every case so that a long run that is stopped keeps its results.
                    File.WriteAllText(path, JsonUtility.ToJson(report, true));
                    done++;
                }
                if (mesh != null)
                {
                    UnityEngine.Object.DestroyImmediate(mesh);
                }
            }
            ApplyBaseline(settings, report);
            File.WriteAllText(path, JsonUtility.ToJson(report, true));
            Debug.Log($"[ApxBenchmark] wrote {path}");
            return path;
        }

        /// <summary>Writes the run as the baseline, or compares it with the baseline and logs the differences.</summary>
        private static void ApplyBaseline(Settings settings, BenchmarkReport report)
        {
            if (string.IsNullOrEmpty(settings.Baseline) || settings.Baseline == "none")
            {
                return;
            }
            if (settings.WriteBaseline)
            {
                BenchmarkBaseline.FromReport(report).Save(settings.Baseline);
                Debug.Log($"[ApxBenchmark] wrote the baseline {Path.GetFullPath(settings.Baseline)}");
                return;
            }
            if (!File.Exists(settings.Baseline))
            {
                Debug.Log($"[ApxBenchmark] no baseline at {settings.Baseline}; not compared");
                return;
            }
            BaselineComparison comparison = BenchmarkBaseline.Load(settings.Baseline).Compare(report, settings.Baseline);
            report.baseline = comparison;
            foreach (string difference in comparison.environmentDifferences)
            {
                Debug.LogWarning($"[ApxBenchmark] environment differs from the baseline, hashes may differ: {difference}");
            }
            foreach (string difference in comparison.settingsDifferences)
            {
                Debug.LogWarning($"[ApxBenchmark] not compared with the baseline: {difference}");
            }
            foreach (BaselineDifference difference in comparison.differences)
            {
                Debug.LogWarning($"[ApxBenchmark] differs from the baseline: {difference}");
            }
            Debug.Log($"[ApxBenchmark] baseline {settings.Baseline} ({comparison.baselineCommit}): " +
                      $"{comparison.matching} of {comparison.compared} cases match" +
                      (comparison.notCompared.Count > 0 ? $", not compared: {string.Join(", ", comparison.notCompared)}" : ""));
        }

        /// <summary>A small case that is not reported: Burst compilation and first use of the managed code happen here.</summary>
        private static void WarmUp(Settings settings)
        {
            Mesh mesh = MeshUtil.CreateBox(new Vector3(0.1f, 0.05f, 0.08f), 4);
            var quick = new Settings { Substeps = settings.Substeps, Steps = 4, DropSteps = 4 };
            CaseResult result = RunCase(mesh, "warm-up", 8, quick, new SettingsInfo());
            UnityEngine.Object.DestroyImmediate(mesh);
            if (result.status != "ok")
            {
                throw new InvalidOperationException("Warm-up case failed: " + result.error);
            }
        }

        private static CaseResult RunCase(Mesh mesh, string name, int resolution, Settings settings, SettingsInfo info)
        {
            var result = new CaseResult { mesh = name, resolution = resolution, status = "ok" };
            var created = new List<GameObject>();
            ApxSoftBody body = null;
            try
            {
                var bodyObject = new GameObject($"Benchmark {name} {resolution}");
                created.Add(bodyObject);
                bodyObject.AddComponent<MeshFilter>().sharedMesh = mesh;
                bodyObject.AddComponent<MeshRenderer>();
                body = bodyObject.AddComponent<ApxSoftBody>();
                body.sourceMesh = mesh;
                body.resolution = resolution;
                body.substeps = settings.Substeps;
                body.useGravity = false;
                body.allowSleep = false;
                info.youngsModulus = body.youngsModulus;
                info.poissonRatio = Math.Round(body.poissonRatio, 4);
                info.density = body.density;

                long allocated = Profiler.GetTotalAllocatedMemoryLong();
                body.Initialize();
                if (!body.IsInitialized)
                {
                    throw new InvalidOperationException("ApxSoftBody.Initialize failed; see the log.");
                }
                ApxSoftBody.InitializeTimings timings = body.LastInitializeTimings;
                result.init = new InitResult
                {
                    totalMs = Stats.Round(timings.Total),
                    prepareMs = Stats.Round(timings.Prepare),
                    voxelizeMs = Stats.Round(timings.Voxelize),
                    simulationMs = Stats.Round(timings.Simulation),
                    surfaceMs = Stats.Round(timings.Surface),
                    allocatedMB = Stats.Round((Profiler.GetTotalAllocatedMemoryLong() - allocated) / (1024.0 * 1024.0)),
                };
                SoftBodySimulation sim = body.Simulation;
                result.cellSizeMm = Stats.Round(sim.CellSize * 1000.0);
                result.dims = new[] { sim.Grid.Dims.x, sim.Grid.Dims.y, sim.Grid.Dims.z };
                result.cells = sim.CellCount;
                result.copies = sim.CopyCount;
                result.nodes = sim.NodeCount;
                result.tets = sim.TetCount;
                result.renderVertices = body.Surface.Mesh.vertexCount;
                result.renderTriangles = body.Surface.SkinTriangleCount;
                double restVolume = SkinVolume(body);

                RunFrames(body, null, WarmupSteps, null, out _);
                result.rest = Summarize(body, RunFrames(body, null, settings.Steps, null, out double bytes), bytes);

                // Drop a few millimetres onto a box collider and settle.
                float floorTop = body.WorldBounds.min.y - 0.005f;
                var floor = new GameObject("Benchmark Floor");
                created.Add(floor);
                floor.transform.position = new Vector3(body.WorldBounds.center.x, floorTop - 0.05f, body.WorldBounds.center.z);
                floor.AddComponent<BoxCollider>().size = new Vector3(1.0f, 0.1f, 1.0f);
                Physics.SyncTransforms();
                body.useGravity = true;
                result.drop = new DropResult
                {
                    frames = Summarize(body, RunFrames(body, null, settings.DropSteps, null, out bytes), bytes),
                    volumeRatio = Math.Round(SkinVolume(body) / restVolume, 4),
                    floorPenetrationMm = Stats.Round((floorTop - LowestRealNode(sim)) * 1000.0),
                };

                // Full cut: a vertical blade taller than the body crosses it along x. The blade plane runs
                // through the middle of the central cell layer of the rest lattice, as in the results of
                // DESIGN §6.1, or through the middle of the body when that layer has left it (a body that
                // tipped over on landing).
                var scalpel = new GameObject("Benchmark Scalpel").AddComponent<ApxScalpel>();
                created.Add(scalpel.gameObject);
                Bounds bounds = body.WorldBounds;
                float margin = 2.0f * sim.CellSize;
                float3 restMiddle = sim.Grid.Origin + ((float3)(sim.Grid.Dims / 2) + 0.5f) * sim.CellSize;
                float z = math.transform(sim.RestToWorld, restMiddle).z;
                if (z < bounds.min.z || z > bounds.max.z)
                {
                    z = bounds.center.z;
                }
                scalpel.edgeStart = new Vector3(0.0f, bounds.extents.y + margin, 0.0f);
                scalpel.edgeEnd = new Vector3(0.0f, -bounds.extents.y - margin, 0.0f);
                var from = new Vector3(bounds.min.x - margin, bounds.center.y, z);
                var to = new Vector3(bounds.max.x + margin, bounds.center.y, z);
                scalpel.transform.position = from;
                scalpel.cutting = true;
                scalpel.ResetSweep();
                List<FrameTime> frames = RunFrames(body, scalpel, settings.Steps + 1,
                    f => scalpel.transform.position = Vector3.Lerp(from, to, f / (float)settings.Steps), out bytes);
                scalpel.cutting = false;
                List<FrameTime> cutting = frames.Where(f => f.Severed > 0).ToList();
                List<int> pieces = sim.PieceSizes();
                result.cut = new CutResult
                {
                    frames = Summarize(body, frames, bytes),
                    cutFrames = cutting.Count,
                    severedEdges = cutting.Sum(f => f.Severed),
                    sweep = Stats.Of(cutting.Select(f => f.Sweep).ToList()),
                    topology = Stats.Of(cutting.Select(f => f.Topology).ToList()),
                    renderRebuild = Stats.Of(cutting.Select(f => f.Render).ToList()),
                    renderRebuildStages = RenderStages.Of(cutting.Select(f => f.Stages).ToList()),
                    firstCutFrameMs = cutting.Count > 0 ? Stats.Round(cutting[0].Sweep + cutting[0].Step + cutting[0].Render) : 0.0,
                    pieces = pieces.Count,
                    largestPieceNodes = pieces.Take(4).ToArray(),
                    copies = sim.CopyCount,
                    nodes = sim.NodeCount,
                    cutTriangles = body.Surface.CutTriangleCount,
                };
                result.afterCut = Summarize(body, RunFrames(body, null, settings.Steps, null, out bytes), bytes);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                result.status = "error";
                result.error = $"{e.GetType().Name}: {e.Message}";
            }
            finally
            {
                body?.Release();
                foreach (GameObject go in created)
                {
                    UnityEngine.Object.DestroyImmediate(go);
                }
            }
            return result;
        }

        /// <summary>
        /// Steps the body <paramref name="count"/> times as the capture tool does: blade sweep, step,
        /// surface update. <paramref name="managedBytes"/> is the growth of the managed heap per frame, or -1
        /// when a collection ran in between (Unity's Mono has no per-thread allocation counter).
        /// </summary>
        private static List<FrameTime> RunFrames(ApxSoftBody body, ApxScalpel scalpel, int count, Action<int> beforeFrame,
            out double managedBytes)
        {
            var frames = new List<FrameTime>(count);
            var timer = new Stopwatch();
            int collections = GC.CollectionCount(0);
            long used = Profiler.GetMonoUsedSizeLong();
            for (int f = 0; f < count; f++)
            {
                beforeFrame?.Invoke(f);
                var frame = new FrameTime();
                timer.Restart();
                if (scalpel != null)
                {
                    scalpel.Sweep();
                    frame.Severed = scalpel.LastSeveredEdges;
                    frame.Topology = frame.Severed > 0 ? body.Simulation.LastTopologyMilliseconds : 0.0;
                }
                frame.Sweep = Lap(timer);
                body.Simulate(Dt);
                body.Simulation.CompleteStep();
                frame.Step = Lap(timer);
                body.UpdateRender();
                frame.Render = Lap(timer);
                frame.Stages = body.Surface.LastTimings;
                frames.Add(frame);
            }
            managedBytes = GC.CollectionCount(0) == collections
                ? (Profiler.GetMonoUsedSizeLong() - used) / (double)math.max(count, 1)
                : -1.0;
            return frames;
        }

        private static PhaseResult Summarize(ApxSoftBody body, List<FrameTime> frames, double managedBytes)
        {
            SoftBodySimulation sim = body.Simulation;
            var result = new PhaseResult
            {
                steps = frames.Count,
                step = Stats.Of(frames.Select(f => f.Step).ToList()),
                render = Stats.Of(frames.Select(f => f.Render).ToList()),
                renderStages = RenderStages.Of(frames.Select(f => f.Stages).ToList()),
                managedKBPerStep = managedBytes < 0.0 ? -1.0 : Stats.Round(managedBytes / 1024.0),
                maxSpeed = Math.Round(sim.MaxSpeed, 6),
                finite = true,
            };
            ulong hash = 14695981039346656037UL;
            for (int n = 0; n < sim.NodeCount; n++)
            {
                float3 p = sim.Positions[n];
                result.finite &= math.all(math.isfinite(p));
                uint3 bits = math.asuint(p);
                hash = Fnv(Fnv(Fnv(hash, bits.x), bits.y), bits.z);
            }
            result.positionsHash = hash.ToString("x16");
            return result;
        }

        private static ulong Fnv(ulong hash, uint value)
        {
            for (int b = 0; b < 4; b++)
            {
                hash = (hash ^ ((value >> (8 * b)) & 0xff)) * 1099511628211UL;
            }
            return hash;
        }

        /// <summary>Signed volume enclosed by the skin submesh (the rendered surface without cut faces).</summary>
        private static double SkinVolume(ApxSoftBody body)
        {
            Mesh mesh = body.Surface.Mesh;
            Vector3[] v = mesh.vertices;
            int[] t = mesh.GetTriangles(0);
            double volume = 0.0;
            for (int i = 0; i < t.Length; i += 3)
            {
                double3 a = (float3)v[t[i]], b = (float3)v[t[i + 1]], c = (float3)v[t[i + 2]];
                volume += math.dot(a, math.cross(b, c));
            }
            return volume / 6.0;
        }

        private static float LowestRealNode(SoftBodySimulation sim)
        {
            float lowest = float.MaxValue;
            for (int n = 0; n < sim.NodeCount; n++)
            {
                if (sim.NodeReal[n] != 0)
                {
                    lowest = math.min(lowest, sim.Positions[n].y);
                }
            }
            return lowest;
        }

        private static double Lap(Stopwatch timer)
        {
            double ms = timer.Elapsed.TotalMilliseconds;
            timer.Restart();
            return ms;
        }

        private static void LogCase(CaseResult r)
        {
            if (r.status != "ok")
            {
                Debug.Log($"[ApxBenchmark] {r.mesh} {r.resolution}: {r.status} {r.error}");
                return;
            }
            Debug.Log($"[ApxBenchmark] {r.mesh} {r.resolution}: cells {r.cells} nodes {r.nodes} h {r.cellSizeMm:0.0} mm | " +
                      $"init {r.init.totalMs:0} ms (voxelize {r.init.voxelizeMs:0}, surface {r.init.surfaceMs:0}) | " +
                      $"rest step p50 {r.rest.step.p50:0.00} p95 {r.rest.step.p95:0.00}, render p50 {r.rest.render.p50:0.00} | " +
                      $"cut sweep p95 {r.cut.sweep.p95:0.00}, render rebuild p95 {r.cut.renderRebuild.p95:0.00}, " +
                      $"pieces {r.cut.pieces} | after cut step p50 {r.afterCut.step.p50:0.00} | volume {r.drop.volumeRatio:0.000}");
        }

        private static EnvironmentInfo DescribeEnvironment() => new EnvironmentInfo
        {
            unityVersion = Application.unityVersion,
            os = SystemInfo.operatingSystem,
            cpu = SystemInfo.processorType,
            logicalProcessors = SystemInfo.processorCount,
            jobWorkers = JobsUtility.JobWorkerCount,
            systemMemoryMB = SystemInfo.systemMemorySize,
            editor = Application.isEditor,
            batchMode = Application.isBatchMode,
            codeOptimization = CompilationPipeline.codeOptimization.ToString(),
            burstEnabled = BurstCompiler.IsEnabled,
            jobsDebugger = JobsUtility.JobDebuggerEnabled,
            leakDetection = NativeLeakDetection.Mode.ToString(),
        };

        /// <summary>Commit of the working tree from .git, or null.</summary>
        private static string GitCommit()
        {
            try
            {
                string head = File.ReadAllText(".git/HEAD").Trim();
                if (!head.StartsWith("ref: "))
                {
                    return head;
                }
                string reference = head.Substring(5);
                string file = Path.Combine(".git", reference);
                if (File.Exists(file))
                {
                    return File.ReadAllText(file).Trim();
                }
                foreach (string line in File.ReadLines(".git/packed-refs"))
                {
                    if (line.EndsWith(" " + reference))
                    {
                        return line.Substring(0, line.IndexOf(' '));
                    }
                }
            }
            catch (IOException)
            {
            }
            return null;
        }
    }
}
