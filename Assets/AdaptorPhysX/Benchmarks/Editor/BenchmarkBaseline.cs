using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace AdaptorPhysX.Benchmarks.Editor
{
    /// <summary>
    /// Reference results of the benchmark (<c>Benchmarks/baseline.json</c>, committed): for every case the
    /// node position hashes of the four phases, the pieces and the severed edges of the full cut. A run is
    /// compared with it to tell whether a change altered the simulation. Hashes are only comparable on the
    /// same machine and build; the environment is recorded and differences in it are reported.
    /// </summary>
    [Serializable]
    public sealed class BenchmarkBaseline
    {
        public const string Schema = "apx-benchmark-baseline/1";
        public const string DefaultPath = "Benchmarks/baseline.json";

        public string schema;
        public string timestamp;
        public string gitCommit;
        public EnvironmentInfo environment;
        public SettingsInfo settings;
        public List<MeshInfo> meshes = new List<MeshInfo>();
        public List<BaselineCase> cases = new List<BaselineCase>();

        public static BenchmarkBaseline FromReport(BenchmarkReport report)
        {
            var baseline = new BenchmarkBaseline
            {
                schema = Schema,
                timestamp = report.timestamp,
                gitCommit = report.gitCommit,
                environment = report.environment,
                settings = report.settings,
                meshes = report.meshes,
            };
            foreach (CaseResult c in report.cases)
            {
                if (c.status == "ok")
                {
                    baseline.cases.Add(BaselineCase.Of(c));
                }
            }
            return baseline;
        }

        public static BenchmarkBaseline Load(string path)
        {
            var baseline = JsonUtility.FromJson<BenchmarkBaseline>(File.ReadAllText(path));
            if (baseline == null || baseline.schema != Schema)
            {
                throw new InvalidDataException($"{path} is not a benchmark baseline ({Schema}).");
            }
            return baseline;
        }

        public void Save(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(path)));
            File.WriteAllText(path, JsonUtility.ToJson(this, true));
        }

        /// <summary>Compares the cases present in both; settings that change the results must be equal.</summary>
        public BaselineComparison Compare(BenchmarkReport report, string path)
        {
            var comparison = new BaselineComparison { path = path.Replace('\\', '/'), baselineCommit = gitCommit };
            comparison.environmentDifferences = EnvironmentDifferences(environment, report.environment);
            comparison.settingsDifferences = SettingsDifferences(settings, report.settings);
            foreach (MeshInfo mesh in report.meshes)
            {
                MeshInfo reference = meshes.Find(m => m.name == mesh.name);
                if (reference != null && mesh.status == "ok" &&
                    (reference.vertices != mesh.vertices || reference.triangles != mesh.triangles))
                {
                    comparison.settingsDifferences.Add($"mesh {mesh.name}: {reference.vertices} vertices / " +
                        $"{reference.triangles} triangles in the baseline, {mesh.vertices} / {mesh.triangles} now");
                }
            }
            if (comparison.settingsDifferences.Count > 0)
            {
                return comparison;
            }
            foreach (CaseResult c in report.cases)
            {
                BaselineCase reference = cases.Find(b => b.mesh == c.mesh && b.resolution == c.resolution);
                if (reference == null || c.status != "ok")
                {
                    comparison.notCompared.Add($"{c.mesh} {c.resolution}" + (reference == null ? " (not in the baseline)" : $" ({c.status})"));
                    continue;
                }
                comparison.compared++;
                int before = comparison.differences.Count;
                reference.CompareWith(BaselineCase.Of(c), comparison.differences);
                comparison.matching += comparison.differences.Count == before ? 1 : 0;
            }
            return comparison;
        }

        private static List<string> EnvironmentDifferences(EnvironmentInfo a, EnvironmentInfo b)
        {
            var differences = new List<string>();
            void Check(string field, object x, object y)
            {
                if (!Equals(x, y))
                {
                    differences.Add($"{field}: {x} in the baseline, {y} now");
                }
            }

            Check(nameof(a.unityVersion), a.unityVersion, b.unityVersion);
            Check(nameof(a.cpu), a.cpu, b.cpu);
            Check(nameof(a.os), a.os, b.os);
            Check(nameof(a.codeOptimization), a.codeOptimization, b.codeOptimization);
            Check(nameof(a.burstEnabled), a.burstEnabled, b.burstEnabled);
            return differences;
        }

        private static List<string> SettingsDifferences(SettingsInfo a, SettingsInfo b)
        {
            var differences = new List<string>();
            void Check(string field, double x, double y)
            {
                if (x != y)
                {
                    differences.Add($"{field}: {x} in the baseline, {y} now");
                }
            }

            Check(nameof(a.dt), a.dt, b.dt);
            Check(nameof(a.substeps), a.substeps, b.substeps);
            Check(nameof(a.warmupSteps), a.warmupSteps, b.warmupSteps);
            Check(nameof(a.steps), a.steps, b.steps);
            Check(nameof(a.dropSteps), a.dropSteps, b.dropSteps);
            Check(nameof(a.sizeM), a.sizeM, b.sizeM);
            Check(nameof(a.youngsModulus), a.youngsModulus, b.youngsModulus);
            Check(nameof(a.poissonRatio), a.poissonRatio, b.poissonRatio);
            Check(nameof(a.density), a.density, b.density);
            return differences;
        }
    }

    [Serializable]
    public sealed class BaselineCase
    {
        public string mesh;
        public int resolution;
        public string restHash;
        public string dropHash;
        public string cutHash;
        public string afterCutHash;
        public int pieces;
        public int severedEdges;
        public int[] largestPieceNodes;

        public static BaselineCase Of(CaseResult c) => new BaselineCase
        {
            mesh = c.mesh,
            resolution = c.resolution,
            restHash = c.rest.positionsHash,
            dropHash = c.drop.frames.positionsHash,
            cutHash = c.cut.frames.positionsHash,
            afterCutHash = c.afterCut.positionsHash,
            pieces = c.cut.pieces,
            severedEdges = c.cut.severedEdges,
            largestPieceNodes = c.cut.largestPieceNodes,
        };

        public void CompareWith(BaselineCase current, List<BaselineDifference> differences)
        {
            void Check(string field, string expected, string actual)
            {
                if (expected != actual)
                {
                    differences.Add(new BaselineDifference
                        { mesh = mesh, resolution = resolution, field = field, baseline = expected, current = actual });
                }
            }

            Check(nameof(restHash), restHash, current.restHash);
            Check(nameof(dropHash), dropHash, current.dropHash);
            Check(nameof(cutHash), cutHash, current.cutHash);
            Check(nameof(afterCutHash), afterCutHash, current.afterCutHash);
            Check(nameof(pieces), pieces.ToString(), current.pieces.ToString());
            Check(nameof(severedEdges), severedEdges.ToString(), current.severedEdges.ToString());
            Check(nameof(largestPieceNodes), string.Join("/", largestPieceNodes ?? new int[0]),
                string.Join("/", current.largestPieceNodes ?? new int[0]));
        }
    }

    [Serializable]
    public sealed class BaselineDifference
    {
        public string mesh;
        public int resolution;
        public string field;
        public string baseline;
        public string current;

        public override string ToString() => $"{mesh} {resolution} {field}: {baseline} -> {current}";
    }

    /// <summary>Result of comparing a run with the baseline, stored in the report.</summary>
    [Serializable]
    public sealed class BaselineComparison
    {
        public string path;
        public string baselineCommit;
        /// <summary>Machine or build differences; the hashes are then not expected to match.</summary>
        public List<string> environmentDifferences = new List<string>();
        /// <summary>Settings or meshes that differ from the baseline; no case is compared then.</summary>
        public List<string> settingsDifferences = new List<string>();
        public int compared;
        public int matching;
        public List<string> notCompared = new List<string>();
        public List<BaselineDifference> differences = new List<BaselineDifference>();

        public bool Matches => settingsDifferences.Count == 0 && differences.Count == 0;
    }
}
