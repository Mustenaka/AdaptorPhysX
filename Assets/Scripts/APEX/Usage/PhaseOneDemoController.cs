using System;
using APEX.Native;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace APEX.Usage
{
    /// <summary>
    /// Phase 1 integration scene: deterministic 100k cloth generation,
    /// fixed-tick replay, live blade binding, native render readback, and a
    /// separate precise-cut preview driven by the same query.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public sealed class PhaseOneDemoController : MonoBehaviour
    {
        [Header("固定 workload")]
        public ApxBackendKind backend = ApxBackendKind.Cuda;
        public bool allowCpuFallback = true;
        public int columns = 400;
        public int rows = 250;
        public float spacing = 0.02F;
        public uint solverIterations = 2U;
        public float particleRadius = 0.006F;
        public float warpCompliance = 1.0e-7F;
        public float weftCompliance = 2.0e-6F;
        public float breakThreshold = 100.0F;

        [Header("回放 / 交互")]
        public ulong scriptedCutTick = 1U;
        public float cutRadius = 0.001F;
        public bool enableMouseCut = true;
        public ApxBladeInteractor liveBlade;
        public ObjCut precisionCutPreview;
        public float scriptedCutVerticalPadding = 5.0F;

        private const float NativeFixedTimeStep = 1.0F / 120.0F;
        private const float NativeFrameDeltaTime = 1.0F / 60.0F;

        private NativeWorld _world;
        private PhaseOneReplayTimeline _timeline;
        private PhaseOneReplayCursor _cursor;
        private ApxVec3[] _renderPositions;
        private ApxVec3[] _renderNormals;
        private NativeArray<Vector3> _unityPositions;
        private NativeArray<Vector3> _unityNormals;
        private Mesh _mesh;
        private ulong _fixedTick;
        private ulong _stateHash;

        public ulong FixedTick => _fixedTick;

        public ulong StateHash => _stateHash;

        public uint ParticleCount => _world != null ? _world.ParticleCount : 0U;

        private void Start()
        {
            try
            {
                Initialize();
            }
            catch (Exception exception)
            {
                enabled = false;
                Debug.LogException(exception, this);
            }
        }

        private void Update()
        {
            if (liveBlade != null && enableMouseCut)
            {
                liveBlade.SetCutHeld(Input.GetMouseButton(0));
            }
        }

        private void FixedUpdate()
        {
            if (_world == null)
            {
                return;
            }
            if (_fixedTick == ulong.MaxValue)
            {
                enabled = false;
                throw new InvalidOperationException("Phase 1 demo fixed-tick counter is exhausted.");
            }

            ++_fixedTick;
            while (_cursor.TryDequeue(_fixedTick, out PhaseOneReplayCommand command))
            {
                ApxCutResult cut = _world.Cut(command.Query);
                PhaseOneReplayHash.Append(ref _stateHash, command);
                PhaseOneReplayHash.Append(ref _stateHash, cut);
                if (precisionCutPreview != null)
                {
                    precisionCutPreview.ApplyPreciseWorldCut(command.Query);
                }
            }

            _world.Step(NativeFrameDeltaTime);
            // TODO(phase4-zerocopy): replace both blocking readbacks with CUDA
            // external-memory interop into a Unity GraphicsBuffer.
            int written = _world.GetRenderVertexPositions(_renderPositions);
            int normalWritten = _world.GetRenderVertexNormals(_renderNormals);
            if (written != normalWritten)
            {
                throw new InvalidOperationException("Render position/normal counts diverged.");
            }
            for (int index = 0; index < written; ++index)
            {
                ApxVec3 position = _renderPositions[index];
                ApxVec3 normal = _renderNormals[index];
                _unityPositions[index] = new Vector3(position.X, position.Y, position.Z);
                _unityNormals[index] = new Vector3(normal.X, normal.Y, normal.Z);
            }
            _mesh.SetVertices(_unityPositions);
            _mesh.SetNormals(_unityNormals);
            _mesh.RecalculateBounds();
            PhaseOneReplayHash.Append(ref _stateHash, _renderPositions, written);
        }

        private void OnDestroy()
        {
            _world?.Dispose();
            _world = null;
            if (_unityPositions.IsCreated)
            {
                _unityPositions.Dispose();
            }
            if (_unityNormals.IsCreated)
            {
                _unityNormals.Dispose();
            }
            if (_mesh != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(_mesh);
                }
                else
                {
                    DestroyImmediate(_mesh);
                }
                _mesh = null;
            }
        }

        private void OnGUI()
        {
            GUI.Label(
                new Rect(12.0F, 12.0F, 700.0F, 24.0F),
                $"AdaptorPhysX Phase 1 | tick {_fixedTick} | particles {ParticleCount} | hash 0x{_stateHash:X16}");
            GUI.Label(
                new Rect(12.0F, 36.0F, 700.0F, 24.0F),
                "左键：fixed-tick 刀刃采样；scripted cut 与 live cut 共用 native/render query");
        }

        private void Initialize()
        {
            if (_world != null)
            {
                throw new InvalidOperationException("Phase 1 demo is already initialized.");
            }
            if (scriptedCutTick == 0U)
            {
                throw new ArgumentOutOfRangeException(nameof(scriptedCutTick));
            }
            if (float.IsNaN(scriptedCutVerticalPadding) ||
                float.IsInfinity(scriptedCutVerticalPadding) ||
                scriptedCutVerticalPadding < 0.0F)
            {
                throw new ArgumentOutOfRangeException(nameof(scriptedCutVerticalPadding));
            }

            PhaseOneClothWorkload workload = PhaseOneClothWorkload.Create(
                columns,
                rows,
                spacing,
                warpCompliance,
                weftCompliance,
                breakThreshold);
            _world = CreateWorld(workload.Particles.Length);
            try
            {
                _world.AddParticles(workload.Particles);
                _world.AddClothDistanceConstraints(workload.ClothConstraints);
                _world.SetRenderVertexBindings(workload.RenderBindings);
                _world.SetRenderTriangles(workload.TriangleIndices);
                _world.Step(NativeFixedTimeStep * 0.5F);

                ApxCutQuery query = workload.CreateVerticalCut(columns / 2, cutRadius);
                query.Start.Y += scriptedCutVerticalPadding;
                query.End.Y -= scriptedCutVerticalPadding;
                _timeline = new PhaseOneReplayTimeline(
                    new[]
                    {
                        new PhaseOneReplayCommand(0U, scriptedCutTick, query),
                    });
                _cursor = new PhaseOneReplayCursor(_timeline);
                _stateHash = PhaseOneReplayHash.Begin();
                PhaseOneReplayHash.Append(ref _stateHash, _timeline.ContentHash);
                _renderPositions = new ApxVec3[workload.RenderBindings.Length];
                _renderNormals = new ApxVec3[workload.RenderBindings.Length];
                _unityPositions = new NativeArray<Vector3>(
                    workload.Particles.Length,
                    Allocator.Persistent,
                    NativeArrayOptions.UninitializedMemory);
                _unityNormals = new NativeArray<Vector3>(
                    workload.Particles.Length,
                    Allocator.Persistent,
                    NativeArrayOptions.UninitializedMemory);
                for (int index = 0; index < workload.Particles.Length; ++index)
                {
                    ApxVec3 position = workload.Particles[index].Position;
                    _unityPositions[index] = new Vector3(position.X, position.Y, position.Z);
                    _unityNormals[index] = Vector3.forward;
                }
                BuildMesh(workload);
                if (liveBlade != null)
                {
                    liveBlade.BindNativeWorld(_world);
                }
            }
            catch
            {
                _world.Dispose();
                _world = null;
                throw;
            }
        }

        private NativeWorld CreateWorld(int particleCount)
        {
            ApxWorldDesc description = ApxWorldDesc.Create(
                backend,
                NativeFixedTimeStep,
                solverIterations,
                new ApxVec3(0.0F, -9.81F, 0.0F),
                particleRadius,
                checked((uint)particleCount));
            try
            {
                return NativeWorld.Create(description);
            }
            catch (ApxException exception)
                when (allowCpuFallback &&
                      backend == ApxBackendKind.Cuda &&
                      (exception.Result == ApxResult.Unsupported ||
                       exception.Result == ApxResult.BackendError))
            {
                description.Backend = ApxBackendKind.Cpu;
                Debug.LogWarning($"CUDA unavailable; Phase 1 demo uses CPU fallback: {exception.Message}");
                return NativeWorld.Create(description);
            }
        }

        private void BuildMesh(PhaseOneClothWorkload workload)
        {
            int[] triangles = new int[workload.TriangleIndices.Length];
            for (int index = 0; index < triangles.Length; ++index)
            {
                triangles[index] = checked((int)workload.TriangleIndices[index]);
            }
            Vector2[] uvs = new Vector2[_unityPositions.Length];
            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    int stableId = row * columns + column;
                    uvs[stableId] = new Vector2(
                        column / (float)(columns - 1),
                        1.0F - row / (float)(rows - 1));
                }
            }

            _mesh = new Mesh
            {
                name = "AdaptorPhysX_Phase1_100k_Cloth",
                indexFormat = IndexFormat.UInt32,
            };
            _mesh.SetVertices(_unityPositions);
            _mesh.SetNormals(_unityNormals);
            _mesh.uv = uvs;
            _mesh.triangles = triangles;
            _mesh.MarkDynamic();
            GetComponent<MeshFilter>().sharedMesh = _mesh;
        }
    }
}
