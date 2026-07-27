using System;
using APEX.Native;
using UnityEngine;
using UnityEngine.Rendering;

namespace APEX.Cutting
{
    /// <summary>
    /// Unity-facing bridge for deterministic fine render cuts and P1-4 coarse
    /// simulation cuts. Runtime geometry remains in AdaptorPhysX.Runtime.
    /// </summary>
    public static class PreciseMeshCutCoordinator
    {
        public static RenderMeshCutResult CutRender(
            GameObject target,
            ApxCutQuery worldQuery)
        {
            MeshFilter meshFilter = RequireMeshFilter(target);
            RenderMeshData input = ReadMesh(meshFilter.sharedMesh);
            RenderMeshCutResult result = RenderMeshCutter.Cut(
                input,
                ToLocalQuery(target.transform, worldQuery));
            Publish(meshFilter, target.GetComponent<MeshCollider>(), result);
            return result;
        }

        public static RenderMeshCutResult[] CutRenderPolyline(
            GameObject target,
            ApxCutQuery[] worldQueries)
        {
            if (worldQueries == null)
            {
                throw new ArgumentNullException(nameof(worldQueries));
            }

            MeshFilter meshFilter = RequireMeshFilter(target);
            RenderMeshData current = ReadMesh(meshFilter.sharedMesh);
            RenderMeshCutResult[] results = new RenderMeshCutResult[worldQueries.Length];
            bool changed = false;
            for (int segment = 0; segment < worldQueries.Length; ++segment)
            {
                RenderMeshCutResult result = RenderMeshCutter.Cut(
                    current,
                    ToLocalQuery(target.transform, worldQueries[segment]));
                results[segment] = result;
                current = result.Mesh;
                changed |= result.CutTriangleCount != 0U;
            }

            if (changed)
            {
                PublishMesh(
                    meshFilter,
                    target.GetComponent<MeshCollider>(),
                    current);
            }
            return results;
        }

        public static CoupledMeshCutResult CutCoupled(
            GameObject target,
            NativeWorld world,
            ApxCutQuery worldQuery)
        {
            MeshFilter meshFilter = RequireMeshFilter(target);
            RenderMeshData input = ReadMesh(meshFilter.sharedMesh);
            CoupledMeshCutResult result = CoupledMeshCutter.Cut(
                world,
                input,
                ToLocalQuery(target.transform, worldQuery),
                worldQuery);
            Publish(meshFilter, target.GetComponent<MeshCollider>(), result.Render);
            return result;
        }

        public static CoupledMeshCutResult[] CutCoupledPolyline(
            GameObject target,
            NativeWorld world,
            ApxCutQuery[] worldQueries)
        {
            if (worldQueries == null)
            {
                throw new ArgumentNullException(nameof(worldQueries));
            }

            CoupledMeshCutResult[] results =
                new CoupledMeshCutResult[worldQueries.Length];
            for (int segment = 0; segment < worldQueries.Length; ++segment)
            {
                // Each segment is committed in caller order. A failing native
                // commit never publishes that segment's render candidate.
                results[segment] = CutCoupled(target, world, worldQueries[segment]);
            }
            return results;
        }

        private static MeshFilter RequireMeshFilter(GameObject target)
        {
            if (target == null)
            {
                throw new ArgumentNullException(nameof(target));
            }

            MeshFilter meshFilter = target.GetComponent<MeshFilter>();
            if (meshFilter == null || meshFilter.sharedMesh == null)
            {
                throw new ArgumentException(
                    "The render cut target must have a MeshFilter with a mesh.",
                    nameof(target));
            }
            return meshFilter;
        }

        private static RenderMeshData ReadMesh(Mesh source)
        {
            Vector3[] sourcePositions = source.vertices;
            Vector3[] sourceNormals = source.normals;
            Vector2[] sourceUvs = source.uv;
            int[] sourceIndices = source.triangles;
            RenderMeshVertex[] vertices = new RenderMeshVertex[sourcePositions.Length];
            for (int index = 0; index < sourcePositions.Length; ++index)
            {
                Vector3 normal = sourceNormals.Length == sourcePositions.Length
                    ? sourceNormals[index]
                    : Vector3.zero;
                Vector2 uv = sourceUvs.Length == sourcePositions.Length
                    ? sourceUvs[index]
                    : Vector2.zero;
                vertices[index] = new RenderMeshVertex(
                    ToApx(sourcePositions[index]),
                    ToApx(normal),
                    uv.x,
                    uv.y);
            }

            uint[] indices = new uint[sourceIndices.Length];
            for (int index = 0; index < sourceIndices.Length; ++index)
            {
                indices[index] = checked((uint)sourceIndices[index]);
            }
            return new RenderMeshData(vertices, indices);
        }

        private static ApxCutQuery ToLocalQuery(
            Transform target,
            ApxCutQuery worldQuery)
        {
            Vector3 scale = target.lossyScale;
            float maximumScale = Mathf.Max(
                Mathf.Abs(scale.x),
                Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
            Vector3 localNormal = target.localToWorldMatrix.transpose
                .MultiplyVector(ToUnity(worldQuery.SideNormal))
                .normalized;
            return new ApxCutQuery(
                ToApx(target.InverseTransformPoint(ToUnity(worldQuery.Start))),
                ToApx(target.InverseTransformPoint(ToUnity(worldQuery.End))),
                ToApx(localNormal),
                worldQuery.Radius / Mathf.Max(maximumScale, 1.0e-6F));
        }

        private static void Publish(
            MeshFilter meshFilter,
            MeshCollider meshCollider,
            RenderMeshCutResult result)
        {
            if (result.CutTriangleCount == 0U)
            {
                return;
            }

            PublishMesh(meshFilter, meshCollider, result.Mesh);
        }

        private static void PublishMesh(
            MeshFilter meshFilter,
            MeshCollider meshCollider,
            RenderMeshData mesh)
        {
            RenderMeshVertex[] resultVertices = mesh.Vertices;
            Vector3[] positions = new Vector3[resultVertices.Length];
            Vector3[] normals = new Vector3[resultVertices.Length];
            Vector2[] uvs = new Vector2[resultVertices.Length];
            for (int index = 0; index < resultVertices.Length; ++index)
            {
                positions[index] = ToUnity(resultVertices[index].Position);
                normals[index] = ToUnity(resultVertices[index].Normal);
                uvs[index] = new Vector2(resultVertices[index].U, resultVertices[index].V);
            }

            uint[] resultIndices = mesh.Indices;
            int[] triangles = new int[resultIndices.Length];
            for (int index = 0; index < resultIndices.Length; ++index)
            {
                triangles[index] = checked((int)resultIndices[index]);
            }

            Mesh source = meshFilter.sharedMesh;
            Mesh preciseMesh = new Mesh
            {
                name = source.name + "_PreciseCut",
                indexFormat = resultVertices.Length > ushort.MaxValue
                    ? IndexFormat.UInt32
                    : IndexFormat.UInt16,
                vertices = positions,
                normals = normals,
                uv = uvs,
                triangles = triangles,
            };
            preciseMesh.RecalculateBounds();
            meshFilter.sharedMesh = preciseMesh;
            if (meshCollider != null)
            {
                meshCollider.sharedMesh = preciseMesh;
            }
        }

        private static ApxVec3 ToApx(Vector3 value)
        {
            return new ApxVec3(value.x, value.y, value.z);
        }

        private static Vector3 ToUnity(ApxVec3 value)
        {
            return new Vector3(value.X, value.Y, value.Z);
        }

    }
}
