using System;
using System.Collections.Generic;

namespace APEX.Native
{
    public struct RenderMeshVertex
    {
        public ApxVec3 Position;
        public ApxVec3 Normal;
        public float U;
        public float V;

        public RenderMeshVertex(ApxVec3 position, ApxVec3 normal, float u, float v)
        {
            Position = position;
            Normal = normal;
            U = u;
            V = v;
        }
    }

    public sealed class RenderMeshData
    {
        public RenderMeshData(RenderMeshVertex[] vertices, uint[] indices)
        {
            Vertices = vertices ?? throw new ArgumentNullException(nameof(vertices));
            Indices = indices ?? throw new ArgumentNullException(nameof(indices));
        }

        public RenderMeshVertex[] Vertices { get; }

        public uint[] Indices { get; }
    }

    public struct RenderMeshSeamPair
    {
        public uint NegativeVertexId;
        public uint PositiveVertexId;

        public RenderMeshSeamPair(uint negativeVertexId, uint positiveVertexId)
        {
            NegativeVertexId = negativeVertexId;
            PositiveVertexId = positiveVertexId;
        }
    }

    public sealed class RenderMeshCutResult
    {
        internal RenderMeshCutResult(
            RenderMeshData mesh,
            RenderMeshSeamPair[] seamPairs,
            uint cutTriangleCount)
        {
            Mesh = mesh;
            SeamPairs = seamPairs;
            CutTriangleCount = cutTriangleCount;
        }

        public RenderMeshData Mesh { get; }

        public RenderMeshSeamPair[] SeamPairs { get; }

        public uint CutTriangleCount { get; }

        public uint CreatedSeamPairCount => checked((uint)SeamPairs.Length);
    }

    /// <summary>
    /// Deterministically splits render triangles along finite swept segments.
    /// Existing vertex IDs stay fixed; each crossed edge appends one negative
    /// and one positive seam vertex in stable triangle/edge encounter order.
    /// </summary>
    public static class RenderMeshCutter
    {
        private struct CanonicalQuery
        {
            public ApxVec3 Start;
            public ApxVec3 End;
            public ApxVec3 SideNormal;
            public float RadiusSquared;
        }

        private struct EdgeIntersection
        {
            public uint VertexA;
            public uint VertexB;

            public EdgeIntersection(uint vertexA, uint vertexB)
            {
                VertexA = vertexA;
                VertexB = vertexB;
            }
        }

        public static RenderMeshCutResult Cut(RenderMeshData input, ApxCutQuery query)
        {
            ValidateMesh(input);
            CanonicalQuery canonical = Canonicalize(query);

            int vertexHeadroom = Math.Max(16, input.Vertices.Length / 32);
            int indexHeadroom = Math.Max(24, input.Indices.Length / 32);
            List<RenderMeshVertex> vertices =
                new List<RenderMeshVertex>(
                    checked(input.Vertices.Length + vertexHeadroom));
            vertices.AddRange(input.Vertices);
            List<uint> indices =
                new List<uint>(checked(input.Indices.Length + indexHeadroom));
            Dictionary<ulong, RenderMeshSeamPair> seamByEdge =
                new Dictionary<ulong, RenderMeshSeamPair>();
            List<RenderMeshSeamPair> seamPairs = new List<RenderMeshSeamPair>();
            uint cutTriangleCount = 0;

            for (int triangleOffset = 0; triangleOffset < input.Indices.Length; triangleOffset += 3)
            {
                uint id0 = input.Indices[triangleOffset];
                uint id1 = input.Indices[triangleOffset + 1];
                uint id2 = input.Indices[triangleOffset + 2];
                float distance0 = SignedSide(input.Vertices[id0].Position, canonical);
                float distance1 = SignedSide(input.Vertices[id1].Position, canonical);
                float distance2 = SignedSide(input.Vertices[id2].Position, canonical);
                bool hasPositive = distance0 > 0.0F || distance1 > 0.0F || distance2 > 0.0F;
                bool hasNegative = distance0 < 0.0F || distance1 < 0.0F || distance2 < 0.0F;
                if (!hasPositive || !hasNegative ||
                    !FiniteSegmentHitsTrianglePlaneIntersection(
                        input,
                        id0,
                        id1,
                        id2,
                        distance0,
                        distance1,
                        distance2,
                        canonical))
                {
                    indices.Add(id0);
                    indices.Add(id1);
                    indices.Add(id2);
                    continue;
                }

                uint[] triangleIds = { id0, id1, id2 };
                float[] distances = { distance0, distance1, distance2 };
                AppendClippedPolygon(
                    input,
                    triangleIds,
                    distances,
                    false,
                    canonical,
                    vertices,
                    indices,
                    seamByEdge,
                    seamPairs);
                AppendClippedPolygon(
                    input,
                    triangleIds,
                    distances,
                    true,
                    canonical,
                    vertices,
                    indices,
                    seamByEdge,
                    seamPairs);
                ++cutTriangleCount;
            }

            RenderMeshVertex[] outputVertices = vertices.ToArray();
            uint[] outputIndices = indices.ToArray();
            if (cutTriangleCount != 0U)
            {
                RebuildNormals(outputVertices, outputIndices);
            }

            return new RenderMeshCutResult(
                new RenderMeshData(outputVertices, outputIndices),
                seamPairs.ToArray(),
                cutTriangleCount);
        }

        public static RenderMeshCutResult CutPolyline(RenderMeshData input, ApxCutQuery[] segments)
        {
            ValidateMesh(input);
            if (segments == null)
            {
                throw new ArgumentNullException(nameof(segments));
            }

            RenderMeshData current = CloneMesh(input);
            List<RenderMeshSeamPair> allSeams = new List<RenderMeshSeamPair>();
            uint totalCutTriangles = 0;
            for (int index = 0; index < segments.Length; ++index)
            {
                RenderMeshCutResult segmentResult = Cut(current, segments[index]);
                current = segmentResult.Mesh;
                allSeams.AddRange(segmentResult.SeamPairs);
                totalCutTriangles = checked(totalCutTriangles + segmentResult.CutTriangleCount);
            }

            return new RenderMeshCutResult(current, allSeams.ToArray(), totalCutTriangles);
        }

        public static bool TryIntersectSegmentPlane(
            ApxVec3 segmentStart,
            ApxVec3 segmentEnd,
            ApxVec3 planePoint,
            ApxVec3 planeNormal,
            out float parameter,
            out ApxVec3 point)
        {
            parameter = 0.0F;
            point = default;
            if (!IsFinite(segmentStart) ||
                !IsFinite(segmentEnd) ||
                !IsFinite(planePoint) ||
                !IsFinite(planeNormal))
            {
                return false;
            }

            ApxVec3 direction = Subtract(segmentEnd, segmentStart);
            float denominator = Dot(planeNormal, direction);
            float normalLengthSquared = Dot(planeNormal, planeNormal);
            if (!IsFinite(denominator) ||
                !IsFinite(normalLengthSquared) ||
                normalLengthSquared <= 0.0F ||
                denominator == 0.0F)
            {
                return false;
            }

            float candidate = Dot(planeNormal, Subtract(planePoint, segmentStart)) / denominator;
            if (!IsFinite(candidate) || candidate < 0.0F || candidate > 1.0F)
            {
                return false;
            }

            ApxVec3 candidatePoint = AddScaled(segmentStart, direction, candidate);
            if (!IsFinite(candidatePoint))
            {
                return false;
            }

            parameter = candidate;
            point = candidatePoint;
            return true;
        }

        private static void ValidateMesh(RenderMeshData input)
        {
            if (input == null)
            {
                throw new ArgumentNullException(nameof(input));
            }
            if (input.Indices.Length % 3 != 0)
            {
                throw new ArgumentException("Render mesh indices must contain complete triangles.", nameof(input));
            }

            for (int index = 0; index < input.Vertices.Length; ++index)
            {
                RenderMeshVertex vertex = input.Vertices[index];
                if (!IsFinite(vertex.Position) ||
                    !IsFinite(vertex.Normal) ||
                    !IsFinite(vertex.U) ||
                    !IsFinite(vertex.V))
                {
                    throw new ArgumentException("Render mesh vertices must be finite.", nameof(input));
                }
            }
            bool[] referenced = new bool[input.Vertices.Length];
            for (int index = 0; index < input.Indices.Length; ++index)
            {
                if (input.Indices[index] >= input.Vertices.Length)
                {
                    throw new ArgumentException("Render mesh index is out of bounds.", nameof(input));
                }
                referenced[input.Indices[index]] = true;
            }

            for (int triangleOffset = 0;
                 triangleOffset < input.Indices.Length;
                 triangleOffset += 3)
            {
                ApxVec3 first = input.Vertices[input.Indices[triangleOffset]].Position;
                ApxVec3 second = input.Vertices[input.Indices[triangleOffset + 1]].Position;
                ApxVec3 third = input.Vertices[input.Indices[triangleOffset + 2]].Position;
                ApxVec3 twiceArea = Cross(Subtract(second, first), Subtract(third, first));
                float areaSquared = Dot(twiceArea, twiceArea);
                if (!IsFinite(areaSquared) || areaSquared <= 0.0F)
                {
                    throw new ArgumentException(
                        "Render mesh triangles must have finite non-zero area.",
                        nameof(input));
                }
            }

            for (int vertexId = 0; vertexId < referenced.Length; ++vertexId)
            {
                if (!referenced[vertexId])
                {
                    throw new ArgumentException(
                        "Render mesh must not contain isolated vertices.",
                        nameof(input));
                }
            }
        }

        private static CanonicalQuery Canonicalize(ApxCutQuery query)
        {
            if (!IsFinite(query.Start) ||
                !IsFinite(query.End) ||
                !IsFinite(query.SideNormal) ||
                !IsFinite(query.Radius) ||
                query.Radius < 0.0F)
            {
                throw new ArgumentException("Cut query fields must be finite and radius non-negative.", nameof(query));
            }

            ApxVec3 segment = Subtract(query.End, query.Start);
            float segmentLengthSquared = Dot(segment, segment);
            float normalLengthSquared = Dot(query.SideNormal, query.SideNormal);
            float radiusSquared = query.Radius * query.Radius;
            if (!IsFinite(segmentLengthSquared) ||
                segmentLengthSquared <= 0.0F ||
                !IsFinite(normalLengthSquared) ||
                normalLengthSquared <= 0.0F ||
                !IsFinite(radiusSquared))
            {
                throw new ArgumentException("Cut segment and side normal must be non-degenerate.", nameof(query));
            }

            float inverseNormalLength = 1.0F / (float)Math.Sqrt(normalLengthSquared);
            ApxVec3 sideNormal = Scale(query.SideNormal, inverseNormalLength);
            if (!IsFinite(sideNormal))
            {
                throw new ArgumentException("Cut side normal cannot be normalized.", nameof(query));
            }

            return new CanonicalQuery
            {
                Start = query.Start,
                End = query.End,
                SideNormal = sideNormal,
                RadiusSquared = radiusSquared,
            };
        }

        private static RenderMeshData CloneMesh(RenderMeshData input)
        {
            RenderMeshVertex[] vertices = new RenderMeshVertex[input.Vertices.Length];
            uint[] indices = new uint[input.Indices.Length];
            Array.Copy(input.Vertices, vertices, vertices.Length);
            Array.Copy(input.Indices, indices, indices.Length);
            return new RenderMeshData(vertices, indices);
        }

        private static bool FiniteSegmentHitsTrianglePlaneIntersection(
            RenderMeshData input,
            uint id0,
            uint id1,
            uint id2,
            float distance0,
            float distance1,
            float distance2,
            CanonicalQuery query)
        {
            uint[] ids = { id0, id1, id2 };
            float[] distances = { distance0, distance1, distance2 };
            ApxVec3[] intersections = new ApxVec3[2];
            int intersectionCount = 0;
            for (int edge = 0; edge < 3; ++edge)
            {
                int next = (edge + 1) % 3;
                if (!StrictlyCrosses(distances[edge], distances[next]))
                {
                    continue;
                }

                float parameter = distances[edge] / (distances[edge] - distances[next]);
                ApxVec3 start = input.Vertices[ids[edge]].Position;
                ApxVec3 end = input.Vertices[ids[next]].Position;
                intersections[intersectionCount++] =
                    AddScaled(start, Subtract(end, start), parameter);
            }

            return intersectionCount == 2 &&
                   SegmentDistanceSquared(
                       intersections[0],
                       intersections[1],
                       query.Start,
                       query.End) <= query.RadiusSquared;
        }

        private static void AppendClippedPolygon(
            RenderMeshData input,
            uint[] triangleIds,
            float[] distances,
            bool positiveSide,
            CanonicalQuery query,
            List<RenderMeshVertex> vertices,
            List<uint> indices,
            Dictionary<ulong, RenderMeshSeamPair> seamByEdge,
            List<RenderMeshSeamPair> seamPairs)
        {
            List<uint> polygon = new List<uint>(4);
            for (int edge = 0; edge < 3; ++edge)
            {
                int next = (edge + 1) % 3;
                bool currentInside = positiveSide ? distances[edge] > 0.0F : distances[edge] <= 0.0F;
                if (currentInside)
                {
                    polygon.Add(triangleIds[edge]);
                }
                if (!StrictlyCrosses(distances[edge], distances[next]))
                {
                    continue;
                }

                RenderMeshSeamPair seam = GetOrCreateSeam(
                    input,
                    new EdgeIntersection(triangleIds[edge], triangleIds[next]),
                    query,
                    vertices,
                    seamByEdge,
                    seamPairs);
                polygon.Add(positiveSide ? seam.PositiveVertexId : seam.NegativeVertexId);
            }

            for (int vertex = 1; vertex + 1 < polygon.Count; ++vertex)
            {
                indices.Add(polygon[0]);
                indices.Add(polygon[vertex]);
                indices.Add(polygon[vertex + 1]);
            }
        }

        private static RenderMeshSeamPair GetOrCreateSeam(
            RenderMeshData input,
            EdgeIntersection edge,
            CanonicalQuery query,
            List<RenderMeshVertex> vertices,
            Dictionary<ulong, RenderMeshSeamPair> seamByEdge,
            List<RenderMeshSeamPair> seamPairs)
        {
            uint minimum = Math.Min(edge.VertexA, edge.VertexB);
            uint maximum = Math.Max(edge.VertexA, edge.VertexB);
            ulong key = ((ulong)minimum << 32) | maximum;
            if (seamByEdge.TryGetValue(key, out RenderMeshSeamPair existing))
            {
                return existing;
            }

            RenderMeshVertex start = input.Vertices[minimum];
            RenderMeshVertex end = input.Vertices[maximum];
            float startDistance = SignedSide(start.Position, query);
            float endDistance = SignedSide(end.Position, query);
            float parameter = startDistance / (startDistance - endDistance);
            RenderMeshVertex seamVertex = Interpolate(start, end, parameter);
            uint negativeId = checked((uint)vertices.Count);
            vertices.Add(seamVertex);
            uint positiveId = checked((uint)vertices.Count);
            vertices.Add(seamVertex);
            RenderMeshSeamPair created = new RenderMeshSeamPair(negativeId, positiveId);
            seamByEdge.Add(key, created);
            seamPairs.Add(created);
            return created;
        }

        private static RenderMeshVertex Interpolate(
            RenderMeshVertex start,
            RenderMeshVertex end,
            float parameter)
        {
            ApxVec3 normal = AddScaled(start.Normal, Subtract(end.Normal, start.Normal), parameter);
            float normalLengthSquared = Dot(normal, normal);
            if (normalLengthSquared > 0.0F)
            {
                normal = Scale(normal, 1.0F / (float)Math.Sqrt(normalLengthSquared));
            }

            return new RenderMeshVertex(
                AddScaled(start.Position, Subtract(end.Position, start.Position), parameter),
                normal,
                start.U + (end.U - start.U) * parameter,
                start.V + (end.V - start.V) * parameter);
        }

        private static void RebuildNormals(
            RenderMeshVertex[] vertices,
            uint[] indices)
        {
            ApxVec3[] accumulated = new ApxVec3[vertices.Length];
            for (int triangleOffset = 0; triangleOffset < indices.Length; triangleOffset += 3)
            {
                uint id0 = indices[triangleOffset];
                uint id1 = indices[triangleOffset + 1];
                uint id2 = indices[triangleOffset + 2];
                ApxVec3 first = vertices[id0].Position;
                ApxVec3 second = vertices[id1].Position;
                ApxVec3 third = vertices[id2].Position;
                ApxVec3 faceNormal =
                    Cross(Subtract(second, first), Subtract(third, first));

                // Fixed triangle/index order makes this area-weighted reduction
                // reproducible. Seam copies accumulate independently by vertex ID.
                accumulated[id0] = Add(accumulated[id0], faceNormal);
                accumulated[id1] = Add(accumulated[id1], faceNormal);
                accumulated[id2] = Add(accumulated[id2], faceNormal);
            }

            for (int vertexId = 0; vertexId < vertices.Length; ++vertexId)
            {
                ApxVec3 normal = accumulated[vertexId];
                float lengthSquared = Dot(normal, normal);
                if (lengthSquared > 0.0F && IsFinite(lengthSquared))
                {
                    normal = Scale(normal, 1.0F / (float)Math.Sqrt(lengthSquared));
                }
                else
                {
                    // A zero-area accumulation is a deterministic finite fallback.
                    normal = default;
                }

                RenderMeshVertex vertex = vertices[vertexId];
                vertex.Normal = normal;
                vertices[vertexId] = vertex;
            }
        }

        private static bool StrictlyCrosses(float first, float second)
        {
            return (first < 0.0F && second > 0.0F) ||
                   (first > 0.0F && second < 0.0F);
        }

        private static float SignedSide(ApxVec3 position, CanonicalQuery query)
        {
            return Dot(Subtract(position, query.Start), query.SideNormal);
        }

        private static float SegmentDistanceSquared(
            ApxVec3 firstStart,
            ApxVec3 firstEnd,
            ApxVec3 secondStart,
            ApxVec3 secondEnd)
        {
            ApxVec3 first = Subtract(firstEnd, firstStart);
            ApxVec3 second = Subtract(secondEnd, secondStart);
            ApxVec3 offset = Subtract(firstStart, secondStart);
            float firstLengthSquared = Dot(first, first);
            float secondLengthSquared = Dot(second, second);
            float firstSecond = Dot(first, second);
            float firstOffset = Dot(first, offset);
            float secondOffset = Dot(second, offset);
            if (firstLengthSquared <= 0.0F)
            {
                return PointSegmentDistanceSquared(firstStart, secondStart, secondEnd);
            }

            float denominator =
                firstLengthSquared * secondLengthSquared - firstSecond * firstSecond;
            float firstParameter = 0.0F;
            if (denominator > 0.0F)
            {
                firstParameter = ClampUnit(
                    (firstSecond * secondOffset - firstOffset * secondLengthSquared) /
                    denominator);
            }
            float secondParameter = ClampUnit(
                (firstSecond * firstParameter + secondOffset) / secondLengthSquared);
            firstParameter = ClampUnit(
                (firstSecond * secondParameter - firstOffset) / firstLengthSquared);

            ApxVec3 firstClosest = AddScaled(firstStart, first, firstParameter);
            ApxVec3 secondClosest = AddScaled(secondStart, second, secondParameter);
            ApxVec3 delta = Subtract(firstClosest, secondClosest);
            return Dot(delta, delta);
        }

        private static float PointSegmentDistanceSquared(
            ApxVec3 point,
            ApxVec3 start,
            ApxVec3 end)
        {
            ApxVec3 segment = Subtract(end, start);
            float parameter =
                ClampUnit(Dot(Subtract(point, start), segment) / Dot(segment, segment));
            ApxVec3 delta = Subtract(point, AddScaled(start, segment, parameter));
            return Dot(delta, delta);
        }

        private static ApxVec3 Subtract(ApxVec3 first, ApxVec3 second)
        {
            return new ApxVec3(
                first.X - second.X,
                first.Y - second.Y,
                first.Z - second.Z);
        }

        private static ApxVec3 AddScaled(ApxVec3 value, ApxVec3 direction, float scale)
        {
            return new ApxVec3(
                value.X + direction.X * scale,
                value.Y + direction.Y * scale,
                value.Z + direction.Z * scale);
        }

        private static ApxVec3 Add(ApxVec3 first, ApxVec3 second)
        {
            return new ApxVec3(
                first.X + second.X,
                first.Y + second.Y,
                first.Z + second.Z);
        }

        private static ApxVec3 Scale(ApxVec3 value, float scale)
        {
            return new ApxVec3(value.X * scale, value.Y * scale, value.Z * scale);
        }

        private static ApxVec3 Cross(ApxVec3 first, ApxVec3 second)
        {
            return new ApxVec3(
                first.Y * second.Z - first.Z * second.Y,
                first.Z * second.X - first.X * second.Z,
                first.X * second.Y - first.Y * second.X);
        }

        private static float Dot(ApxVec3 first, ApxVec3 second)
        {
            return (first.X * second.X + first.Y * second.Y) + first.Z * second.Z;
        }

        private static float ClampUnit(float value)
        {
            return Math.Max(0.0F, Math.Min(1.0F, value));
        }

        private static bool IsFinite(ApxVec3 value)
        {
            return IsFinite(value.X) && IsFinite(value.Y) && IsFinite(value.Z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
