using Unity.Mathematics;

namespace AdaptorPhysX
{
    public static class Geometry
    {
        /// <summary>Closest point on triangle abc (Ericson, Real-Time Collision Detection 5.1.5).</summary>
        public static float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 ab = b - a, ac = c - a, ap = p - a;
            float d1 = math.dot(ab, ap), d2 = math.dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) return a;
            float3 bp = p - b;
            float d3 = math.dot(ab, bp), d4 = math.dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) return b;
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) return a + ab * (d1 / (d1 - d3));
            float3 cp = p - c;
            float d5 = math.dot(ab, cp), d6 = math.dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) return c;
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) return a + ac * (d2 / (d2 - d6));
            float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
            {
                return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
            }
            float denom = 1.0f / (va + vb + vc);
            return a + ab * (vb * denom) + ac * (vc * denom);
        }

        /// <summary>Two-sided segment/triangle intersection; returns the segment parameter in [0,1] or -1.</summary>
        public static float SegmentTriangle(float3 p, float3 q, float3 a, float3 b, float3 c)
        {
            float3 dir = q - p;
            float3 e1 = b - a, e2 = c - a;
            float3 h = math.cross(dir, e2);
            float det = math.dot(e1, h);
            if (math.abs(det) < 1e-12f)
            {
                return -1.0f;
            }
            float inv = 1.0f / det;
            float3 s = p - a;
            float u = inv * math.dot(s, h);
            if (u < 0.0f || u > 1.0f) return -1.0f;
            float3 qv = math.cross(s, e1);
            float v = inv * math.dot(dir, qv);
            if (v < 0.0f || u + v > 1.0f) return -1.0f;
            float t = inv * math.dot(e2, qv);
            return t >= 0.0f && t <= 1.0f ? t : -1.0f;
        }

        /// <summary>Whether x, a point in the plane of triangle abc with normal n, lies inside the triangle or on its boundary.</summary>
        public static bool PointInTriangle(float3 x, float3 a, float3 b, float3 c, float3 n)
        {
            return math.dot(math.cross(b - a, x - a), n) >= 0.0f && math.dot(math.cross(c - b, x - b), n) >= 0.0f &&
                   math.dot(math.cross(a - c, x - c), n) >= 0.0f;
        }

        /// <summary>Trilinear weight of corner <paramref name="k"/> at local cell coordinate <paramref name="u"/>.</summary>
        public static float TrilinearWeight(int k, float3 u)
        {
            float3 w = math.select(1.0f - u, u, new bool3((k & 1) != 0, (k & 2) != 0, (k & 4) != 0));
            return w.x * w.y * w.z;
        }

        /// <summary>Gradient of the trilinear weight of corner <paramref name="k"/> with respect to <paramref name="u"/>.</summary>
        public static float3 TrilinearGradient(int k, float3 u)
        {
            bool3 hi = new bool3((k & 1) != 0, (k & 2) != 0, (k & 4) != 0);
            float3 w = math.select(1.0f - u, u, hi);
            float3 s = math.select(-1.0f, 1.0f, hi);
            return new float3(s.x * w.y * w.z, w.x * s.y * w.z, w.x * w.y * s.z);
        }
    }
}
