// Interior of cut tissue (built-in pipeline). Shades with 3D noise of the rest-space
// position stored in UV1, so cut faces need no UV unwrapping and stay stable while
// the tissue deforms.
Shader "AdaptorPhysX/Tissue Interior"
{
    Properties
    {
        _Color ("Base Color", Color) = (0.62, 0.12, 0.12, 1)
        _Color2 ("Fiber Color", Color) = (0.92, 0.45, 0.42, 1)
        _Scale ("Noise Scale (1/m)", Float) = 80
        _Glossiness ("Smoothness", Range(0, 1)) = 0.65
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" }
        CGPROGRAM
        #pragma surface surf Standard fullforwardshadows vertex:vert
        #pragma target 3.0

        fixed4 _Color;
        fixed4 _Color2;
        float _Scale;
        half _Glossiness;

        struct Input
        {
            float3 restPos;
        };

        void vert(inout appdata_full v, out Input o)
        {
            UNITY_INITIALIZE_OUTPUT(Input, o);
            o.restPos = v.texcoord1.xyz;
        }

        float hash(float3 p)
        {
            p = frac(p * 0.3183099 + 0.1);
            p *= 17.0;
            return frac(p.x * p.y * p.z * (p.x + p.y + p.z));
        }

        float noise(float3 x)
        {
            float3 i = floor(x);
            float3 f = frac(x);
            f = f * f * (3.0 - 2.0 * f);
            return lerp(lerp(lerp(hash(i + float3(0, 0, 0)), hash(i + float3(1, 0, 0)), f.x),
                             lerp(hash(i + float3(0, 1, 0)), hash(i + float3(1, 1, 0)), f.x), f.y),
                        lerp(lerp(hash(i + float3(0, 0, 1)), hash(i + float3(1, 0, 1)), f.x),
                             lerp(hash(i + float3(0, 1, 1)), hash(i + float3(1, 1, 1)), f.x), f.y), f.z);
        }

        void surf(Input IN, inout SurfaceOutputStandard o)
        {
            float3 p = IN.restPos * _Scale;
            float n = 0.55 * noise(p) + 0.3 * noise(p * 2.3) + 0.15 * noise(p * 5.1);
            o.Albedo = lerp(_Color.rgb, _Color2.rgb, smoothstep(0.35, 0.75, n));
            o.Smoothness = _Glossiness;
            o.Metallic = 0;
        }
        ENDCG
    }
    FallBack "Diffuse"
}
