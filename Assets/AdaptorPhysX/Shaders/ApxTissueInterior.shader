// Interior of cut tissue (built-in pipeline). UV1 holds the rest-space position, so the 3D
// noise needs no unwrapping and stays attached while the tissue deforms. UV0.x of cut faces is
// the depth below the original surface, which selects the layer colour: dermis, then fat, then
// muscle. With both layer depths at 0 the whole cut is muscle-coloured.
Shader "AdaptorPhysX/Tissue Interior"
{
    Properties
    {
        _Color ("Muscle Color", Color) = (0.62, 0.12, 0.12, 1)
        _Color2 ("Muscle Fiber Color", Color) = (0.92, 0.45, 0.42, 1)
        _SkinColor ("Dermis Color", Color) = (0.93, 0.78, 0.72, 1)
        _FatColor ("Fat Color", Color) = (0.97, 0.86, 0.52, 1)
        _SkinDepth ("Dermis Thickness (m)", Float) = 0
        _FatDepth ("Fat Thickness (m)", Float) = 0
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
        fixed4 _SkinColor;
        fixed4 _FatColor;
        float _SkinDepth;
        float _FatDepth;
        float _Scale;
        half _Glossiness;

        struct Input
        {
            float3 restPos;
            float depth;
        };

        void vert(inout appdata_full v, out Input o)
        {
            UNITY_INITIALIZE_OUTPUT(Input, o);
            o.restPos = v.texcoord1.xyz;
            o.depth = v.texcoord.x;
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
            float3 muscle = lerp(_Color.rgb, _Color2.rgb, smoothstep(0.35, 0.75, n));
            float3 fat = _FatColor.rgb * (0.85 + 0.3 * noise(p * 3.0));
            float3 dermis = _SkinColor.rgb * (0.92 + 0.16 * n);
            float blend = 0.0006;
            float toFat = smoothstep(_SkinDepth - blend, _SkinDepth + blend, IN.depth);
            float toMuscle = smoothstep(_SkinDepth + _FatDepth - blend, _SkinDepth + _FatDepth + blend, IN.depth);
            float3 albedo = lerp(lerp(dermis, fat, toFat), muscle, toMuscle);
            if (_SkinDepth + _FatDepth <= 0.0)
            {
                albedo = muscle;
            }
            o.Albedo = albedo;
            o.Smoothness = _Glossiness;
            o.Metallic = 0;
        }
        ENDCG
    }
    FallBack "Diffuse"
}
