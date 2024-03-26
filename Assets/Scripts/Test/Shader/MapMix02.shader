Shader "Custom/MapMixShader02"
{
    Properties
    {
        _MainTex1 ("Texture 1 (Albedo)", 2D) = "white" {}
        _MainTex2 ("Texture 2 (Albedo)", 2D) = "white" {}
        _NormalMap1 ("Normal Map 1", 2D) = "bump" {}
        _NormalMap2 ("Normal Map 2", 2D) = "bump" {}
    }
    SubShader
    {
        Tags
        {
            "RenderType"="Opaque"
        }
        Blend SrcAlpha OneMinusSrcAlpha
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata_t
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float3 normal : NORMAL;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float3 normal : TEXCOORD1;
                UNITY_FOG_COORDS(1)
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex1;
            sampler2D _MainTex2;
            sampler2D _NormalMap1;
            sampler2D _NormalMap2;

            v2f vert(appdata_t v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.normal = UnityObjectToWorldNormal(v.normal); // 使用世界空间法线
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                fixed4 tex1 = tex2D(_MainTex1, i.uv);
                fixed4 tex2 = tex2D(_MainTex2, i.uv);
                fixed3 normal1 = UnpackNormal(tex2D(_NormalMap1, i.uv));
                fixed3 normal2 = UnpackNormal(tex2D(_NormalMap2, i.uv));

                // 计算正片叠底混合后的法线
                fixed3 normal = normalize(normal1 + normal2);

                // 计算最终颜色
                fixed4 result = tex1 * tex2 * dot(normal, normalize(i.normal));
                return result;
            }
            ENDCG
        }
    }
}