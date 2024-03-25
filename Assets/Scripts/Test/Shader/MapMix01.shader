Shader "Custom/MapMixShader01"
{
    Properties
    {
        //下面三个是要叠加的贴图
        _MainTex ("Base (RGB), Alpha (A)", 2D) = "white" {}
        _MiddleTex ("Middle (RGB),Alpha (A)", 2D) = "white" {}
        _TopTex ("Top (RGB),Alpha (A)", 2D) = "white" {}
        _WhiteBoardTex ("WhiteBoard (RGB),Alpha (A)", 2D) = "white" {}//需要一个白色的贴图做底色

        _StencilComp ("Stencil Comparison", Float) = 8
        _Stencil ("Stencil ID", Float) = 0
        _StencilOp ("Stencil Operation", Float) = 0
        _StencilWriteMask ("Stencil Write Mask", Float) = 255
        _StencilReadMask ("Stencil Read Mask", Float) = 255

        _ColorMask ("Color Mask", Float) = 15
        // 以 1 - _Height 长度为高度
        _Height ("Radius", Range(0,0.5)) = 0.5

        _TopRate ("TopRate", Range(0,1)) = 0.33 //显示裁剪的比例
        _MiddleRate ("MiddleRate", Range(0,1)) = 0.33
        _ButtomRate ("ButtomRate", Range(0,1)) = 0.33
    }

    SubShader
    {
        LOD 100

        Tags
        {
            "Queue" = "Transparent"
            "IgnoreProjector" = "True"
            "RenderType" = "Transparent"
            "PreviewType"="Plane"
        }

        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }

        Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Offset -1, -1
        Fog
        {
            Mode Off
        }
        Blend SrcAlpha OneMinusSrcAlpha
        ColorMask [_ColorMask]



        Pass
        {
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float _Height;
            float _TopRate;
            float _MiddleRate;
            float _ButtomRate;

            struct appdata_t
            {
                float4 vertex : POSITION;
                float2 texcoord : TEXCOORD0;
                fixed4 color : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                half2 texcoord : TEXCOORD0;
                fixed4 color : COLOR;
            };

            sampler2D _MainTex;
            sampler2D _MiddleTex;
            sampler2D _TopTex;
            sampler2D _WhiteBoardTex;
            float4 _MainTex_ST;

            v2f vert(appdata_t v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);
                o.color = v.color;
                #ifdef UNITY_HALF_TEXEL_OFFSET
                    o.vertex.xy += (_ScreenParams.zw-1.0)*float2(-1,1);
                #endif
                return o;
            }

            fixed4 frag(v2f i) : COLOR
            {
                fixed4 col2 = tex2D(_MiddleTex, i.texcoord); //获得贴图的rgba,颜色像素信息
                fixed4 base_col = tex2D(_MainTex, i.texcoord);
                fixed4 top_pic = tex2D(_TopTex, i.texcoord);
                fixed4 white_canva = tex2D(_WhiteBoardTex, i.texcoord);

                float2 uv = i.texcoord.xy; //输入的面的uv坐标
                float4 c = i.color; //输入的面的颜色信息
                float4 c1 = i.color;
                float4 c3 = i.color;
                float4 c_alpha_mask = i.color;

                fixed4 col4 = col2;

                if (uv.y < _TopRate && uv.y > 0.0)
                {
                    c.rgba = col2.rgba; //根据不同的uv坐标显示不同贴图的颜色信息
                }
                if (uv.y > _TopRate + _MiddleRate && uv.y < 1.0)
                {
                    c1.rgba = base_col.rgba;
                }

                if (uv.y > _TopRate && uv.y < _TopRate + _MiddleRate)
                {
                    c3.rgba = top_pic.rgba;
                }

                fixed4 col_a = white_canva * c * c1 * c3; //将颜色信息与白色底图叠加
                return col_a; //最后输出叠加后的颜色信息，贴图就叠加完成
            }
            ENDCG
        }
    }

}