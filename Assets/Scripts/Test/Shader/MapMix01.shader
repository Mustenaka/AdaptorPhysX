Shader "Custom/MapMixShader01"
{
    Properties
    {
        // 下面三个是要叠加的贴图
        _MainTex ("Base (RGB), Alpha (A)", 2D) = "white" {} // 主贴图（也是底贴图）
        _MiddleTex ("Middle (RGB),Alpha (A)", 2D) = "white" {} // 中间贴图
        _TopTex ("Top (RGB),Alpha (A)", 2D) = "white" {} // 顶贴图
        _WhiteBoardTex ("WhiteBoard (RGB),Alpha (A)", 2D) = "white" {}//需要一个白色的贴图做底色

        // 模版比较函数
        _StencilComp ("Stencil Comparison", Float) = 8 //模板测试的比较函数。比较函数决定了当前像素的模板值和参考值之间的比较方式
        _Stencil ("Stencil ID", Float) = 0//当前对象的模板值。模板值是一个整数，用于标识当前对象的模板掩码。
        _StencilOp ("Stencil Operation", Float) = 0//模板测试通过或失败后执行的操作
        _StencilWriteMask ("Stencil Write Mask", Float) = 255//写入模板缓冲时的掩码
        _StencilReadMask ("Stencil Read Mask", Float) = 255//读取模板缓冲时的掩码

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

        Cull Off    // 关闭背面剔除
        Lighting Off    // 关闭光照效果
        ZWrite Off  // 关闭深度缓冲区写入
        ZTest [unity_GUIZTestMode]  // 设置深度测试，根据[unity_GUIZTestMode]
        Offset -1, -1   // 设置深度偏移，用于解决深度冲突的问题，即对深度缓冲区的值进行微小的偏移
        Fog // 雾效
        {
            Mode Off // 关闭雾效
        }
        Blend SrcAlpha OneMinusSrcAlpha // 设置颜色混合模式，使用源颜色的alpha值作为源因子，目标颜色的alpha值补值作为目标因子
        ColorMask [_ColorMask]  // 颜色掩码

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
                float4 vertex : POSITION; // 顶点位置
                float2 texcoord : TEXCOORD0; // 纹理坐标（uv）
                fixed4 color : COLOR; // 顶点颜色
            };

            struct v2f
            {
                float4 vertex : SV_POSITION; // 顶点的位置信息
                half2 texcoord : TEXCOORD0; // 顶点的纹理坐标信息（u,v）
                fixed4 color : COLOR; // 顶点颜色信息
            };

            sampler2D _MainTex;
            sampler2D _MiddleTex;
            sampler2D _TopTex;
            sampler2D _WhiteBoardTex;
            float4 _MainTex_ST;

            // 顶点着色器 Vertex Shader
            v2f vert(appdata_t v)
            {
                v2f o;
                // 顶点从对象空间（Object Space）转换到裁剪空间（Clip Space）
                o.vertex = UnityObjectToClipPos(v.vertex);
                // 
                o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);
                o.color = v.color;
                #ifdef UNITY_HALF_TEXEL_OFFSET
                    o.vertex.xy += (_ScreenParams.zw-1.0)*float2(-1,1);
                #endif
                return o;
            }

            // 片段着色器 Fragment Shader
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