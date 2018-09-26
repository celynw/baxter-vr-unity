Shader "Custom/VertexColour" {
    //gl_PointSize = psize;
    SubShader {
    Pass {
        LOD 100

        CGPROGRAM
        #pragma vertex vert
        #pragma fragment frag

        struct VertexInput {
            float4 v : POSITION;
            float4 color: COLOR;
        };

        struct VertexOutput {
            float4 pos : SV_POSITION;
            float4 col : COLOR;
            float size : PSIZE;
        };

        VertexOutput vert(VertexInput v) {
            VertexOutput o;
            o.pos = mul(UNITY_MATRIX_MVP, v.v);
            o.col = v.color;
            o.size = 2000.0f;

            return o;
        };

        float4 frag(VertexOutput o) : COLOR {
            return o.col;
        };

        ENDCG
        }
    }
}
