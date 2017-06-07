/*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float4x4 NDCToOffset;
};

struct PSInput
{
	float4 Position : SV_Position;
	float3 Offset : Offfset;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	//This is drawn with depth testing enabled. Don't want to be far clipped. Note reversed depth.
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.0000001, 1);
	float4 unprojected = mul(output.Position, NDCToOffset);
	output.Offset = unprojected.xyz / unprojected.w;
	return output;
}

float3 PSMain(PSInput input) : SV_Target0
{
	return abs(normalize(input.Offset));
}