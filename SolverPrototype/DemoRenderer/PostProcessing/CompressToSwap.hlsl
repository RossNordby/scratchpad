/*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

SamplerState ColorSampler : register(s0);
Texture2D<float3> Color : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float2 TextureCoordinates : TEXCOORD;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	float2 ndc = GetWholeScreenTriangleVertexNDC(vertexId);
	output.Position = float4(ndc, 0.5, 1);
	output.TextureCoordinates = ndc * 0.5 + 0.5;
	return output;
}

float3 PSMain(PSInput input) : SV_Target0
{

	const float ditherWidth = 1.0 / 255.0;
	//Compute dither amount from screen position. There are more principled ways of doing this, but shrug.
	float dither = input.Position.x * input.Position.x + input.Position.y;
	dither = asuint(dither) * 776531419;
	dither = asuint(dither) * 961748927;
	dither = asuint(dither) * 217645199;
	dither = (asuint(dither) & 65535) / 65535.0;

	//Note saturate to get rid of warning. Down the road, if you do goofy stuff like HDR, you could insert the tonemapping operator in here.
	float3 adjustedColor = pow(saturate(Color[input.Position.xy]), InverseGamma);

	adjustedColor = saturate(adjustedColor + ditherWidth * (dither - 0.5));
	return adjustedColor;
}