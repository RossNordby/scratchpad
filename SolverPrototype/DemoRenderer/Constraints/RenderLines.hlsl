/*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float4x4 ViewProjection;
	float2 NDCToScreenScale; //0 to 2 => 0 to resolution
	float2 Padding0;
	float3 CameraForward;
	float TanAnglePerPixel;
	float3 CameraRight;
	float Padding1;
	float3 CameraPosition;
};

#include "ColorUnpacking.hlsl"
struct LineInstance
{
	float3 Start;
	float Padding;
	float3 End;
	uint PackedColor;
};

StructuredBuffer<LineInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	nointerpolation float2 Start : ScreenLineStart;
	nointerpolation float2 LineDirection : ScreenLineDirection;
	nointerpolation float Length : ScreenLineLength;
	nointerpolation float3 Color : ScreenLineColor;
};
#define InnerRadius 1
#define OuterRadius 1.5
#define SampleRadius 0.70710678118
PSInput VSMain(uint vertexId : SV_VertexId)
{
	int instanceId = vertexId >> 2;
	LineInstance instance = Instances[instanceId];
	PSInput output;

	//Project the start and end points into NDC.
	//Output the line start, direction, and length in screenspace for use by the pixel shader. This is similar to the UI line renderer.
	float4 start = mul(float4(instance.Start, 1), ViewProjection);
	float4 end = mul(float4(instance.End, 1), ViewProjection);
	//Keep the z and w components unmodified. They'll be brought back to compute the vertex position.
	start.xy /= start.w;
	end.xy /= end.w;
	//Now in NDC. Scale x and y to screenspace.
	output.Start = float2(
		start.x * NDCToScreenScale.x + NDCToScreenScale.x,
		NDCToScreenScale.y - start.y * NDCToScreenScale.y);
	float2 screenEnd = float2(
		end.x * NDCToScreenScale.x + NDCToScreenScale.x,
		NDCToScreenScale.y - end.y * NDCToScreenScale.y);
	output.LineDirection = screenEnd - output.Start;
	output.Length = length(output.LineDirection);
	output.LineDirection = output.Length > 1e-5 ? output.LineDirection / output.Length : float2(1, 0);
	output.Color = UnpackR11G11B10_UNorm(instance.PackedColor);

	//Build the quad vertices in screenspace, padded by 1 pixel on each side. 
	float2 quadCoordinates = float2(vertexId & 1, (vertexId & 2) >> 1);
	float3 worldLineX = instance.End - instance.Start;
	worldLineX -= CameraForward * dot(CameraForward, worldLineX);
	float worldLineXLength = length(worldLineX);
	worldLineX = worldLineXLength > 1e-5 ? worldLineX / worldLineXLength : CameraRight;
	//How wide is a pixel at this vertex, approximately?
	float3 endpoint = quadCoordinates.x > 0 ? instance.End : instance.Start;
	//Note that distance is used instead of z. Resizing lines based on camera orientation is a bit odd.
	float distance = length(endpoint - CameraPosition);
	float pixelSize = distance * TanAnglePerPixel;
	//Use the pixel size in world space to pad the quad.
	const float paddingInPixels = OuterRadius + SampleRadius;
	//Points down in screenspace by convention to match UI line stuff.
	float3 worldLineY = cross(CameraForward, worldLineX);

	float worldPadding = paddingInPixels * pixelSize;
	float3 horizontalPadding = worldPadding * worldLineX;
	float3 position = quadCoordinates.x > 0 ? 
		instance.End + horizontalPadding :
		instance.Start - horizontalPadding;
	position += (quadCoordinates.y * 2 - 1) * (worldPadding * worldLineY);
	output.Position = mul(float4(position, 1), ViewProjection);
	return output;
	//A couple of notes:
	//1) More accurate depths could be calculated for the vertices rather than simply assuming that they have the same depth as the endpoint.
	//That assumption overestimates the depth of the vertices associated with the near point and underestimates the depth of the far point.
	//Only bother if actually matters.
	//2) The pixel shader could interpolate linear depth and offset it with conservative depth output.
	//That would allow smoother transitions between lines during overlap.
}

struct PSOutput
{
	float3 Color : SV_Target0;
};



PSOutput PSMain(PSInput input)
{
	PSOutput output;
	//Measure the distance from the line in screen pixels. First, compute the closest point.
	float2 perp = float2(-input.LineDirection.y, input.LineDirection.x);
	float2 offset = input.Position.xy - input.Start;
	float alongLine = clamp(dot(offset, input.LineDirection), 0, input.Length);
	float lineDistance = length(offset - alongLine * input.LineDirection);

	float innerDistance = lineDistance - InnerRadius;
	float outerDistance = lineDistance - OuterRadius;
	//This distance is measured in screen pixels. Treat every pixel as having a set radius.
	//If the line's distance is beyond the sample radius, then there is zero coverage.
	//If the distance is 0, then the sample is half covered.
	//If the distance is less than -sampleRadius, then it's fully covered.
	float innerColorScale = saturate(0.5 - innerDistance / (SampleRadius * 2));
	float outerColorScale = saturate(0.5 - outerDistance / (SampleRadius * 2));
	//TODO: For now, don't bother using a falloff on the outer radius.
	output.Color = input.Color * innerColorScale + 1 - innerColorScale;
	return output;
}