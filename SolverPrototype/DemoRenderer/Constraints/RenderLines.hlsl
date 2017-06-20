/*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float4x4 Projection;
	float3 CameraPosition;
	float NearClip;
	float3 CameraRight;
	float Padding0;
	float3 CameraUp;
	float Padding1;
	float3 CameraBackward;
};

#include "ColorUnpacking.hlsl"
struct LineInstance
{
	float3 Start;
	float Radius;
	float3 End;
	uint PackedColor;
};

StructuredBuffer<LineInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
};
#define SampleRadius 0.70710678118
PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each AABB has 8 vertices; the position is based on the 3 least significant bits.
	int instanceId = vertexId >> 2;
	LineInstance instance = Instances[instanceId];
	PSInput output;

	//Project the start and end points into NDC.
	//Build the quad vertices in screenspace, padded by 1 pixel on each side. 
	float2 quadCoordinates = float2((vertexId & 1) << 1, vertexId & 2) - 1;
	//Output the line start, direction, and length in screenspace for use by the pixel shader. This is similar to the UI line renderer.

	//1) More accurate depths could be calculated for the vertices rather than simply assuming that they have the same depth as the endpoint.
	//That assumption overestimates the depth of the vertices associated with the near point and underestimates the depth of the far point.
	//Only bother if actually matters.
	//2) The Z coordinate could be unprojected to get linear depth, which the pixel shader could then interpolate and offset with conservative depth output.
	//That would allow smoother transitions between overlap.
	output.Position = 0;
	return output;
}

struct PSOutput
{
	float3 Color : SV_Target0;
	float Depth : SV_DepthLessEqual;
};

cbuffer PixelConstants : register(b0)
{
	float3 CameraForward;
	float Padding;
	float Near;
	float Far;
};


float GetProjectedDepth(float linearDepth, float near, float far)
{
	//Note the reversal of near and far relative to a standard depth projection.
	//We use 0 to mean furthest, and 1 to mean closest.
	float dn = linearDepth * near;
	return (far * near - dn) / (linearDepth * far - dn);
}


PSOutput PSMain(PSInput input)
{
	PSOutput output;

	return output;
}