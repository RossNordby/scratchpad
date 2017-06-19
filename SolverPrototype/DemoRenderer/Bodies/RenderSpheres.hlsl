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
	float3 CameraForward;
};

struct SphereInstance
{
	nointerpolation float3 Position : SpherePosition;
	nointerpolation float Radius : SphereRadius;
	nointerpolation float4 Orientation : SphereOrientation;
};

StructuredBuffer<SphereInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToAABB : RayDirection;
	SphereInstance Sphere;
};
#define SampleRadius 0.70710678118
PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each AABB has 8 vertices; the position is based on the 3 least significant bits.
	int instanceId = vertexId >> 3;
	SphereInstance instance = Instances[instanceId];
	PSInput output;
	output.Sphere.Position = instance.Position - CameraPosition;
	output.Sphere.Radius = instance.Radius;
	output.Sphere.Orientation = instance.Orientation;

	//Convert the vertex id to local AABB coordinates, and then into view space.
	float3 aabbCoordinates = float3((vertexId & 1) << 1, vertexId & 2, (vertexId & 4) >> 1) - 1;

	float3 offset = instance.Position - CameraPosition;
	float3 sphereViewPosition = float3(dot(CameraRight, offset), dot(CameraUp, offset), dot(CameraForward, offset));

	float3 vertexViewPosition = sphereViewPosition + instance.Radius * aabbCoordinates;
	if (aabbCoordinates.z < 0)
	{
		//Clamp the near side of the AABB to the camera nearclip plane (unless the far side of the AABB passes the near plane).
		//This keeps the raytraced sphere visible even during camera overlap.
		vertexViewPosition.z = min(max(NearClip + 1e-5, vertexViewPosition.z), sphereViewPosition.z + instance.Radius);
	}
	output.ToAABB = vertexViewPosition.x * CameraRight + vertexViewPosition.y * CameraUp + vertexViewPosition.z * CameraForward;
	output.Position = mul(float4(vertexViewPosition, 1), Projection);
	return output;
}

struct PSOutput
{
	float3 Color : SV_Target0;
	float Depth : SV_DepthLessEqual;
};

cbuffer PixelConstants : register(b0)
{
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
	float sphereDistanceSquared = dot(input.ToAABB, input.ToAABB);
	float sphereDistance = sqrt(sphereDistanceSquared);
	float3 direction = input.ToAABB / sphereDistance;
	float3 m = -input.ToAABB;
	float c = sphereDistanceSquared - input.Sphere.Radius * input.Sphere.Radius;
	float discriminant = sphereDistanceSquared - c;

	//This isn't exactly an ideal GPU implementation by any means, but ehh we can worry about it if it is ever actually too slow.
	if (c > 0 || discriminant < 0)
	{
		output.Color = 0;
		output.Depth = 0;
	}
	else
	{
		float t = sphereDistance - sqrt(discriminant);
		if (t < 0)
		{
			output.Color = 0;
			output.Depth = 0;
		}
		else
		{
			float3 hitLocation = direction * t;
			float3 hitNormal = normalize(hitLocation - input.Sphere.Position);
			output.Color = abs(direction);
			output.Depth = GetProjectedDepth(hitLocation.z, Near, Far);
		}
	}
	return output;
}