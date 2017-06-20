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
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 aabbCoordinates = float3((vertexId & 1) << 1, vertexId & 2, (vertexId & 4) >> 1) - 1;

	float3 offset = instance.Position - CameraPosition;
	//Note that in view space, -z moves away along the camera's forward axis by convention.
	float3 sphereViewPosition = float3(dot(CameraRight, offset), dot(CameraUp, offset), dot(CameraBackward, offset));

	float3 vertexViewPosition = sphereViewPosition + instance.Radius * aabbCoordinates;
	if (aabbCoordinates.z > 0)
	{
		//Clamp the near side of the AABB to the camera nearclip plane (unless the far side of the AABB passes the near plane).
		//This keeps the raytraced sphere visible even during camera overlap.
		//Note the consequences of -z being forward.
		vertexViewPosition.z = max(min(-NearClip - 1e-5, vertexViewPosition.z), sphereViewPosition.z - instance.Radius);
	}
	output.ToAABB = vertexViewPosition.x * CameraRight + vertexViewPosition.y * CameraUp + vertexViewPosition.z * CameraBackward;
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
bool RayCastSphere(float3 rayDirection, float3 spherePosition, float radius,
	out float t, out float3 hitLocation, out float3 hitNormal)
{
	float directionLength = length(rayDirection);
	float3 normalizedDirection = rayDirection / directionLength;
	float3 m = -spherePosition;
	float b = dot(m, normalizedDirection);
	float c = dot(m, m) - radius * radius;

	if (c > 0 && b > 0)
	{
		t = 0;
		hitLocation = 0;
		hitNormal = 0;
		return false;
	}
	float discriminant = b * b - c;
	if (discriminant < 0)
	{
		t = 0;
		hitLocation = 0;
		hitNormal = 0;
		return false;
	}

	t = -b - sqrt(discriminant);
	if (t < 0)
	{
		t = 0;
		hitLocation = 0;
		hitNormal = 0;
		return false;
	}
	t /= directionLength;
	hitLocation = normalizedDirection * t;
	hitNormal = normalize(hitLocation - spherePosition);
	return true;
}
PSOutput PSMain(PSInput input)
{
	PSOutput output;
	float t;
	float3 hitLocation, hitNormal;
	if (RayCastSphere(input.ToAABB, input.Sphere.Position, input.Sphere.Radius, t, hitLocation, hitNormal))
	{
		output.Color = normalize(abs(hitNormal));
		output.Depth = GetProjectedDepth(-hitLocation.z, Near, Far);
	}
	else
	{
		output.Color = 0;
		output.Depth = 0;
	}
	return output;
}