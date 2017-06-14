/*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float2 HorizontalAxis;
	float2 ScreenToNDCScale;
	float2 InverseAtlasResolution;
};


struct GlyphInstance
{
	float2 TargetPosition; //In texels.
	float Scale;
	int SourceId;
};

struct GlyphSource
{
	float2 Minimum; //In texels, but offset such that 0,0 would be at UV 0,0. Texel corners, not centers.
	uint PackedSpan; //Lower 16 bits X, upper 16 bits Y. In texels.
	float DistanceScale;
};

StructuredBuffer<GlyphInstance> Instances : register(t0);
StructuredBuffer<GlyphSource> Sources : register(t1);

struct PSInput
{
	float4 Position : SV_Position;
	float2 AtlasUV : TextureCoordinates;
	float DistanceScale : DistanceScale; 
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	GlyphInstance instance = Instances[quadIndex];
	//Note that we find the source minimum/span by a second indirection.
	//The assumption here is that the glyph source data will be in the cache and hit over and over,
	//while the instances are just used once.
	//This could make sense if you're rendering a book or something, but for very low character counts,
	//this is just added overhead. For the purposes of the demo, eh, whatever, it'll be fine.
	GlyphSource source = Sources[instance.SourceId];
	float2 unpackedSpan = float2(source.PackedSpan & 0xFFFF, source.PackedSpan >> 16);
		
	PSInput output;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = instance.Scale * unpackedSpan * quadCoordinates;
	float2 verticalAxis = float2(-HorizontalAxis.y, HorizontalAxis.x);
	float2 screenPosition = instance.TargetPosition +
		localOffset.x * HorizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position. Note the negation along Y;
	//NDC +1 is up, while in screenspace/texture space +1 is down.
	output.Position = float4(
		screenPosition * ScreenToNDCScale + float2(-1.0, 1.0), 0.5, 1);
	output.AtlasUV = (source.Minimum + unpackedSpan * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = source.DistanceScale * instance.Scale;
	return output;
}

cbuffer PixelConstants : register(b0)
{
	float3 Color;
};
SamplerState Sampler : register(s0);
Texture2D<snorm float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	//The distances stored in the atlas are encoded such that:
	//1 atlas-loaded unit = max(glyphSource.Span.x, glyphSource.Span.y) texels.
	//The VS gives us a scaling factor of instance.Scale * max(glyphSource.Span.x, glyphSource.Span.y)
	//to get it into units of screen texels.
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a set radius.
	//If the glyph's distance is beyond the sample radius, then there is zero coverage.
	//If the distance is 0, then the sample is half covered.
	//If the distance is less than -sampleRadius, then it's fully covered.
	const float sampleRadius = 0.70710678118;
	float alpha = saturate(0.5 - screenDistance / (sampleRadius * 2));
	//return float4(1, 0, 0, 1);
	//return float4(screenDistance, screenDistance, screenDistance, 1);
	//return float4(saturate(-Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale) * 1, 0, 0, 1);
	//return float4((1 + Atlas.Sample(Sampler, input.AtlasUV)) * 1, 0, 0, 1);
	return float4(Color * alpha, alpha);
}