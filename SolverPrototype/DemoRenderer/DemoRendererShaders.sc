   Font\RenderGlyphs.hlsl.vshader      DXBCQтy`ћзчюf|H'|        4   є  (  А  x  RDEFИ     Р      <    ўџ    RD11<          (   $                                   І                        Ў                             Instances Sources VertexConstants ЋЋЎ                          №            І                                    џџџџ    џџџџ    М                 џџџџ    џџџџ    и                 џџџџ    џџџџ    HorizontalAxis float2 ЋЋ                              InverseHalfScreenResolution InverseAtlasResolution Ћ            ь      џџџџ    џџџџ    $Element GlyphInstance TargetPosition ЋЋ                              Scale float                              j  SourceId int ЋЋЋ                               /  @      d  p       Є             Ш                  !              l      џџџџ    џџџџ    GlyphSource Minimum Span ЋЋЋD  @      L  @             T                  8  Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_VertexId OSGN         P                    \                   o                   SV_Position TextureCoordinates DistanceScale ЋЋЋSHEXР  P  А   j Y  F         Ђ   p        Ђ   p       `          g  ђ         e  2     e  B     h     8  2                @    П  ?          B      
     @     V       *        	B      @     @     
     V  "     *      U  B      
     @     Ї    ђ     *      @      F~     Ї    ђ     :     @      F~    8  Т      І
    І    2  	Т     І            8  2     F     ц
     4  B     :      *      8  2     ц
    F         2  
Т                         2  	2      V    F      ц
     2  2      F      ц          @    П  П        6  Т      @             ?  ?>  STAT                	                                                                                                                              Font\RenderGlyphs.hlsl.pshader       DXBCС=бgтТ8Љю6ик         4       P  d  RDEFX     М      <    џџ  0  RD11<          (   $                                      Є            џџџџ          Њ                             Sampler Atlas PixelConstants ЋЋЋЊ      д              ќ                   џџџџ    џџџџ    Color float3 ЋЋЋ                              Microsoft (R) HLSL Shader Compiler 10.1 ISGN         P                    \                   o                   SV_Position TextureCoordinates DistanceScale ЋЋЋOSGN,                               SV_Target ЋЋSHEX  P   C   j Y  F         Z   `     X  p     UU  b 2    b B    e  ђ      h     E  Т  CU       F    F~      `     8        
      *    2  
      
 A       @  ѓЕ?@    ?8  r            F          6        
      >  STAT                                                                                                                                                *PostProcessing\CompressToSwap.hlsl.vshader    И  DXBC<СЋ^иF!#юuls!   И     4       д       RDEFd               <    ўџ  <   RD11<          (   $          Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_VertexId OSGN,                              SV_Position SHEX  P  C   j `          g  ђ         h     )        
     @             
      @     V  "      
        B      @     @     
     @      V        *         
2      F      @    П  П        6  Т      @             ?  ?>  STAT                                                                                                                                              *PostProcessing\CompressToSwap.hlsl.pshader    д  DXBCИ!z ЦйdхИйзХ9   д     4   h    а  8  RDEF,           <    џџ    RD11<          (   $          |            џџџџ       	                                Color Constants       Є              Ь             р       џџџџ    џџџџ    InverseGamma float Ћ                             й   Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_Position OSGN,                               SV_Target ЋЋSHEX`  P      j Y  F         X  p     UU  d  2        e  ђ      h     2  	      
     
          &   а        
      @  лэH.V        
      &   а        
      @  ПS9V        
      &   а        
      @   љV        
              
      @  џџ  V        
      2  	      
      @   7@     П  2     F     6  Т     @                  -  Т  CU т      F    6y     6  т      V     /  т      V     8  т      V                 т      V     2  r            @  ;;;         6        @    ?>  STAT                                                                                                                                             (Background\RenderBackground.hlsl.vshader    L  DXBCЫѕgCЧLу§Чѕla   L     4   H  |  а  А  RDEF     h      <    ўџ  ф   RD11<          (   $          \                             Constants ЋЋ\         @           Ј       @      Р       џџџџ    џџџџ    NDCToOffset float4x4 ЋЋЋ                            Д   Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_VertexId OSGNL         8                    D                   SV_Position Offset ЋSHEXи  P  v   j Y  F         `          g  ђ         e  r     h     )        
     @             
      @     V  "      
        B      @     @     
     @      V        *         
2      F      @    П  П        6  Т      @          Пж3  ?6  ђ      F            F     F            "     F     F           B     F     F                 F     F           r     F          >  STAT                                                                                                                                              (Background\RenderBackground.hlsl.pshader    `  DXBCЏzдМyФ9aЗжЁ`   `     4       є   (  Ф  RDEFd               <    џџ  <   RD11<          (   $          Microsoft (R) HLSL Shader Compiler 10.1 ISGNL         8                    D                   SV_Position Offset ЋOSGN,                              SV_Target ЋЋSHEX   P   %   j b r    e  r      h             F    F    D        
      8  r            F    6  r      F       >  STAT                                                                                                                                                  nt SourceId;
};

struct GlyphSource
{
	float2 Minimum; //In texels, but offset such that 0,0 would be at UV 0,0. Texel corners, not centers.
	float2 Span; //In texels.
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
	
	PSInput output;
	float2 scaledSpan = instance.Scale * source.Span;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = scaledSpan * quadCoordinates;
	float2 verticalAxis = float2(-HorizontalAxis.y, HorizontalAxis.x);
	float2 screenPosition = instance.TargetPosition +
		localOffset.x * HorizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position.
	//NDC = screenPosition * inverseResolution * 2 - 1
	//(1 / resolution) * 2 = 1 / (resolution * 0.5)
	output.Position = float4(
		screenPosition * InverseHalfScreenResolution - 1.0, 0.5, 1);
	output.AtlasUV = (source.Minimum + source.Span * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = max(scaledSpan.x, scaledSpan.y);
	return output;
}

cbuffer PixelConstants : register(b0)
{
	float3 Color;
};
SamplerState Sampler : register(s0);
Texture2D<float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	//The distances stored in the atlas are encoded such that:
	//1 atlas-loaded unit = max(glyphSource.Span.x, glyphSource.Span.y) texels.
	//The VS gives us a scaling factor of instance.Scale * max(glyphSource.Span.x, glyphSource.Span.y)
	//to get it into units of screen texels.
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a given width.
	//If the glyph is at a distance equal to the sample width or higher, the pixel is fully transparent.
	//At 0 distance, it becomes opaque. Intermediate distances imply partial coverage.
	const float sampleWidth = .707;
	float alpha = saturate(1 - screenDistance / sampleWidth);
	return float4(Color * alpha, alpha);
}                                                                                                                                                                                                                                                                                                                                                                                               ўяўя   l   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Font\RenderGlyphs.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\font\renderglyphs.hlsl /*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float2 HorizontalAxis;
	float2 InverseHalfScreenResolution;
	float2 InverseAtlasResolution;
};


struct GlyphInstance
{
	float2 TargetPosition; //In texels.
	float Scale;
	int SourceIdт0   нЩцув                                                               v   (   т0^FЎ;     u   v                                                                                                                                                                                                                                                                                                                                                                                                                  B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget vs_5_0 hlslEntry VSMain    .     $      `      `    |     VSMain   . >u    vertexId                               P     |    `    > >   <VSMain return value>                                  P    |    `    P    |    `    P    |    `    P     |    `     P    |    `    P    |    `    P    |    `   2 >t     quadIndex                                  P      Ќ    А     . >   instance                               P      и        P        h    P     0   ј     P     \   Ь     . >
   source                                 P         T     P     Д   ($    P     р   ќ(    P        а,   . >   output                                 P          м     P         м     P        Ш     P     (   Д     P        \     P        \     P        @     2 >   scaledSpan                                 P      (   t     P     (   Д   6 >   quadCoordinates                                P      Ь   0    P     р   ќ4   2 >   localOffset                                P      ќ        P     ќ       2 >   verticalAxis                               P         Ф@    P     0   ЌD   6 >   screenPosition                                 P      Є         P     Є          є         ДVnя;[а ЅЪkxЇ  ђ   Р        м      N   Д  |   (  |   (      (     (   Ќ   )  Ќ   )   и   )  и   )     )    )   0  )  0  )   \  /  \  /     /    /   Д  /  Д  /   р  /  р  /     2    2   (  3  (  3   <  3  <  3   X  3  X  3   l  3  l  3     3    3     3    3   И  3  И  3   Ь  3  Ь  3   р  4  р  4   ќ  5  ќ  5     5    5   0  6  0  7   P  6  P  6  l  6  l  7     6    6  Є  ;  Є  <   Ф  ;  Ф  <   ф  ;  ф  <      ;     ;    ;    ;  (  =  (  =   D  =  D  =   `  =  `  =     >    >     ?    ?   А  ?  А  ?   Ф  ?  Ф  ?   и  ?  и  ?            /  .  /  .  /  .  /  .  1  0  1  0  1  0  1  0  2  1  D " -  D " -  D 1 =  D 1 =  D 0 B  D 0 B  D  C  D  C  3  2  C  /  C 	 B  @     @     @ $ ?  @  ?  >  .  >  4  >  4  >  =  >  =  \ % A  \  A  \  [  8  7                 і                    <   l      И                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     Ъ18        и  
 џџ   џџ     L   L      T       
    u    @       float4 ѓђё @       float2 ѓђёB      Position ё    AtlasUV ђё @    DistanceScale                PSInput ђё
             B      TargetPosition ѓђё @    Scale  t    SourceId ё"               GlyphInstance &      Minimum ђё    Span ё"    	           GlyphSource ђё
     
            ђё
     
      
      ђё
   Ъ18              џџ   џџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	GlyphInstance instance = Instances[quadIndex];
	//Note that we find the source minimum/span by a second indirection.
	//The assumption here is that the glyph source data will be in the cache and hit over and over,
	//while the instances are just used once.
	//This could make sense if you're rendering a book or something, but for very low character counts,
	//this is just added overhead. For the purposes of the demo, eh, whatever, it'll be fine.
	GlyphSource source = Sources[instance.SourceId];
	
	PSInput output;
	float2 scaledSpan = instance.Scale * source.Span;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = scaledSpan * quadCoordinates;
	float2 verticalAxis = float2(-HorizontalAxis.y, HorizontalAxis.x);
	float2 screenPosition = instance.TargetPosition +
		localOffset.x * HorizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position.
	//NDC = screenPosition * inverseResolution * 2 - 1
	//(1 / resolution) * 2 = 1 / (resolution * 0.5)
	output.Position = float4(
		screenPosition * InverseHalfScreenResolution - 1.0, 0.5, 1);
	output.AtlasUV = (source.Minimum + source.Span * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = max(scaledSpan.x, scaledSpan.y);
	return output;
}

cbuffer PixelConstants : register(b0)
{
	float3 Color;
};
SamplerState Sampler : register(s0);
Texture2D<float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	//The distances stored in the atlas are encoded such that:
	//1 atlas-loaded unit = max(glyphSource.Span.x, glyphSource.Span.y) texels.
	//The VS gives us a scaling factor of instance.Scale * max(glyphSource.Span.x, glyphSource.Span.y)
	//to get it into units of screen texels.
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a given width.
	//If the glyph is at a distance equal to the sample width or higher, the pixel is fully transparent.
	//At 0 distance, it becomes opaque. Intermediate distances imply partial coverage.
	const float sampleWidth = .707;
	float alpha = saturate(1 - screenDistance / sampleWidth);
	return float4(Color * alpha, alpha);
}        u      v   ъ                                                                                                                  D3DSHDR м                             `                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        џџџџ	/ё0     m                  Й            =                                                                                                                                                                                                                                                                                                                                    @                                                                                                                                                                                                           $   0   <                                                                                                                                                                                                                                                                                                                                                                                                                                        %        VSMain    " Q       џџџџџџHorizontalAxis  . Q      џџџџџџInverseHalfScreenResolution * Q      џџџџџџInverseAtlasResolution   Q   џџџџ  џџџџInstances    Q   џџџџ џџџџSources                                                                                                                                                                                                                                                                                                                                        џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y     ічПBЈЃх9}Ё   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\font\renderglyphs.hlsl          :             
             "          мQ3                                                                                                                                                                                                                                                                   џџџџw	1    
 Д  L       ,                                       м     `             	 (      є                 VSMain none -К.ё       м     `                    џџџџ    м        џџџџ    џџџџ         C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Font\RenderGlyphs.hlsl ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                                                                §       8              8
  T       (   \  ,   д      $         %                                 	   
                                           !   #   "                                                                                                                                                                                                                                                                                                                           &                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               Font\RenderGlyphs.hlsl.pshader    R  DXBC'&зЈ|МFЄ&   R     8        T  №    RDEFX     М      <    џџA  0  RD11<          (   $                                      Є            џџџџ          Њ                             Sampler Atlas PixelConstants ЋЋЋЊ      д              ќ                   џџџџ    џџџџ    Color float3 ЋЋЋ                              Microsoft (R) HLSL Shader Compiler 10.1 ISGN         P                    \                   o                   SV_Position TextureCoordinates DistanceScale ЋЋЋOSGN,                               SV_Target ЋЋSHEX  P   e   j Y  F         Z   `     X  p     UU  b 2    b B    e  ђ      h     E  Т  CU       F    F~      `     8        
      *    6  "      @  є§4?+  B      @             
            6        
 A                
      *      4        
      @      3        
      @    ?8  r            F          6        
      >  STAT                                                                                                                                               SPDB N  Microsoft C/C++ MSF 7.00
DS         '   И       #                                                                                                                                                                                                                                                                                                                                                                                                                                                                           Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ8   №џџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ       <       џџџџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         .1/н=Y   АuєLBрCБVёІуяK                          мQ3                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ;
};

struct GlyphSource
{
	float2 Minimum; //In texels, but offset such that 0,0 would be at UV 0,0. Texel corners, not centers.
	float2 Span; //In texels.
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
	//The vertex id is used to position each ЦZ  u 8  Э   &ї A$ 'R ц Lш }
 }Е Л С! AЙ ; 1 }к                                                                                                                                                                                                                                                                                                                                                                                                                                                            /*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float2 HorizontalAxis;
	float2 InverseHalfScreenResolution;
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
	float2 Span; //In texels.
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
	
	PSInput output;
	float2 scaledSpan = instance.Scale * source.Span;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = scaledSpan * quadCoordinates;
	float2 verticalAxis = float2(-HorizontalAxis.y, HorizontalAxis.x);
	float2 screenPosition = instance.TargetPosition +
		localOffset.x * HorizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position.
	//NDC = screenPosition * inverseResolution * 2 - 1
	//(1 / resolution) * 2 = 1 / (resolution * 0.5)
	output.Position = float4(
		screenPosition * InverseHalfScreenResolution - 1.0, 0.5, 1);
	output.AtlasUV = (source.Minimum + source.Span * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = max(scaledSpan.x, scaledSpan.y);
	return output;
}

cbuffer PixelConstants : register(b0)
{
	float3 Color;
};
SamplerState Sampler : register(s0);
Texture2D<float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	//The distances stored in the atlas are encoded such that:
	//1 atlas-loaded unit = max(glyphSource.Span.x, glyphSource.Span.y) texels.
	//The VS gives us a scaling factor of instance.Scale * max(glyphSource.Span.x, glyphSource.Span.y)
	//to get it into units of screen texels.
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a given width.
	//If the glyph is at a distance equal to the sample width or higher, the pixel is fully transparent.
	//At 0 distance, it becomes opaque. Intermediate distances imply partial coverage.
	const float sampleWidth = .707;
	float alpha = saturate(1 - screenDistance / sampleWidth);
	return float4(Color * alpha, alpha);
}                                                                                                                                                                                                                                                                                                                                                                                               ўяўя   l   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Font\RenderGlyphs.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\font\renderglyphs.hlsl /*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float2 HorizontalAxis;
	float2 InverseHalfScreenResolution;
	float2 InverseAtlasResolution;
};


struct GlyphInstance
{
	float2 TargetPosition; //In texels.
	float Scale;
	int SourceIdт0   6цув                                                               v   (   т0^FЎ;     u   v                                                                                                                                                                                                                                                                                                                                                                                                                  B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget ps_5_0 hlslEntry PSMain    .           0      0    d     PSMain   . >  	 input                                  P     d    0     P    d    0    P    d    0    P    d    0    P    d    0    P    d    0    P    d    0   > >   <PSMain return value>                                  P     d    0     P    d    0    P    d    0    P    d    0   6 >@     screenDistance                                 P      Ќ    D     2 >    sampleWidth                                P      Р    д    . >@     alpha                                  P      \   8       є         ДVnя;[а ЅЪkxЇ  ђ   8                 ,  d   O  d   O      O     O   Ќ   S  Ќ   S   Р   T  Р   T   д   T  д   T   №   T  №   T     T    T   $  T  $  T   @  T  @  T   \  U  \  U   |  U  |  U     U    U    S  <  S  R       :  8  :  8  :  8  :  8  :  9  :  9  %    %  %  %  % і                    4   P                                                                                                                                                                                                                                                                                                                                                                               Ъ18        H  
 џџ   џџ     D   D      L        @       float4 ѓђё @       float2 ѓђёB       Position ё    AtlasUV ђё @    DistanceScale                PSInput ђё
      
             
 @      @       float3 ѓђё
     
 	         
   ђё
     
      @      ђё
     
                                                                                                                                     Ъ18              џџ   џџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	GlyphInstance instance = Instances[quadIndex];
	//Note that we find the source minimum/span by a second indirection.
	//The assumption here is that the glyph source data will be in the cache and hit over and over,
	//while the instances are just used once.
	//This could make sense if you're rendering a book or something, but for very low character counts,
	//this is just added overhead. For the purposes of the demo, eh, whatever, it'll be fine.
	GlyphSource source = Sources[instance.SourceId];
	
	PSInput output;
	float2 scaledSpan = instance.Scale * source.Span;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = scaledSpan * quadCoordinates;
	float2 verticalAxis = float2(-HorizontalAxis.y, HorizontalAxis.x);
	float2 screenPosition = instance.TargetPosition +
		localOffset.x * HorizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position.
	//NDC = screenPosition * inverseResolution * 2 - 1
	//(1 / resolution) * 2 = 1 / (resolution * 0.5)
	output.Position = float4(
		screenPosition * InverseHalfScreenResolution - 1.0, 0.5, 1);
	output.AtlasUV = (source.Minimum + source.Span * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = max(scaledSpan.x, scaledSpan.y);
	return output;
}

cbuffer PixelConstants : register(b0)
{
	float3 Color;
};
SamplerState Sampler : register(s0);
Texture2D<float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	//The distances stored in the atlas are encoded such that:
	//1 atlas-loaded unit = max(glyphSource.Span.x, glyphSource.Span.y) texels.
	//The VS gives us a scaling factor of instance.Scale * max(glyphSource.Span.x, glyphSource.Span.y)
	//to get it into units of screen texels.
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a given width.
	//If the glyph is at a distance equal to the sample width or higher, the pixel is fully transparent.
	//At 0 distance, it becomes opaque. Intermediate distances imply partial coverage.
	const float sampleWidth = .707;
	float alpha = saturate(1 - screenDistance / sampleWidth);
	return float4(Color * alpha, alpha);
}        u      v   ъ                                                                                                                  D3DSHDR                              `                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        џџџџ	/ё            Q      5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       $                                                                                                                                                                                                                                                                                                                                                                                                                                                                %        PSMain     Q
       џџџџџџColor    Q   џџџџџџ  џџSampler  Q   џџџџ  џџџџAtlas                                                                                                                                                                                                                                                                                                                                                                                                                                                  џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y   АuєLBрCБVёІуяKЁ   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\font\renderglyphs.hlsl          :             
             "          мQ3                                                                                                                                                                                                                                                                   џџџџw	1    
 Д  L       ,                                            `             	       l   q            PSMain none -К.ё            `                    џџџџ            џџџџ    џџџџ         C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Font\RenderGlyphs.hlsl ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                                                                §       8                L       (   D  ,   l             !                                 	   
                                                                                                                                                                                                                                                                                                                                                                                  "                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               *PostProcessing\CompressToSwap.hlsl.vshader    A  DXBChЭHuК`)Ф-I   A     8   Є   и     №    RDEFd               <    ўџA  <   RD11<          (   $          Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_VertexId OSGN,                              SV_Position SHEXм  P  w   j `          g  ђ         h     :  6        
     6  "      @     )  "      
            6  B      @       "      *            6  B      @     )        
      *      6  B      @             *      
      V             V  "     
      +  2      @                6  2      F A          2      F      F     +        @     6  2      F      6  B      @     ?6  ђ      F     >  STAT                                                                                                                                              SPDB >  Microsoft C/C++ MSF 7.00
DS            Ј                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ8  џџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ       <       џџџџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         .1/н=Y   zuтзэGК
ЅNО                          мQ3                                                                                                                                                                                                                                                                                                                                                                                                                                                                    	float4 Position : SV_Position;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
{
	const float ditherWidth = 1.0 / 255.0;
	//Compute dither amount from screen position. There are more principled ways of doing this, but shrug.
	float dither = input.Position.x * input.Position.x + input.Position.y;
	dither = asuint(dither) * 776531419;
	dzП  ЦZ  ;I Э   џ о# u 1ћ c                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            /*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

Texture2D<float3> Color : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
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
	return float4(adjustedColor, 1);
}                                                                                                                                                                                                                                                                                                                                                                                        float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}                                                                                                                                                                                                                                                                                                                                                                                                      ўяўя   9   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\PostProcessing\CompressToSwap.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\postprocessing\compresstoswap.hlsl WholeScreenTriangle.hlsl wholescreentriangle.hlsl /*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

Texture2D<float3> Color : register(t0);

struct PSInput
{
т0Ќ   6цув                                                                  (   т0aз                         (   т0gЂў{                                                                                                                                                                                                                                                                                                                                                                             B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget vs_5_0 hlslEntry VSMain    .     D      Ј      Ј    4     VSMain   . >u    vertexId                               P     4    Ј    > >   <VSMain return value>                                  P     4    Ј     P    4    Ј    P    4    Ј    P    4    Ј   . >   output                                 P        @     P      А   ,      P     А   ,     P     Ф       B M   @      	=< 	L 	"0	&20	%70		80<T( V >   <GetWholeScreenTriangleVertexNDC return value>                                 P               P            . >u    vertexId                               P      L          N  є   0      M пoor5@
љ3    Ў^t,1аНnшт_  ђ   ј        м      (   ь  4     4      8     8      L     L      `     `      |     |                 Ќ     Ќ      Р     Р      м     м      №     №                          4    4     T    T     l    l                       А    А     Ф    Ф     и    и      M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  C  M  L  M  L  M  L         і                                                                                                                         Ъ18      	  Ќ    џџ   џџ     $   $      ,       
    u    @       float4 ѓђё      Position ё               PSInput ђё
              @       float2 ѓђё
                                                                                                                                                                                                                                                                                                         g                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ither = asuint(dither) * 961748927;
	dither = asuint(dither) * 217645199;
	dither = (asuint(dither) & 65535) / 65535.0;

	//Note saturate to get rid of warning. Down the road, if you do goofy stuff like HDR, you could insert the tonemapping operator in here.
	float3 adjustedColor = pow(saturate(Color[input.Position.xy]), InverseGamma);

	adjustedColor = saturate(adjustedColor + ditherWidth * (dither - 0.5));
	return float4(adjustedColor, 1);
} float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}           4                      Н                                                                                                                                                                                                                                                                                                                                                                                                               Ъ18        ,    џџ   џџ                     *       GetWholeScreenTriangleVertexNDC                                                                                                                                                                                                                                                                                                                                                                                                                             D3DSHDR м                             `             *       GetWholeScreenTriangleVertexNDC                                                                                                                                                                                                                                                                                                                                                                                                                             џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   %        VSMain        џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y   zuтзэGК
ЅNОб   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\postprocessing\compresstoswap.hlsl /src/files/wholescreentriangle.hlsl    
      .                   "      
      ­   	       мQ3                                                                                                                                                                                                           џџџџw	1    
 Д  L       ,   Ќ                                    м     `             
 H      T   q            VSMain none -К.ё       м     `                    џџџџ    м        џџџџ    џџџџ            C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\PostProcessing\CompressToSwap.hlsl WholeScreenTriangle.hlsl    ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                    5  ф   Г  d       y  Ќ     {   Є  ,      (      ,                                       	   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           *PostProcessing\CompressToSwap.hlsl.pshader    (D  DXBC_]фJl.Г
Ћ^Ж9Ўђ   (D     8   l     д       RDEF,           <    џџA    RD11<          (   $          |            џџџџ       	                                Color Constants       Є              Ь             р       џџџџ    џџџџ    InverseGamma float Ћ                             й   Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_Position OSGN,                               SV_Target ЋЋSHEXЈ  P   ъ   j Y  F         X  p     UU  d  2        e  ђ      h     6        @  ;8  "      
     
        "                 6  B      @  лэH.&   а  "      *            V  "            6  B      @  ПS9&   а  "      *            V  "            6  B      @   љ&   а  "      *            V  "            6  B      @  џџ    "      *            V  "              "            @   џG  2     F     6  Т     @                  -  Т  CU r     F    F~     4  
r     F    @                  3  
r     F    @    ?  ?  ?    /  r     F    8  r     F                r     F    6  B      @     П   "      *            8              
         r            F    4  
r      F     @                  3  
r      F     @    ?  ?  ?    +        @     6  r      F     >  STAT   !                                                                                                                                          SPDB >  Microsoft C/C++ MSF 7.00
DS            Ј                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ8  џџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ       <       џџџџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         .1/н=Y   ryчcўHБmлл[о4                          мQ3                                                                                                                                                                                                                                                                                                                                                                                                                                                                    	float4 Position : SV_Position;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
{
	const float ditherWidth = 1.0 / 255.0;
	//Compute dither amount from screen position. There are more principled ways of doing this, but shrug.
	float dither = input.Position.x * input.Position.x + input.Position.y;
	dither = asuint(dither) * 776531419;
	dЦZ  Dб Э   Т A$ a ц Lш 	ё [9 Ю7  9Ю                                                                                                                                                                                                                                                                                                                                                                                                                                                                                /*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

Texture2D<float3> Color : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
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
	return float4(adjustedColor, 1);
}                                                                                                                                                                                                                                                                                                                                                                                        float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}                                                                                                                                                                                                                                                                                                                                                                                                      ўяўя   9   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\PostProcessing\CompressToSwap.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\postprocessing\compresstoswap.hlsl WholeScreenTriangle.hlsl wholescreentriangle.hlsl /*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

Texture2D<float3> Color : register(t0);

struct PSInput
{
т0Ќ   нЩцув                                                                  (   т0aз                         (   т0gЂў{                                                                                                                                                                                                                                                                                                                                                                             B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget ps_5_0 hlslEntry PSMain    .     P      X      X    P     PSMain   . >  	 input                                  P     P    X     P    P    X    P    P    X    P    P    X   > >   <PSMain return value>                                  P     P    X     P    P    X    P    P    X    P    P    X   2 >    ditherWidth                                P      d    Ќ    . >@     dither                                & P          X4  |  Ф  0    6 >   adjustedColor                                  P      Ф   И     P     Ф   И     P     Ф   И     P      |   ,      P     |   ,     P     |   ,      є   0      M пoor5@
љ3    Ў^t,1аНnшт_  ђ   0        Ј      B   $  P     P      d     d                            А     А      а     а      ф     ф      ј     ј               ,     ,      @     @      `     `      t  !  t  !     !    !   Є  !  Є  !   И  !  И  !   д  $  д  $   ш  $  ш  $     $    $   ,  $  ,  $   T  $  T  $   |  $  |  $     $    $   А  $  А  $   Ф  &  Ф  &   и  &  и  &   є  &  є  &     &    &   ,  &  ,  &   T  &  T  &   |  '  |  '     '    '   Є  '  Є  '    '  &  G  3  G  F  %  $  %  $  %  $  %  $  %  $  %  $  %  $  %  $  %  $  -  !  -  !  -  ,  -  ,  N & =  N & =  N & =  N  >  N  >  N  M  N  M  N  M  H : E  H : E  H + F  H  F  H  G  H  G  ! 	    !  !  !  ! і                    <                                                                                                                                                                                                                                                                                                   Ъ18        а    џџ   џџ     0   0      8        @       float4 ѓђё       Position ё               PSInput ђё
      
             
 @      @       float3 ѓђё
            ђё
 	    
 
                                                                                                                                                                                                                                                            Ъ18              џџ   џџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 ither = asuint(dither) * 961748927;
	dither = asuint(dither) * 217645199;
	dither = (asuint(dither) & 65535) / 65535.0;

	//Note saturate to get rid of warning. Down the road, if you do goofy stuff like HDR, you could insert the tonemapping operator in here.
	float3 adjustedColor = pow(saturate(Color[input.Position.xy]), InverseGamma);

	adjustedColor = saturate(adjustedColor + ditherWidth * (dither - 0.5));
	return float4(adjustedColor, 1);
} float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}           4                      Н                                                                                                                                                                                                                                                                                                                                                                                                               D3DSHDR Ј                             `                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        џџџџ	/ё                 =                                                                                                                                                                                                                                                                                                                                                                                                                             @                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              %        PSMain    " Q       џџџџџџInverseGamma     Q   џџџџ  џџџџColor                                                                                                                                                                                                                                                                                                                                                                                                                                                                      џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y   ryчcўHБmлл[о4б   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\postprocessing\compresstoswap.hlsl /src/files/wholescreentriangle.hlsl    
      .                   "      
      ­   	       мQ3                                                                                                                                                                                                           џџџџw	1    
 Д  L       ,   Ќ                                    Ј     `             
 T      |   W            PSMain none -К.ё       Ј     `                    џџџџ    Ј        џџџџ    џџџџ            C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\PostProcessing\CompressToSwap.hlsl WholeScreenTriangle.hlsl    ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                    5    Г  8       y  Ќ     {   р  8       (   8  ,   X                                    	   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (Background\RenderBackground.hlsl.vshader    (C  DXBCЈr3CCшz*гДЂ   (C     8   L    д       RDEF     h      <    ўџA  ф   RD11<          (   $          \                             Constants ЋЋ\         @           Ј       @      Р       џџџџ    џџџџ    NDCToOffset float4x4 ЋЋЋ                            Д   Microsoft (R) HLSL Shader Compiler 10.1 ISGN,                             SV_VertexId OSGNL         8                    D                   SV_Position Offset ЋSHEXЈ  P  Њ   j Y  F         `          g  ђ         e  r     h     :  6        
     6  "      @     )  "      
            6  B      @       "      *            6  B      @     )        
      *      6  B      @             *      
      V             V  "     
      +  2      @                6  2      F A          2      F      F     +        @     6  2      F      6  B      @  Пж3       F     F            "     F     F           B     F     F                F     F           r     F    і    6  ђ      F     6  r     F    >  STAT                                                                        	                                                                      SPDB >  Microsoft C/C++ MSF 7.00
DS            Є                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ8  Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ       <       џџџџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         .1/н=Y   ъ?шЕBіЌxsдхЁ                          мQ3                                                                                                                                                                                                                                                                                                                                                                                                                                                                    et : Offset;
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
} float2 GetWholeScreenTrianzП  ЦZ  Lш є­  Э   н С u й* р g I Н                                                                                                                                                                                                                                                                                                                                                                                                                                                                            /*[META]
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
	float3 Offset : Offset;
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
}                                                                                                                                                                                                                                                                                                                                                       float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}                                                                                                                                                                                                                                                                                                                                                                                                      ўяўя   V   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Background\RenderBackground.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\background\renderbackground.hlsl WholeScreenTriangle.hlsl wholescreentriangle.hlsl /*[META]
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
	float3 Offsт0Ќ   і№цув                                                        	          (   т0.:1Љ                         (   т0gЂў{   ў                                                                                                                                                                                                                                                                                                                                                                           B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget vs_5_0 hlslEntry VSMain    .     h      X      X    P     VSMain   . >u    vertexId                               P     P    X    > >   <VSMain return value>                                  P    P    X    P    P    X    P    P    X    P     P    X     P    P    X    P    P    X    P    P    X   . >   output                                 P     И   №     P      Ь   м      P     Ь   м     P     р   Ш     P     |   ,     P     |   ,     P     |   ,    2 >   unprojected                                P          |     P         \     P     @   <     P     `   H    B M   d      	=< 	h 	"0	&20	%70		80<T( V >   <GetWholeScreenTriangleVertexNDC return value>                                 P      Є         P     Є       . >u    vertexId                               P      h          N  є   0      x1ШkT#ђ$iO#єю  ў   Ў^t,1аНnшт_  ђ           Ј      4   |  P     P      T     T      h     h      |     |                 Ќ     Ќ      Ш     Ш      м     м      ј     ј               (    (     <    <     P    P     p    p              Є    Є     И    И     Ь    Ь     р    р                           @    @     `    `     |    |              Є    Є      S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  C  S  R  S  R  S  R  8  7  8  7  8  7  8  7  1  0             і                                                                                                                                                                                                    Ъ18           џџ   џџ     4   4      <       
    u    @       float4 ѓђё @       float3 ѓђё*      Position ё    Offset ѓђё               PSInput ђё
              @       float2 ѓђё
              @             @ float4x4 
 
    
                                                                                                                                                                                              №                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            gleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}           к                  ў     0                                                                                                                                                                                                                                                                                                                                                                                  Ъ18        ,    џџ   џџ                     *     	  GetWholeScreenTriangleVertexNDC                                                                                                                                                                                                                                                                                                                                                                                                                             D3DSHDR Ј                             `             *     	  GetWholeScreenTriangleVertexNDC                                                                                                                                                                                                                                                                                                                                                                                                                             џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               %        VSMain     Q       џџџџџџNDCToOffset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y   ъ?шЕBіЌxsдхЁЯ   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\background\renderbackground.hlsl /src/files/wholescreentriangle.hlsl    
      6                   "      
      Ћ   	       мQ3                                                                                                                                                                                                             џџџџw	1    
 Д  L       ,   Ј                                    Ј     `             
 l      ф                 VSMain none -К.ё       Ј     `                    џџџџ    Ј        џџџџ    џџџџ         ~   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Background\RenderBackground.hlsl WholeScreenTriangle.hlsl  ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                        3  H  Џ  d         Ќ   Љ  {   \  <      (   ,  ,   8                                 	   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (Background\RenderBackground.hlsl.pshader    8  DXBCQEpЇ)<ЛЎ   8     8   Є   ј   ,  ф    RDEFd               <    џџA  <   RD11<          (   $          Microsoft (R) HLSL Shader Compiler 10.1 ISGNL         8                    D                   SV_Position Offset ЋOSGN,                              SV_Target ЋЋSHEXА   P   ,   j b r    e  r      h             F    F    D        
      8  r            F    6  r     FA       4  r      F     F    >  STAT                                                                                                                                                  SPDB 6  Microsoft C/C++ MSF 7.00
DS                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              Рџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ8  јџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџџ       <       џџџџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         .1/н=Y   0.љC5DЙ#fџFЙhЎ                          мQ3                                                                                                                                                                                                                                                                                                                                                                                                                                                                    et : Offset;
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
} float2 GetWholeScreenTrianЦZ  Lш 3  Э   &ї щ№ 'R                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    /*[META]
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
	float3 Offset : Offset;
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
}                                                                                                                                                                                                                                                                                                                                                       float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}                                                                                                                                                                                                                                                                                                                                                                                                      ўяўя   V   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Background\RenderBackground.hlsl  c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\background\renderbackground.hlsl WholeScreenTriangle.hlsl wholescreentriangle.hlsl /*[META]
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
	float3 Offsт0Ќ   n-цув                                                        	          (   т0.:1Љ                         (   т0gЂў{   ў                                                                                                                                                                                                                                                                                                                                                                           B <   
  Lз:
  Lз:Microsoft (R) HLSL Shader Compiler 10.1   : =hlslFlags 0x4015 hlslTarget ps_5_0 hlslEntry PSMain    .                       ,     PSMain   . >  	 input                                  P     ,          P    ,         P    ,         P    ,         P    ,         P    ,         P    ,        > >   <PSMain return value>                                  P     ,          P    ,         P    ,          є   0      x1ШkT#ђ$iO#єю  ў   Ў^t,1аНnшт_  ђ   Ј         А             ,     ,      H     H      \     \      x     x                 Ќ     Ќ       %  #  %  #  %  #  % 	 $  % 	 $  %  % і                                                                                                                                                                                                                                                             Ъ18        Є    џџ   џџ              $        @       float4 ѓђё @       float3 ѓђё*       Position ё    Offset ѓђё               PSInput ђё
      
                                                                                                                                                                                                                                                                                                                Ъ18              џџ   џџ                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 gleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
}           к                  ў     0                                                                                                                                                                                                                                                                                                                                                                                  D3DSHDR А                              `                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   %        PSMain        џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       џџџџ	/ё                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            .1/н=Y   0.љC5DЙ#fџFЙhЎЯ   /LinkInfo /names /src/headerblock /src/files/c:\users\norbo\documents\visual studio 2017\projects\scratchpad\solverprototype\demorenderer\background\renderbackground.hlsl /src/files/wholescreentriangle.hlsl    
      6                   "      
      Ћ   	       мQ3                                                                                                                                                                                                             џџџџw	1    
 Д  L       ,   Ј                                    А      `             
       є                  PSMain none -К.ё       А      `                    џџџџ    А         џџџџ    џџџџ         ~   C:\Users\Norbo\Documents\Visual Studio 2017\Projects\scratchpad\SolverPrototype\DemoRenderer\Background\RenderBackground.hlsl WholeScreenTriangle.hlsl  ўяўя                  џџџџџџџџџџ џџџџџџџџџџ                                                                                        3  м   Џ  8         Ќ   Љ  {     $       (      ,                                    	   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          