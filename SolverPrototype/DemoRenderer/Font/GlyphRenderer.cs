using BEPUutilities2;
using BEPUutilities2.Memory;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Diagnostics;
using System.Numerics;

namespace DemoRenderer.Font
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single character glyph instance.
    /// </summary>
    public struct GlyphInstance
    {
        //Don't really need full precision on the glyphs or their descriptions, but it's easy.
        /// <summary>
        /// Desired location of the minimum corner of the glyph.
        /// </summary>
        public Vector2 TargetPosition;
        /// <summary>
        /// Scale to apply to the source glyph.
        /// </summary>
        public float Scale;
        /// <summary>
        /// Index of the source character description in the description set.
        /// </summary>
        public int SourceId;
    }

    public class GlyphRenderer : IDisposable
    {
        struct VertexConstants
        {
            public Vector2 HorizontalAxis;
            public Vector2 ScreenToNDCScale;
            public Vector2 InverseAtlasResolution;
        }
        ConstantsBuffer<VertexConstants> vertexConstants;
        ConstantsBuffer<Vector3> pixelConstants;

        StructuredBuffer<GlyphInstance> instances;
        IndexBuffer indices;

        SamplerState sampler;
        VertexShader vertexShader;
        PixelShader pixelShader;
        public GlyphRenderer(Device device, DeviceContext context, ShaderCache cache, int maximumGlyphsPerDraw = 2048)
        {
            instances = new StructuredBuffer<GlyphInstance>(device, maximumGlyphsPerDraw, "Glyph Instances");
            //Every glyph uses two triangles.
            //Using redundant indices avoids a slow path for low triangle count instancing.
            var indexData = new uint[maximumGlyphsPerDraw * 6];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 6)
            {
                //Front facing triangles are counter clockwise.
                //Quad layout: 
                // 0____1
                // |  / |
                // | /  |
                // 2____3
                //0 2 1
                //1 2 3 
                indexData[glyphIndexStart + 0] = baseVertex + 0;
                indexData[glyphIndexStart + 1] = baseVertex + 2;
                indexData[glyphIndexStart + 2] = baseVertex + 1;
                indexData[glyphIndexStart + 3] = baseVertex + 1;
                indexData[glyphIndexStart + 4] = baseVertex + 2;
                indexData[glyphIndexStart + 5] = baseVertex + 3;
                baseVertex += 4;
            }
            indices = new IndexBuffer(indexData, device, "Glyph Indices");

            var samplerDescription = SamplerStateDescription.Default();
            samplerDescription.Filter = Filter.MinMagMipLinear;
            sampler = new SamplerState(device, samplerDescription);

            vertexConstants = new ConstantsBuffer<VertexConstants>(device, debugName: "Glyph Renderer Vertex Constants");
            pixelConstants = new ConstantsBuffer<Vector3>(device, debugName: "Glyph Renderer Pixel Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"Font\RenderGlyphs.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Font\RenderGlyphs.hlsl.pshader"));
        }

        /// <summary>
        /// Sets up the rendering pipeline with any glyph rendering specific render state that can be shared across all glyph batches drawn using the GlyphRenderer.Render function.
        /// </summary>
        /// <param name="context">Context to configure.</param>
        public void PreparePipeline(DeviceContext context)
        {
            //This assumes that rasterizer, blend, and depth states have been set appropriately for screenspace transparent rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, instances.SRV);
            context.PixelShader.Set(pixelShader);
            context.PixelShader.SetConstantBuffer(0, pixelConstants.Buffer);
            context.PixelShader.SetSampler(0, sampler);
        }

        public void Render(DeviceContext context, Font font, Int2 screenResolution, Vector2 horizontalAxis, Vector3 color, GlyphInstance[] glyphs, int start, int count)
        {
            var vertexConstantsData = new VertexConstants
            {
                HorizontalAxis = horizontalAxis,
                ScreenToNDCScale = new Vector2(2f / screenResolution.X, -2f / screenResolution.Y),
                InverseAtlasResolution = new Vector2(1f / font.Content.Atlas.Width, 1f / font.Content.Atlas.Height)
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            pixelConstants.Update(context, ref color);
            context.VertexShader.SetShaderResource(1, font.Sources.SRV);
            context.PixelShader.SetShaderResource(0, font.AtlasSRV);

            while (count > 0)
            {
                var batchCount = Math.Min(instances.Capacity, count);
                instances.Update(context, glyphs, batchCount, start);
                context.DrawIndexed(batchCount * 6, 0, 0);
                count -= batchCount;
                start += batchCount;
            }
        }
        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                instances.Dispose();
                indices.Dispose();
                sampler.Dispose();
                vertexConstants.Dispose();
                pixelConstants.Dispose();
            }
        }
    }
}
