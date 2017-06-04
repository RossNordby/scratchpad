using DemosContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemosRenderer.Background
{
    public class BackgroundRenderer
    {
        VertexShader vertexShader;
        PixelShader pixelShader;
        public BackgroundRenderer(Device device, ShaderCache cache)
        {
            vertexShader = new VertexShader(device, cache.GetShader(@"Background\RasterizeBackground.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Background\RasterizeBackground.hlsl.pshader"));
        }

        public void Render()
        {

        }
    }
}
