using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;

namespace DemoRenderer.Background
{
    public class BackgroundRenderer : IDisposable
    {
        VertexShader vertexShader;
        PixelShader pixelShader;
        ConstantBuffer<Matrix4x4> constants;
        public BackgroundRenderer(Device device, ShaderCache cache)
        {
            vertexShader = new VertexShader(device, cache.GetShader(@"Background\RasterizeBackground.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Background\RasterizeBackground.hlsl.pshader"));
            constants = new ConstantBuffer<Matrix4x4>(device, debugName: "BackgroundRenderer Constants");
        }


        public void Render(DeviceContext context)
        {
            context.Draw(3, 0);
        }

        bool disposed;
        public void Dispose()
        {
            if(!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                constants.Dispose();
            }
        }
    }
}
