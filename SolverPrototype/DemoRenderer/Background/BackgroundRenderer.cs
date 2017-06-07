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
        ConstantsBuffer<Matrix4x4> constants;
        public BackgroundRenderer(Device device, ShaderCache cache)
        {
            vertexShader = new VertexShader(device, cache.GetShader(@"Background\RenderBackground.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Background\RenderBackground.hlsl.pshader"));
            constants = new ConstantsBuffer<Matrix4x4>(device, debugName: "BackgroundRenderer Constants");
        }


        public void Render(DeviceContext context, ref Matrix4x4 viewProjection)
        {
            Matrix4x4.Invert(viewProjection, out var constantsData);
            constantsData = Matrix4x4.Transpose(constantsData); //Compensate for the shader packing we use.
            constants.Update(context, ref constantsData);

            context.InputAssembler.InputLayout = null;
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, constants.Buffer);
            context.PixelShader.Set(pixelShader);
            context.Draw(3, 0);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                constants.Dispose();
            }
        }
    }
}
