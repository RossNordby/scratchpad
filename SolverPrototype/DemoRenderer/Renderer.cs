using DemoContentLoader;
using DemoRenderer.Background;
using DemoRenderer.PostProcessing;
using DemoRenderer.Properties;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DemoRenderer
{
    public class Renderer : IDisposable
    {
        public RenderSurface Surface { get; private set; }
        public ShaderCache ShaderCache { get; private set; }
        public BackgroundRenderer Background { get; private set; }
        public CompressToSwap CompressToSwap { get; private set; }

        Texture2D depthBuffer;
        DepthStencilView dsv;
        //Technically we could get away with rendering directly to the backbuffer, but a dedicated color buffer simplifies some things- 
        //you aren't bound by the requirements of the swapchain's buffer during rendering, and post processing is nicer.
        //Not entirely necessary for the demos, but hey, you could add MSAA resolves and tonemapping if you wanted?
        Texture2D colorBuffer;
        ShaderResourceView srv;
        RenderTargetView rtv;

        RasterizerState rasterizerState;
        DepthStencilState opaqueDepthState;
        BlendState opaqueBlendState;


        public Renderer(RenderSurface surface)
        {
            Surface = surface;
            using (var stream = new MemoryStream(Resources.DemoRendererShaders))
            {
                ShaderCache = ShaderCache.Load(stream);
            }
            Background = new BackgroundRenderer(surface.Device, ShaderCache);
            CompressToSwap = new CompressToSwap(surface.Device, ShaderCache);


            OnResize();
            var rasterizerStateDescription = RasterizerStateDescription.Default();
            rasterizerState = new RasterizerState(Surface.Device, rasterizerStateDescription);

            var depthStencilDescription = new DepthStencilStateDescription
            {
                IsDepthEnabled = true,
                DepthWriteMask = DepthWriteMask.All,
                //Note depth reversal.
                DepthComparison = Comparison.Greater,
                IsStencilEnabled = false
            };

            opaqueDepthState = new DepthStencilState(Surface.Device, depthStencilDescription);
            opaqueDepthState.DebugName = "Opaque Depth State";

            var blendStateDescription = BlendStateDescription.Default();
            opaqueBlendState = new BlendState(Surface.Device, blendStateDescription);
            opaqueBlendState.DebugName = "Opaque Blend State";
        }

        void OnResize()
        {
            Helpers.Dispose(ref depthBuffer);
            Helpers.Dispose(ref dsv);
            Helpers.Dispose(ref colorBuffer);
            Helpers.Dispose(ref rtv);

            var resolution = Surface.Resolution;
            depthBuffer = new Texture2D(Surface.Device, new Texture2DDescription
            {
                Format = Format.R32_Typeless,
                ArraySize = 1,
                MipLevels = 1,
                Width = resolution.X,
                Height = resolution.Y,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.DepthStencil,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            });
            depthBuffer.DebugName = "Depth Buffer";

            var depthStencilViewDescription = new DepthStencilViewDescription
            {
                Flags = DepthStencilViewFlags.None,
                Dimension = DepthStencilViewDimension.Texture2D,
                Format = Format.D32_Float,
                Texture2D = { MipSlice = 0 }
            };
            dsv = new DepthStencilView(Surface.Device, depthBuffer, depthStencilViewDescription);
            dsv.DebugName = "Depth DSV";

            //Using a 64 bit texture in the demos for lighting is pretty silly. But we gon do it.
            colorBuffer = new Texture2D(Surface.Device, new Texture2DDescription
            {
                Format = Format.R16G16B16A16_Float,
                ArraySize = 1,
                MipLevels = 1,
                Width = resolution.X,
                Height = resolution.Y,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            });
            colorBuffer.DebugName = "Color Buffer";

            srv = new ShaderResourceView(Surface.Device, colorBuffer);
            srv.DebugName = "Color SRV";

            rtv = new RenderTargetView(Surface.Device, colorBuffer);
            rtv.DebugName = "Color RTV";
        }

        public void Render(Camera camera)
        {
            if (Surface.Resolution.X != depthBuffer.Description.Width || Surface.Resolution.Y != depthBuffer.Description.Height)
            {
                OnResize();
            }
            var context = Surface.Context;

            //Note reversed depth.
            context.ClearDepthStencilView(dsv, DepthStencilClearFlags.Depth, 0, 0);
            //The background render is going to fill out the entire color buffer, but having a clear can be useful- e.g. clearing out MSAA history.
            //We don't use MSAA right now, but the cost of doing this clear is negligible and it avoids a surprise later.
            context.ClearRenderTargetView(rtv, new SharpDX.Mathematics.Interop.RawColor4());
            context.OutputMerger.SetRenderTargets(dsv, rtv);
            context.Rasterizer.State = rasterizerState;
            context.OutputMerger.SetBlendState(opaqueBlendState);
            context.OutputMerger.SetDepthStencilState(opaqueDepthState);
            var viewProjection = camera.ViewProjection;
            Background.Render(context, ref viewProjection);

            //Note that, for now, the compress to swap handles its own depth state since it's the only post processing stage.
            context.OutputMerger.SetBlendState(opaqueBlendState);
            context.Rasterizer.State = rasterizerState;
            CompressToSwap.Render(context, srv, Surface.RTV);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Background.Dispose();
                CompressToSwap.Dispose();

                depthBuffer.Dispose();
                dsv.Dispose();
                colorBuffer.Dispose();
                rtv.Dispose();
                rasterizerState.Dispose();
                opaqueDepthState.Dispose();
                opaqueBlendState.Dispose();
            }
        }

#if DEBUG
        ~Renderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
