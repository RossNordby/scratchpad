﻿using DemoContentLoader;
using DemoRenderer.Background;
using DemoRenderer.UI;
using DemoRenderer.PostProcessing;
using DemoRenderer.Properties;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using DemoRenderer.Shapes;
using SolverPrototype;
using DemoRenderer.Constraints;

namespace DemoRenderer
{
    public class Renderer : IDisposable
    {
        public RenderSurface Surface { get; private set; }
        public ShaderCache ShaderCache { get; private set; }
        public BackgroundRenderer Background { get; private set; }
        //TODO: Down the road, the sphere renderer will be joined by a bunch of other types. 
        //They'll likely be stored in an array indexed by a shape type rather than just being a swarm of properties.
        public SphereRenderer SphereRenderer { get; private set; }
        public ShapesExtractor Shapes { get; private set; }
        public LineRenderer LineRenderer { get; private set; }
        public LineExtractor Lines { get; private set; }
        public GlyphRenderer GlyphRenderer { get; private set; }
        public UILineRenderer UILineRenderer { get; private set; }
        public CompressToSwap CompressToSwap { get; private set; }

        public TextBatcher TextBatcher { get; private set; }
        public UILineBatcher UILineBatcher { get; private set; }

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
        DepthStencilState uiDepthState;
        BlendState uiBlendState;


        public Renderer(RenderSurface surface)
        {
            Surface = surface;
            using (var stream = new MemoryStream(Resources.DemoRendererShaders))
            {
                ShaderCache = ShaderCache.Load(stream);
            }
            Shapes = new ShapesExtractor();
            SphereRenderer = new SphereRenderer(surface.Device, ShaderCache);
            Lines = new LineExtractor();
            LineRenderer = new LineRenderer(surface.Device, ShaderCache);
            Background = new BackgroundRenderer(surface.Device, ShaderCache);
            CompressToSwap = new CompressToSwap(surface.Device, ShaderCache);

            GlyphRenderer = new GlyphRenderer(surface.Device, surface.Context, ShaderCache);
            TextBatcher = new TextBatcher();
            UILineRenderer = new UILineRenderer(surface.Device, ShaderCache);
            UILineBatcher = new UILineBatcher();

            OnResize();
            var rasterizerStateDescription = RasterizerStateDescription.Default();
            rasterizerStateDescription.IsFrontCounterClockwise = true;
            rasterizerState = new RasterizerState(Surface.Device, rasterizerStateDescription);
            rasterizerState.DebugName = "Default Rasterizer State";

            var opaqueDepthStencilDescription = new DepthStencilStateDescription
            {
                IsDepthEnabled = true,
                DepthWriteMask = DepthWriteMask.All,
                //Note depth reversal.
                DepthComparison = Comparison.Greater,
                IsStencilEnabled = false
            };

            opaqueDepthState = new DepthStencilState(Surface.Device, opaqueDepthStencilDescription);
            opaqueDepthState.DebugName = "Opaque Depth State";

            var opaqueBlendStateDescription = BlendStateDescription.Default();
            opaqueBlendState = new BlendState(Surface.Device, opaqueBlendStateDescription);
            opaqueBlendState.DebugName = "Opaque Blend State";

            var uiDepthStateDescription = new DepthStencilStateDescription
            {
                IsDepthEnabled = false,
                DepthWriteMask = DepthWriteMask.Zero,
                //Note depth reversal.
                DepthComparison = Comparison.Greater,
                IsStencilEnabled = false
            };

            uiDepthState = new DepthStencilState(Surface.Device, uiDepthStateDescription);
            uiDepthState.DebugName = "UI Depth State";

            //The UI will use premultiplied alpha.
            var uiBlendStateDescription = BlendStateDescription.Default();
            uiBlendStateDescription.RenderTarget[0].IsBlendEnabled = true;
            uiBlendStateDescription.RenderTarget[0].SourceBlend = BlendOption.One;
            uiBlendStateDescription.RenderTarget[0].SourceAlphaBlend = BlendOption.One;
            uiBlendStateDescription.RenderTarget[0].DestinationBlend = BlendOption.InverseSourceAlpha;
            uiBlendStateDescription.RenderTarget[0].DestinationAlphaBlend = BlendOption.InverseSourceAlpha;
            uiBlendStateDescription.RenderTarget[0].BlendOperation = BlendOperation.Add;
            uiBlendStateDescription.RenderTarget[0].AlphaBlendOperation = BlendOperation.Add;
            uiBlendStateDescription.RenderTarget[0].RenderTargetWriteMask = ColorWriteMaskFlags.All;
            uiBlendState = new BlendState(Surface.Device, uiBlendStateDescription);
            uiBlendState.DebugName = "UI Blend State";
        }

        void OnResize()
        {
            Helpers.Dispose(ref depthBuffer);
            Helpers.Dispose(ref dsv);
            Helpers.Dispose(ref colorBuffer);
            Helpers.Dispose(ref rtv);

            var resolution = Surface.Resolution;

            TextBatcher.Resolution = resolution;
            UILineBatcher.Resolution = resolution;

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
            context.Rasterizer.SetViewport(0, 0, Surface.Resolution.X, Surface.Resolution.Y, 0.0f, 1.0f);

            //Note reversed depth.
            context.ClearDepthStencilView(dsv, DepthStencilClearFlags.Depth, 0, 0);
            //The background render is going to fill out the entire color buffer, but having a clear can be useful- e.g. clearing out MSAA history.
            //We don't use MSAA right now, but the cost of doing this clear is negligible and it avoids a surprise later.
            context.ClearRenderTargetView(rtv, new SharpDX.Mathematics.Interop.RawColor4());
            context.OutputMerger.SetRenderTargets(dsv, rtv);
            context.Rasterizer.State = rasterizerState;
            context.OutputMerger.SetBlendState(opaqueBlendState);
            context.OutputMerger.SetDepthStencilState(opaqueDepthState);

            SphereRenderer.Render(context, camera, Shapes.spheres.Span.Memory, 0, Shapes.spheres.Count);
            LineRenderer.Render(context, camera, Surface.Resolution, Lines.lines.Span.Memory, 0, Lines.lines.Count);

            Background.Render(context, camera);

            //Glyph and screenspace line drawing rely on the same premultiplied alpha blending transparency. We'll handle their state out here.
            context.OutputMerger.SetBlendState(uiBlendState);
            context.OutputMerger.SetDepthStencilState(uiDepthState);
            UILineBatcher.Flush(context, Surface.Resolution, UILineRenderer);
            GlyphRenderer.PreparePipeline(context);
            TextBatcher.Flush(context, Surface.Resolution, GlyphRenderer);

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

                UILineRenderer.Dispose();
                GlyphRenderer.Dispose();

                depthBuffer.Dispose();
                dsv.Dispose();
                colorBuffer.Dispose();
                rtv.Dispose();
                rasterizerState.Dispose();
                opaqueDepthState.Dispose();
                opaqueBlendState.Dispose();
                uiDepthState.Dispose();
                uiBlendState.Dispose();
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
