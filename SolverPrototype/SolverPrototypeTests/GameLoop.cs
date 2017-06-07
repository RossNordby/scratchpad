using DemoRenderer;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Text;
using BEPUutilities2;
using OpenTK;

namespace SolverPrototypeTests
{
    public class GameLoop : IDisposable
    {
        public Window Window { get; private set; }
        public Input Input { get; private set; }
        public Camera Camera { get; private set; }
        public Action<float> OnUpdate { get; set; }
        RenderSurface surface;
        Renderer renderer;

        public GameLoop(Window window)
        {
            Window = window;
            Input = new Input(window);
            var useDebugLayer =
#if DEBUG
                true;
#else
                false;
#endif
            surface = new RenderSurface(window.Handle, window.Resolution, enableDeviceDebugLayer: useDebugLayer);

            renderer = new Renderer(surface);
            Camera = new Camera(window.Resolution.X / window.Resolution.Y, (float)Math.PI / 2, 0.01f, 100000);
        }

        void Update(float dt)
        {
            //We'll let the delgate's logic handle the variable time steps.
            OnUpdate(dt);
            renderer.Render(Camera);
            surface.Present();
            Input.Flush();
        }

        public void Run(Action<float> onUpdate)
        {
            OnUpdate = onUpdate;
            Window.Run(Update, OnResize);
        }

        private void OnResize(Int2 resolution)
        {
            //We just don't support true fullscreen in the demos. Would be pretty pointless.
            renderer.Surface.Resize(resolution, false);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Input.Dispose();
                renderer.Dispose();
                //Note that we do not own the window.
            }
        }
    }
}
