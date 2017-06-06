using BEPUutilities2;
using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;
using System;

namespace SolverPrototypeTests
{
    class Program
    {
        static void Main(string[] args)
        {
            var window = new Window("pretty cool red window", new Int2(512, 512), WindowMode.Windowed);
            var surface = new RenderSurface(window.Handle, window.Resolution);
            var input = new Input(window);
            var camera = new Camera(window.Resolution.X / window.Resolution.Y, (float)Math.PI / 2);
            Console.WriteLine($"Initial Forward: {camera.Forward}");
            Console.WriteLine($"Initial Right: {camera.Right}");
            Console.WriteLine($"Initial Up: {camera.Up}");
            camera.Yaw = (float)Math.PI / 4;
            Console.WriteLine($"Yawed Forward: {camera.Forward}");
            Console.WriteLine($"Yawed Right: {camera.Right}");
            Console.WriteLine($"Yawed Up: {camera.Up}");
            camera.Yaw = 0;
            camera.Pitch = (float)Math.PI / 4;
            Console.WriteLine($"Pitch Forward: {camera.Forward}");
            Console.WriteLine($"Pitch Right: {camera.Right}");
            Console.WriteLine($"Pitch Up: {camera.Up}");
            camera.Yaw = (float)Math.PI / 4; ;
            camera.Pitch = (float)Math.PI / 4;
            Console.WriteLine($"YawPitch Forward: {camera.Forward}");
            Console.WriteLine($"YawPitch Right: {camera.Right}");
            Console.WriteLine($"YawPitch Up: {camera.Up}");
            window.Run(dt =>
            {
                if(input.WasPushed(Key.P))
                {
                    Console.WriteLine("P pushed.");
                }
                if (input.WasPushed(MouseButton.Middle))
                {
                    Console.WriteLine("button pushed.");
                }
                surface.Context.ClearRenderTargetView(surface.RTV, new SharpDX.Mathematics.Interop.RawColor4(1, 0, 0, 1));
                surface.Present();
                input.Flush();
            }, newResolution => surface.Resize(newResolution, false));
            surface.Dispose();
            window.Dispose();

            //ReinterpretVectorRepro.Test();
            //TriangularTests.Test();
            //LocalsinitCodegen.Test();
            //InterlockedOverheadTests.Test();
            ConstraintDescriptionMappingTests.Test();
            //LocalsinitCodegen.Test();
            AutoTester.Test();
            //ContactManifoldConvergenceTests.Test();
            //BallSocketConvergenceTests.Test();
            //MemoryResizeTests.Test();
            //ConstraintCacheOptimizationTests.Test();
            //SortTest.Test();
            //SpanCodegenTests.Test();

        }
    }
}