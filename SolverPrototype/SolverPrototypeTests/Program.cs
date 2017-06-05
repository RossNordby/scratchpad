using BEPUutilities2;
using DemoRenderer;
using DemosUtilities;

namespace SolverPrototypeTests
{
    class Program
    {
        static void Main(string[] args)
        {

            var window = new Window("pretty cool red window", new Int2(512, 512), WindowMode.Windowed);
            var surface = new RenderSurface(window.Handle, window.Resolution);
            window.Run(dt =>
                {
                    surface.Context.ClearRenderTargetView(surface.RTV, new SharpDX.Mathematics.Interop.RawColor4(1,0,0,1));
                    surface.Present();
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