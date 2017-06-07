using BEPUutilities2;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Background;
using DemoUtilities;
using OpenTK.Input;
using System;
using System.IO;

namespace SolverPrototypeTests
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.ReadKey();
            var window = new Window("pretty cool red window", new Int2(512, 512), WindowMode.Windowed);
            var loop = new GameLoop(window);
            var demo = new BasicDemo(loop.Input, loop.Camera);
            loop.Run(demo.Update);
            loop.Dispose();
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