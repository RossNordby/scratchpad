using BEPUutilities2;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Background;
using DemoRenderer.Font;
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
            var window = new Window("pretty cool multicolored window", new Int2(512, 512), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = File.OpenRead("DemoContent.ca"))
            {
                content = ContentArchive.Load(stream);
            }
            var fontContent = content.Load<FontContent>(@"Fonts\linear-by-braydon-fuller.otf");
            var font = new Font(loop.Surface.Device, loop.Surface.Context, fontContent);
            var demo = new BasicDemo(window, loop.Input, loop.Camera, font);
            loop.Run(demo.Update, demo.Render);
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