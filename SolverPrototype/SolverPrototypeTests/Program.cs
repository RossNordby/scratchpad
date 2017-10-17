﻿using BEPUutilities2;
using DemoContentLoader;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK;
using OpenTK.Input;
using SolverPrototype;
using SolverPrototypeTests.Properties;
using SolverPrototypeTests.SpecializedTests;
using System;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    class Program
    {
        static void Main(string[] args)
        {
            //ConstraintDescriptionMappingTests.Test();
            //CollidablePairComparerTests.Test();
            //HeadlessDemo.Simple();
            //BatchedCollisionTests.Test();
            //TypeIdCodeGenTests.Test();
            //DeterminismTest.Test();
            //return;
            //ScalarWideTests.Test();
            //DenseFlagTests.Test();
            //VirtualOverheadTest.Test();
            //TreeTest.Test();
            //ReinterpretVectorRepro.Test();
            //TriangularTests.Test();
            //LocalsinitCodegen.Test();
            //InterlockedOverheadTests.Test();
            //LocalsinitCodegen.Test();
            //AutoTester.Test();
            //ContactManifoldConvergenceTests.Test();
            //BallSocketConvergenceTests.Test();
            //MemoryResizeTests.Test();
            //ConstraintCacheOptimizationTests.Test();
            //SortTest.Test();
            //SpanCodegenTests.Test();
            //return;

            //Console.ReadKey();
            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = new MemoryStream(Resources.SolverPrototypeTestsContent))
            {
                content = ContentArchive.Load(stream);
            }
            //var fontContent = content.Load<FontContent>(@"Content\Courier Prime Sans.ttf");
            var fontContent = content.Load<FontContent>(@"Content\Carlito-Regular.ttf");
            var font = new Font(loop.Surface.Device, loop.Surface.Context, fontContent);
            var demo = new DemoHarness(window, loop.Input, loop.Camera, font);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();

        }
    }
}