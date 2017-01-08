using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    class GatherScatterTests
    {

        struct Context
        {
            public BodyVelocities[] BodyVelocities;
            public BodyReferences[] BodyReferences;
            public int IterationCount;
        }
        static Context GetFreshContext(int iterationCount, int bundleCount)
        {
            Context context;
            context.IterationCount = iterationCount;
            context.BodyVelocities = new BodyVelocities[bundleCount];
            context.BodyReferences = new BodyReferences[iterationCount];
            var random = new Random(5);
            int maximumBodyIndex = bundleCount * Vector<float>.Count;
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    var indexA = random.Next(maximumBodyIndex);
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexA, i) = indexA >> Solver.VectorShift;
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexA, i) = indexA & Solver.VectorMask;
                    var indexB = random.Next(maximumBodyIndex);
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexB, i) = indexB >> Solver.VectorShift;
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexB, i) = indexB & Solver.VectorMask;
                }
                context.BodyReferences[iterationIndex].Count = Vector<int>.Count;
            }
            return context;
        }

        static double Time(Action<Context> action, int iterationCount, int bundleCount)
        {
            var context = GetFreshContext(iterationCount, bundleCount);
            var timer = Timer.Start();
            action(context);
            return timer.Stop() / iterationCount;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestRefGather(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.IterationCount; ++i)
            {
                GatherScatter.GatherVelocities(context.BodyVelocities, ref context.BodyReferences[i], ref a, ref b);
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestPointerGather(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.IterationCount; ++i)
            {
                GatherScatter.GatherVelocities2(context.BodyVelocities, ref context.BodyReferences[i], ref a, ref b);
            }
        }

        struct Timer
        {
            long begin;

            public static Timer Start()
            {
                return new Timer { begin = Stopwatch.GetTimestamp() };
            }
            public double Stop()
            {
                return (Stopwatch.GetTimestamp() - begin) / (double)Stopwatch.Frequency;
            }
        }

        public static void Test()
        {
            const int iterationCount = 10000000;
            const int bundleCount = 8192;
            TestRefGather(GetFreshContext(iterationCount, bundleCount));
            TestPointerGather(GetFreshContext(iterationCount, bundleCount));

            GC.Collect();

            var refTime = Time(TestRefGather, iterationCount, bundleCount);
            GC.Collect();
            var pointerTime = Time(TestPointerGather, iterationCount, bundleCount);

            const double scaling = 1e9;

            Console.WriteLine($"Ref gather time (ns): {refTime * scaling}, Pointer gather time (ns): {pointerTime * scaling}");

        }

    }
}
