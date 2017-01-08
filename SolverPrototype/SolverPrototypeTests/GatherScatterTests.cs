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
        static Context GetFreshContext(int iterationCount, int bundleCount, double nullConnectionProbability = 0)
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
                    if (random.NextDouble() >= nullConnectionProbability)
                    {
                        var index = random.Next(maximumBodyIndex);
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexA, i) = index >> Solver.VectorShift;
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexA, i) = index & Solver.VectorMask;
                    }
                    else
                    {
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexA, i) = 0;
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexA, i) = Vector<float>.Count << 1;
                    }
                    if (random.NextDouble() >= nullConnectionProbability)
                    {
                        var index = random.Next(maximumBodyIndex);
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexB, i) = index >> Solver.VectorShift;
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexB, i) = index & Solver.VectorMask;
                    }
                    else
                    {
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].BundleIndexB, i) = 0;
                        GatherScatter.Get(ref context.BodyReferences[iterationIndex].InnerIndexB, i) = Vector<float>.Count << 1;
                    }
                }
                context.BodyReferences[iterationIndex].Count = Vector<int>.Count;
            }
            return context;
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

        static double Time(Action<Context> action, int iterationCount, int bundleCount, double nullConnectionProbability = 0)
        {
            var context = GetFreshContext(iterationCount, bundleCount, nullConnectionProbability);
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
        static void TestRefGather2(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.IterationCount; ++i)
            {
                GatherScatter.GatherVelocities2(context.BodyVelocities, ref context.BodyReferences[i], ref a, ref b);
            }
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestRefGather3(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.IterationCount; ++i)
            {
                GatherScatter.GatherVelocities3(context.BodyVelocities, ref context.BodyReferences[i], ref a, ref b);
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestRefScatter(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.IterationCount; ++i)
            {
                GatherScatter.ScatterVelocities(context.BodyVelocities, ref context.BodyReferences[i], ref a, ref b);
            }
        }



        public static void Test()
        {
            TestRefGather(GetFreshContext(1, 1));
            TestRefGather2(GetFreshContext(1, 1));
            TestRefGather3(GetFreshContext(1, 1));
            TestRefScatter(GetFreshContext(1, 1));

            const int iterationCount = 10000000;
            const int bundleCount = 8192;
            GC.Collect();

            var refGatherTime = Time(TestRefGather, iterationCount, bundleCount, 0.7);
            GC.Collect();
            var refGather2Time = Time(TestRefGather2, iterationCount, bundleCount);
            GC.Collect();
            var refGather3Time = Time(TestRefGather3, iterationCount, bundleCount, 0.7);
            GC.Collect();
            //var refScatterTime = Time(TestRefScatter, iterationCount, bundleCount);

            const double scaling = 1e9;

            //Console.WriteLine($"Ref gather time (ns): {refGatherTime * scaling}, Ref gather 2 time (ns): {refGather2Time * scaling}, ref scatter time (ns): {refScatterTime * scaling}");
            Console.WriteLine($"Ref gather time (ns): {refGatherTime * scaling}, Ref gather 2 time (ns): {refGather2Time * scaling}, Ref gather 3 time (ns): {refGather3Time * scaling}");

        }

    }
}
