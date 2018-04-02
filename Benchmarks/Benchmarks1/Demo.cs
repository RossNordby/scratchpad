using BenchmarkDotNet.Attributes;
using BEPUphysics;
using BEPUutilities.Threading;
using System;
using System.Collections.Generic;

namespace Benchmarks1
{
    public abstract class Demo
    {
        [ParamsSource(nameof(ThreadCountGrabber))]
        public int ThreadCount;
        public int[] ThreadCountGrabber
        {
            get
            {
                var threadJobs = new List<int>();
                for (int threadCount = 1; threadCount <= Environment.ProcessorCount; threadCount += 1 << Math.Max(0, ((int)Math.Log(threadCount, 2) - 1)))
                    threadJobs.Add(threadCount);
                return threadJobs.ToArray();
            }
        }

        protected Space Space;
        ParallelLooper parallelLooper;

        protected void CommonSetup()
        {
            if (ThreadCount > 1)
            {
                parallelLooper = new ParallelLooper();
                for (int i = 0; i < ThreadCount; ++i)
                {
                    parallelLooper.AddThread();
                }
                Space = new Space(parallelLooper);
            }
            else
            {
                parallelLooper = null;
                Space = new Space();
            }
        }

        [IterationSetup]
        public abstract void IterationSetup();

        [IterationCleanup]
        public void IterationCleanup()
        {
            parallelLooper?.Dispose();
        }

        [Benchmark]
        public void Update()
        {
            Space.Update(1 / 60f);
        }
    }
}
