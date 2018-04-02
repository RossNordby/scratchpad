using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;

namespace Benchmarks2
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
        protected Simulation Simulation;
        BufferPool bufferPool;
        SimpleThreadDispatcher threadDispatcher;

        protected void CommonSetup()
        {
            bufferPool = new BufferPool();
            threadDispatcher = ThreadCount > 1 ? new SimpleThreadDispatcher(ThreadCount) : null;
            Simulation = Simulation.Create(bufferPool, new TestCallbacks());
        }

        [IterationSetup]
        public abstract void IterationSetup();

        [IterationCleanup]
        public void IterationCleanup()
        {
            Simulation.Dispose();
            bufferPool.Clear();
            threadDispatcher?.Dispose();
        }

        [Benchmark]
        public void Update()
        {
            Simulation.Timestep(1 / 60f, threadDispatcher);
        }
    }
}
