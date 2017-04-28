using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
   

    public class NotQuiteAThreadPool : IThreadPool
    {
        public int ThreadCount => 1;

        public void ForLoop(int startIndex, int exclusiveEndIndex, Action<int> loopBody)
        {
            for (int i = startIndex; i < exclusiveEndIndex; ++i)
                loopBody(i);
        }
    }

    public class SimpleThreadPool : IThreadPool, IDisposable
    {
        int threadCount;
        public int ThreadCount => threadCount;
        struct Worker
        {
            public Thread Thread;
            public AutoResetEvent Signal;
        }

        Worker[] workers;
        AutoResetEvent finished;

        public SimpleThreadPool(int threadCount)
        {
            this.threadCount = threadCount;
            workers = new Worker[threadCount - 1];
            for (int i = 0; i < workers.Length; ++i)
            {
                workers[i] = new Worker { Thread = new Thread(WorkerLoop), Signal = new AutoResetEvent(false) };
                workers[i].Thread.IsBackground = true;
                workers[i].Thread.Start(workers[i].Signal);
            }
            finished = new AutoResetEvent(false);
        }

        int jobIndexCounter;
        int completedWorkerCounter;
        volatile Action<int> loopBody;
        volatile int exclusiveJobEndIndex;

        void ConsumeJobs()
        {
            Debug.Assert(this.loopBody != null);
            var loopBody = this.loopBody;
            int jobIndex;
            var exclusiveEnd = exclusiveJobEndIndex;
            while ((jobIndex = Interlocked.Increment(ref jobIndexCounter) - 1) < exclusiveEnd)
            {
                loopBody(jobIndex);
            }
            if (Interlocked.Increment(ref completedWorkerCounter) == threadCount)
            {
                finished.Set();
            }
        }

        void WorkerLoop(object untypedSignal)
        {
            var signal = (AutoResetEvent)untypedSignal;
            while (true)
            {
                signal.WaitOne();
                if (disposed)
                    return;
                ConsumeJobs();
            }
        }

        void SignalThreads()
        {
            for (int i = 0; i < workers.Length; ++i)
            {
                workers[i].Signal.Set();
            }
        }

        public void ForLoop(int startIndex, int exclusiveEndIndex, Action<int> loopBody)
        {
            Debug.Assert(this.loopBody == null);
            jobIndexCounter = startIndex;
            completedWorkerCounter = 0;
            exclusiveJobEndIndex = exclusiveEndIndex;
            this.loopBody = loopBody;
            SignalThreads();
            //Calling thread does work. No reason to spin up another worker and block this one!
            ConsumeJobs();
            finished.WaitOne();
            this.loopBody = null;
        }

        volatile bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                SignalThreads();
                foreach (var worker in workers)
                {
                    worker.Thread.Join();
                    worker.Signal.Dispose();
                }
            }
        }
    }

}
