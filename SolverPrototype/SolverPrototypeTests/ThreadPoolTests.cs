using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public class ThreadPoolTests : IThreadPool
    {
        public int ThreadCount => Environment.ProcessorCount;

        public void ForLoop(int startIndex, int exclusiveEndIndex, Action<int> loopBody)
        {
            Parallel.For(startIndex, exclusiveEndIndex, loopBody);
        }
    }

    public class NotQuiteAThreadPool : IThreadPool
    {
        public int ThreadCount => 1;

        public void ForLoop(int startIndex, int exclusiveEndIndex, Action<int> loopBody)
        {
            loopBody(0);
        }
    }

}
