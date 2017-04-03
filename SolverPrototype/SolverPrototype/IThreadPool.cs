using System;

namespace SolverPrototype
{

    public partial class Solver
    {
        /// <summary>
        /// Provides multithreading dispatch primitives and a thread count for the simulation to use.
        /// </summary>
        public interface IThreadPool
        {
            int ThreadCount { get; }
            void ForLoop(Action<int> loopBody, int startIndex, int exclusiveEndIndex);
        }


    }
}
