using System;

namespace SolverPrototype
{
    /// <summary>
    /// Provides multithreading dispatch primitives and a thread count for the simulation to use.
    /// </summary>
    public interface IThreadPool
    {
        int ThreadCount { get; }
        void ForLoop(int startIndex, int exclusiveEndIndex, Action<int> loopBody);
    }
}
