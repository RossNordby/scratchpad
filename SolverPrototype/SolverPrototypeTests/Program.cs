using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SolverPrototype
{
    class Program
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        static int TestVectorAccess(int iterationCount)
        {
            var vector = new Vector<int>();
            int accumulator = 0;
            Unsafe
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    accumulator += vector[i];
                }
            }

            return accumulator;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static int TestVectorAccess(int iterationCount)
        {
            var vector = new Vector<int>();
            int accumulator = 0;
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    accumulator += vector[i];
                }
            }

            return accumulator;
        }

        static void Main(string[] args)
        {
            const int iterationCount = 100000;
            var vectorized = TestVectorAccess(iterationCount);
            var vectorized = TestVectorAccess(iterationCount);
        }
    }
}
