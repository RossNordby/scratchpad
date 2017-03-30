using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    public static class SpanCodeGenTests
    {

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double Test<TSpan>(ref TSpan span, int iterations, out int accumulator) where TSpan : ISpan<int>
        {
            var start = Stopwatch.GetTimestamp();
            accumulator = 0;
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < span.Length; ++j)
                {
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];
                    accumulator += span[j] + span[j];

                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                    //accumulator += span.Get(j) + span.Get(j);
                }
            }

            return (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
        }
        public static unsafe void Test()
        {
            var array = new int[32];

            fixed (int* pointer = array)
            {
                Buffer<int> pSpan = new Buffer<int>(pointer, array.Length);
                Array<int> mSpan = new Array<int>(array);

                Console.WriteLine($"Warmup: {Test(ref pSpan, 1, out int accumulator)}, {Test(ref mSpan, 1, out accumulator)}");
                const int iterations = 2000000;
                Console.WriteLine($"Pointer: {Test(ref pSpan, iterations, out accumulator)}");
                Console.WriteLine($"Managed: {Test(ref mSpan, iterations, out accumulator)}");
            }
        }
    }
}
