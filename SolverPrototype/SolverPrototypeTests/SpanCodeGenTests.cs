using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public unsafe struct PointerSpan<T>
    {
        public void* Pointer;

        public ref T this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.Add(ref Unsafe.AsRef<T>(Pointer), i);
            }
        }
    }

    public static class SpanCodeGenTests
    {
        public static unsafe void Test()
        {
            var array = new byte[1024];

            int result;
            fixed (byte* pointer = array)
            {

                PointerSpan<int> span;
                span.Pointer = pointer;

                result = span[8];

            }
            Console.WriteLine($"value: {result}");
        }
    }
}
