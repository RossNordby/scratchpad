using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    interface ISpan<T>
    {
        ref T this[int i] { get; }
        T Get(int i);
        int Capacity { get; }
    }

    public unsafe struct PointerSpan<T> : ISpan<T>
    {
        public readonly byte* Pointer;
        int capacity;
        public int Capacity { get { return capacity; } }

        public PointerSpan(void* pointer, int capacity)
        {
            Pointer = (byte*)pointer;
            this.capacity = capacity;
        }
        public ref T this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.Add(ref Unsafe.As<byte, T>(ref *Pointer), i);
                //return ref Unsafe.Add(ref Unsafe.AsRef<T>(Pointer), i);
                //return ref Unsafe.As<byte, T>(ref *(Pointer + i * Unsafe.SizeOf<T>()));
                //return ref Unsafe.AsRef<T>(Pointer + i * Unsafe.SizeOf<T>());
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Get(int i)
        {
            //return ref Unsafe.Add(ref Unsafe.AsRef<T>(Pointer), i);
            //return Unsafe.As<byte, T>(ref *(Pointer + i * Unsafe.SizeOf<T>()));
            //return Unsafe.AsRef<T>(Pointer + i * Unsafe.SizeOf<T>());
            //return Unsafe.Read<T>(Pointer + i * Unsafe.SizeOf<T>());
            return Unsafe.Add(ref Unsafe.As<byte, T>(ref *Pointer), i);
        }
    }
    public unsafe struct ManagedSpan<T> : ISpan<T>
    {
        public T[] Array;
        public int Capacity => Array.Length;

        public ref T this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.Add(ref Array[0], i);
                //return ref Array[i];
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Get(int i)
        {
            //return Unsafe.Add(ref Array[0], i);
            return Array[i];
        }
    }

    public static class SpanCodeGenTests
    {

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double Test<TSpan>(ref TSpan span, int iterations, out int accumulator) where TSpan : ISpan<int>
        {
            var start = Stopwatch.GetTimestamp();
            accumulator = 0;
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < span.Capacity; ++j)
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
                PointerSpan<int> pSpan = new PointerSpan<int>(pointer, array.Length);
                ManagedSpan<int> mSpan = new ManagedSpan<int> { Array = array };

                int accumulator;
                Console.WriteLine($"Warmup: {Test(ref pSpan, 1, out accumulator)}, {Test(ref mSpan, 1, out accumulator)}");
                const int iterations = 2000000;
                Console.WriteLine($"Pointer: {Test(ref pSpan, iterations, out accumulator)}");
                Console.WriteLine($"Managed: {Test(ref mSpan, iterations, out accumulator)}");
            }
        }
    }
}
