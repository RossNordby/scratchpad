using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace CodegenTests
{
    class Program
    {
        public struct V
        {
            public Vector<float> X;
            public Vector<float> Y;
            public Vector<float> Z;

            //public float X;
            //public float Y;
            //public float Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add(in V a, in V b, out V result)
            {
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static V operator +(in V a, in V b)
            {
                V result;
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
                return result;
            }

        }

        unsafe static void Main(string[] args)
        {
            const int outerIterationCount = 1 << 15;
            const int innerIterationCount = 1 << 10;
            var values = new V[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                //values[i] = new V { X = i * 0.25f, Y = i * 0.5f, Z = i * 0.75f };
                values[i] = new V { X = new Vector<float>(i * 0.25f), Y = new Vector<float>(i * 0.5f), Z = new Vector<float>(i * 0.75f) };
            }
            {                
                V accumulator = default;
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < outerIterationCount; ++i)
                {
                    for (int j = 0; j < innerIterationCount; ++j)
                    {
                        ref var value = ref values[j];
                        V.Add(value, value, out var r0);
                        V.Add(value, value, out var r1);
                        V.Add(value, value, out var r2);
                        V.Add(value, value, out var r3);
                        V.Add(r0, r1, out var i0);
                        V.Add(r2, r3, out var i1);
                        V.Add(i0, i1, out var i2);
                        V.Add(i2, accumulator, out accumulator);
                    }
                }
                var end = Stopwatch.GetTimestamp();
                Console.WriteLine($"Accumulator: {accumulator.X + accumulator.Y + accumulator.Z}");
                Console.WriteLine($"Add() time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
            }
            {
                V accumulator = default;
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < outerIterationCount; ++i)
                {
                    for (int j = 0; j < innerIterationCount; ++j)
                    {
                        ref var value = ref values[j];
                        var r0 = value + value;
                        var r1 = value + value;
                        var r2 = value + value;
                        var r3 = value + value;
                        var i0 = r0 + r1;
                        var i1 = r2 + r3;
                        var i2 = i0 + i1;
                        accumulator += i2;
                    }
                }
                
                var end = Stopwatch.GetTimestamp();
                Console.WriteLine($"Accumulator: {accumulator.X + accumulator.Y + accumulator.Z}");
                Console.WriteLine($"+ time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
            }
        }
    }
}
