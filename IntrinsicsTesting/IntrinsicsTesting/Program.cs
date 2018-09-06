using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace IntrinsicsTesting
{
    [StructLayout(LayoutKind.Explicit, Size = 256 / 8)]
    public struct Vector256
    {
    }

    class Program
    {
        public unsafe delegate float InnerLoop(void* setupData, int innerIterationCount);

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static unsafe void Test(int innerIterationCount, int outerIterations, int warmUpIterations, string name, void* setupData, InnerLoop innerLoop)
        {
            float accumulator = 0;
            for (int i = 0; i < warmUpIterations; ++i)
            {
                accumulator += innerLoop(setupData, innerIterationCount);
            }
            Console.WriteLine($"Warm up accumulator: {accumulator}");
            accumulator = 0;
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < outerIterations; ++i)
            {
                accumulator += innerLoop(setupData, innerIterationCount);
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Accumulator: {accumulator}");
            Console.WriteLine($"{name} time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
        }


        unsafe static void Main(string[] args)
        {
            const int outerIterationCount = 1 << 13;
            const int innerIterationCount = 1 << 10;

            Console.WriteLine($"Floats");
            var floatValues = new VFloat[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                floatValues[i] = new VFloat { X = i * 0.25f, Y = i * 0.5f, Z = i * 0.75f };
            }
            var floatHandle = GCHandle.Alloc(floatValues, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VFloat.ManuallyInlinedFloat), Unsafe.AsPointer(ref floatValues[0]), VFloat.ManuallyInlinedFloat);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VFloat.AddFunctionFloat), Unsafe.AsPointer(ref floatValues[0]), VFloat.AddFunctionFloat);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VFloat.OperatorFloat), Unsafe.AsPointer(ref floatValues[0]), VFloat.OperatorFloat);
            floatHandle.Free();

            Console.WriteLine();
            Console.WriteLine($"System.Numerics, 3d vector, each vector width in floats: {Vector<float>.Count}");
            var numerics3Values = new VNumerics3[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                numerics3Values[i] = new VNumerics3 { X = new Vector<float>(i * 0.25f), Y = new Vector<float>(i * 0.5f), Z = new Vector<float>(i * 0.75f) };
            }
            var numerics3Handle = GCHandle.Alloc(numerics3Values, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics3.ManuallyInlinedNumerics3), Unsafe.AsPointer(ref numerics3Values[0]), VNumerics3.ManuallyInlinedNumerics3);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics3.ManuallyInlinedNumerics3WithLoadCaching), Unsafe.AsPointer(ref numerics3Values[0]), VNumerics3.ManuallyInlinedNumerics3WithLoadCaching);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics3.AddFunctionNumerics3), Unsafe.AsPointer(ref numerics3Values[0]), VNumerics3.AddFunctionNumerics3);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics3.OperatorNumerics3), Unsafe.AsPointer(ref numerics3Values[0]), VNumerics3.OperatorNumerics3);
            numerics3Handle.Free();

            Console.WriteLine();
            Console.WriteLine($"System.Numerics, 2d vector");
            var numerics2Values = new VNumerics2[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                numerics2Values[i] = new VNumerics2 { X = new Vector<float>(i * 0.25f), Y = new Vector<float>(i * 0.5f) };
            }
            var numerics2Handle = GCHandle.Alloc(numerics2Values, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics2.ManuallyInlinedNumerics2), Unsafe.AsPointer(ref numerics2Values[0]), VNumerics2.ManuallyInlinedNumerics2);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics2.ManuallyInlinedNumerics2WithLoadCaching), Unsafe.AsPointer(ref numerics2Values[0]), VNumerics2.ManuallyInlinedNumerics2WithLoadCaching);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics2.AddFunctionNumerics2), Unsafe.AsPointer(ref numerics2Values[0]), VNumerics2.AddFunctionNumerics2);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VNumerics2.OperatorNumerics2), Unsafe.AsPointer(ref numerics2Values[0]), VNumerics2.OperatorNumerics2);
            numerics2Handle.Free();


            Console.WriteLine();
            Console.WriteLine("Platform intrinsics AVX");
            var avxValues = new VAvx[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                avxValues[i] = new VAvx { X = Avx.SetAllVector256(i * 0.25f), Y = Avx.SetAllVector256(i * 0.5f), Z = Avx.SetAllVector256(i * 0.75f) };
            }

            var avxHandle = GCHandle.Alloc(avxValues, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvx.ManuallyInlinedAVX), Unsafe.AsPointer(ref avxValues[0]), VAvx.ManuallyInlinedAVX);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvx.AddFunctionAVX), Unsafe.AsPointer(ref avxValues[0]), VAvx.AddFunctionAVX);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvx.OperatorAVX), Unsafe.AsPointer(ref avxValues[0]), VAvx.OperatorAVX);

            Console.WriteLine();
            Console.WriteLine("Platform intrinsics AVX Pointer");
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvxPointer.AddFunctionAVXPointer), Unsafe.AsPointer(ref avxValues[0]), VAvxPointer.AddFunctionAVXPointer);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvxPointer.OperatorAVXPointer), Unsafe.AsPointer(ref avxValues[0]), VAvxPointer.OperatorAVXPointer);

            Console.WriteLine();
            Console.WriteLine("Platform intrinsics AVX Ref Struct");
            Test(innerIterationCount, outerIterationCount, 1024, nameof(VAvxRefStruct.OperatorAVXRefStruct), Unsafe.AsPointer(ref avxValues[0]), VAvxRefStruct.OperatorAVXRefStruct);
            avxHandle.Free();

        }
    }
}
