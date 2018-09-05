using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace CodegenTests
{
    class Program
    {
        public struct VAvx
        {
            public Vector256<float> X;
            public Vector256<float> Y;
            public Vector256<float> Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add(in VAvx a, in VAvx b, out VAvx result)
            {
                result.X = Avx.Add(a.X, b.X);
                result.Y = Avx.Add(a.Y, b.Y);
                result.Z = Avx.Add(a.Z, b.Z);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static VAvx operator +(in VAvx a, in VAvx b)
            {
                VAvx result;
                result.X = Avx.Add(a.X, b.X);
                result.Y = Avx.Add(a.Y, b.Y);
                result.Z = Avx.Add(a.Z, b.Z);
                return result;
            }
        }

        public struct VNumerics
        {
            public Vector<float> X;
            public Vector<float> Y;
            public Vector<float> Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add(in VNumerics a, in VNumerics b, out VNumerics result)
            {
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static VNumerics operator +(in VNumerics a, in VNumerics b)
            {
                VNumerics result;
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
                return result;
            }
        }

        public struct VFloat
        {
            public float X;
            public float Y;
            public float Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add(in VFloat a, in VFloat b, out VFloat result)
            {
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static VFloat operator +(in VFloat a, in VFloat b)
            {
                VFloat result;
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
                return result;
            }
        }

        public static unsafe float ManuallyInlinedAVX(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VAvx>(setupData);
            Vector256<float> accumulatorX = Avx.SetAllVector256(0f);
            Vector256<float> accumulatorY = Avx.SetAllVector256(0f);
            Vector256<float> accumulatorZ = Avx.SetAllVector256(0f);
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var x = Avx.LoadVector256((float*)Unsafe.AsPointer(ref value.X));
                var y = Avx.LoadVector256((float*)Unsafe.AsPointer(ref value.Y));
                var z = Avx.LoadVector256((float*)Unsafe.AsPointer(ref value.Z));
                var r0x = Avx.Add(x, x);
                var r0y = Avx.Add(y, y);
                var r0z = Avx.Add(z, z);
                var r1x = Avx.Add(x, x);
                var r1y = Avx.Add(y, y);
                var r1z = Avx.Add(z, z);
                var r2x = Avx.Add(x, x);
                var r2y = Avx.Add(y, y);
                var r2z = Avx.Add(z, z);
                var r3x = Avx.Add(x, x);
                var r3y = Avx.Add(y, y);
                var r3z = Avx.Add(z, z);

                var i0x = Avx.Add(r0x, r1x);
                var i0y = Avx.Add(r0y, r1y);
                var i0z = Avx.Add(r0z, r1z);
                var i1x = Avx.Add(r2x, r3x);
                var i1y = Avx.Add(r2y, r3y);
                var i1z = Avx.Add(r2z, r3z);

                var i2x = Avx.Add(i0x, i1x);
                var i2y = Avx.Add(i0y, i1y);
                var i2z = Avx.Add(i0z, i1z);

                accumulatorX = Avx.Add(i2x, accumulatorX);
                accumulatorY = Avx.Add(i2y, accumulatorY);
                accumulatorZ = Avx.Add(i2z, accumulatorZ);
            }
            var axy = Avx.Add(accumulatorX, accumulatorY);
            var toReturn = Avx.Add(axy, accumulatorZ);
            return Unsafe.As<Vector256<float>, float>(ref toReturn);
        }

        public static unsafe float AddFunctionAVX(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VAvx>(setupData);
            VAvx accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                VAvx.Add(value, value, out var r0);
                VAvx.Add(value, value, out var r1);
                VAvx.Add(value, value, out var r2);
                VAvx.Add(value, value, out var r3);
                VAvx.Add(r0, r1, out var i0);
                VAvx.Add(r2, r3, out var i1);
                VAvx.Add(i0, i1, out var i2);
                VAvx.Add(i2, accumulator, out accumulator);
            }
            var axy = Avx.Add(accumulator.X, accumulator.Y);
            var toReturn = Avx.Add(axy, accumulator.Z);
            return Unsafe.As<Vector256<float>, float>(ref toReturn);
        }

        public static unsafe float OperatorAVX(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VAvx>(setupData);
            VAvx accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                var r0 = value + value;
                var r1 = value + value;
                var r2 = value + value;
                var r3 = value + value;
                var i0 = r0 + r1;
                var i1 = r2 + r3;
                var i2 = i0 + i1;
                accumulator += i2;
            }
            var axy = Avx.Add(accumulator.X, accumulator.Y);
            var toReturn = Avx.Add(axy, accumulator.Z);
            return Unsafe.As<Vector256<float>, float>(ref toReturn);
        }

        public static unsafe float ManuallyInlinedNumerics(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics>(setupData);
            Vector<float> accumulatorX = new Vector<float>(0f);
            Vector<float> accumulatorY = new Vector<float>(0f);
            Vector<float> accumulatorZ = new Vector<float>(0f);
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var r0x = value.X + value.X;
                var r0y = value.Y + value.Y;
                var r0z = value.Z + value.Z;
                var r1x = value.X + value.X;
                var r1y = value.Y + value.Y;
                var r1z = value.Z + value.Z;
                var r2x = value.X + value.X;
                var r2y = value.Y + value.Y;
                var r2z = value.Z + value.Z;
                var r3x = value.X + value.X;
                var r3y = value.Y + value.Y;
                var r3z = value.Z + value.Z;

                var i0x = r0x + r1x;
                var i0y = r0y + r1y;
                var i0z = r0z + r1z;
                var i1x = r2x + r3x;
                var i1y = r2y + r3y;
                var i1z = r2z + r3z;

                var i2x = i0x + i1x;
                var i2y = i0y + i1y;
                var i2z = i0z + i1z;

                accumulatorX = i2x + accumulatorX;
                accumulatorY = i2y + accumulatorY;
                accumulatorZ = i2z + accumulatorZ;
            }
            var toReturn = accumulatorX + accumulatorY + accumulatorZ;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float AddFunctionNumerics(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics>(setupData);
            VNumerics accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                VNumerics.Add(value, value, out var r0);
                VNumerics.Add(value, value, out var r1);
                VNumerics.Add(value, value, out var r2);
                VNumerics.Add(value, value, out var r3);
                VNumerics.Add(r0, r1, out var i0);
                VNumerics.Add(r2, r3, out var i1);
                VNumerics.Add(i0, i1, out var i2);
                VNumerics.Add(i2, accumulator, out accumulator);
            }
            var axy = accumulator.X + accumulator.Y;
            var toReturn = axy + accumulator.Z;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float OperatorNumerics(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics>(setupData);
            VNumerics accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                var r0 = value + value;
                var r1 = value + value;
                var r2 = value + value;
                var r3 = value + value;
                var i0 = r0 + r1;
                var i1 = r2 + r3;
                var i2 = i0 + i1;
                accumulator += i2;
            }
            var axy = accumulator.X + accumulator.Y;
            var toReturn = axy + accumulator.Z;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }


        public static unsafe float ManuallyInlinedFloat(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VFloat>(setupData);
            float accumulatorX = 0;
            float accumulatorY = 0;
            float accumulatorZ = 0;
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var r0x = value.X + value.X;
                var r0y = value.Y + value.Y;
                var r0z = value.Z + value.Z;
                var r1x = value.X + value.X;
                var r1y = value.Y + value.Y;
                var r1z = value.Z + value.Z;
                var r2x = value.X + value.X;
                var r2y = value.Y + value.Y;
                var r2z = value.Z + value.Z;
                var r3x = value.X + value.X;
                var r3y = value.Y + value.Y;
                var r3z = value.Z + value.Z;

                var i0x = r0x + r1x;
                var i0y = r0y + r1y;
                var i0z = r0z + r1z;
                var i1x = r2x + r3x;
                var i1y = r2y + r3y;
                var i1z = r2z + r3z;

                var i2x = i0x + i1x;
                var i2y = i0y + i1y;
                var i2z = i0z + i1z;

                accumulatorX = i2x + accumulatorX;
                accumulatorY = i2y + accumulatorY;
                accumulatorZ = i2z + accumulatorZ;
            }
            return accumulatorX + accumulatorY + accumulatorZ;
        }

        public static unsafe float AddFunctionFloat(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VFloat>(setupData);
            VFloat accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                VFloat.Add(value, value, out var r0);
                VFloat.Add(value, value, out var r1);
                VFloat.Add(value, value, out var r2);
                VFloat.Add(value, value, out var r3);
                VFloat.Add(r0, r1, out var i0);
                VFloat.Add(r2, r3, out var i1);
                VFloat.Add(i0, i1, out var i2);
                VFloat.Add(i2, accumulator, out accumulator);
            }
            return accumulator.X + accumulator.Y + accumulator.Z;
        }

        public static unsafe float OperatorFloat(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VFloat>(setupData);
            VFloat accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                var r0 = value + value;
                var r1 = value + value;
                var r2 = value + value;
                var r3 = value + value;
                var i0 = r0 + r1;
                var i1 = r2 + r3;
                var i2 = i0 + i1;
                accumulator += i2;
            }
            return accumulator.X + accumulator.Y + accumulator.Z;
        }

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
            const int outerIterationCount = 1 << 15;
            const int innerIterationCount = 1 << 10;

            Console.WriteLine($"Floats");
            var floatValues = new VFloat[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                floatValues[i] = new VFloat { X = i * 0.25f, Y = i * 0.5f, Z = i * 0.75f };
            }
            var floatHandle = GCHandle.Alloc(floatValues, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(ManuallyInlinedFloat), Unsafe.AsPointer(ref floatValues[0]), ManuallyInlinedFloat);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(AddFunctionFloat), Unsafe.AsPointer(ref floatValues[0]), AddFunctionFloat);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(OperatorFloat), Unsafe.AsPointer(ref floatValues[0]), OperatorFloat);
            floatHandle.Free();

            Console.WriteLine();
            Console.WriteLine($"System.Numerics, vector width in floats: {Vector<float>.Count}");
            var numericsValues = new VNumerics[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                numericsValues[i] = new VNumerics { X = new Vector<float>(i * 0.25f), Y = new Vector<float>(i * 0.5f), Z = new Vector<float>(i * 0.75f) };
            }
            var numericsHandle = GCHandle.Alloc(numericsValues, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(ManuallyInlinedNumerics), Unsafe.AsPointer(ref numericsValues[0]), ManuallyInlinedNumerics);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(AddFunctionNumerics), Unsafe.AsPointer(ref numericsValues[0]), AddFunctionNumerics);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(OperatorNumerics), Unsafe.AsPointer(ref numericsValues[0]), OperatorNumerics);
            numericsHandle.Free();


            Console.WriteLine();
            Console.WriteLine("Platform intrinsics AVX");
            var avxValues = new VAvx[innerIterationCount];
            for (int i = 0; i < innerIterationCount; ++i)
            {
                avxValues[i] = new VAvx { X = Avx.SetAllVector256(i * 0.25f), Y = Avx.SetAllVector256(i * 0.5f), Z = Avx.SetAllVector256(i * 0.75f) };
            }

            var avxHandle = GCHandle.Alloc(avxValues, GCHandleType.Pinned);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(ManuallyInlinedAVX), Unsafe.AsPointer(ref avxValues[0]), ManuallyInlinedAVX);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(AddFunctionAVX), Unsafe.AsPointer(ref avxValues[0]), AddFunctionAVX);
            Test(innerIterationCount, outerIterationCount, 1024, nameof(OperatorAVX), Unsafe.AsPointer(ref avxValues[0]), OperatorAVX);
            avxHandle.Free();


        }
    }
}
