using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Text;

namespace IntrinsicsTesting
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
    }
}
