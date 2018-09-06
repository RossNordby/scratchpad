using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Text;

namespace IntrinsicsTesting
{
    public unsafe struct VAvxPointer
    {
        public Vector256 X;
        public Vector256 Y;
        public Vector256 Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(VAvxPointer* a, VAvxPointer* b, VAvxPointer* result)
        {
            var ax = Avx.LoadVector256((float*)&a->X);
            var ay = Avx.LoadVector256((float*)&a->Y);
            var az = Avx.LoadVector256((float*)&a->Z);
            var bx = Avx.LoadVector256((float*)&b->X);
            var by = Avx.LoadVector256((float*)&b->Y);
            var bz = Avx.LoadVector256((float*)&b->Z);
            var rx = Avx.Add(ax, bx);
            var ry = Avx.Add(ay, by);
            var rz = Avx.Add(az, bz);
            Avx.Store((float*)&result->X, rx);
            Avx.Store((float*)&result->Y, ry);
            Avx.Store((float*)&result->Z, rz);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static VAvxPointer operator +(in VAvxPointer a, in VAvxPointer b)
        {
            var aPointer = (VAvxPointer*)Unsafe.AsPointer(ref Unsafe.AsRef(a));
            var bPointer = (VAvxPointer*)Unsafe.AsPointer(ref Unsafe.AsRef(b));
            var ax = Avx.LoadVector256((float*)&aPointer->X);
            var ay = Avx.LoadVector256((float*)&aPointer->Y);
            var az = Avx.LoadVector256((float*)&aPointer->Z);
            var bx = Avx.LoadVector256((float*)&bPointer->X);
            var by = Avx.LoadVector256((float*)&bPointer->Y);
            var bz = Avx.LoadVector256((float*)&bPointer->Z);
            var rx = Avx.Add(ax, bx);
            var ry = Avx.Add(ay, by);
            var rz = Avx.Add(az, bz);
            VAvxPointer result;
            Avx.Store((float*)&result.X, rx);
            Avx.Store((float*)&result.Y, ry);
            Avx.Store((float*)&result.Z, rz);
            return result;
        }


        public static unsafe float AddFunctionAVXPointer(void* setupData, int innerIterationCount)
        {
            var values = (VAvxPointer*)setupData;
            VAvxPointer accumulator = default;
            VAvxPointer r0 = default, r1 = default, r2 = default, r3 = default, i0 = default, i1 = default, i2 = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                var value = values + j;
                VAvxPointer.Add(value, value, &r0);
                VAvxPointer.Add(value, value, &r1);
                VAvxPointer.Add(value, value, &r2);
                VAvxPointer.Add(value, value, &r3);
                VAvxPointer.Add(&r0, &r1, &i0);
                VAvxPointer.Add(&r2, &r3, &i1);
                VAvxPointer.Add(&i0, &i1, &i2);
                VAvxPointer.Add(&i2, &accumulator, &accumulator);
            }
            var axy = Avx.Add(Unsafe.As<Vector256, Vector256<float>>(ref accumulator.X), Unsafe.As<Vector256, Vector256<float>>(ref accumulator.Y));
            var toReturn = Avx.Add(axy, Unsafe.As<Vector256, Vector256<float>>(ref accumulator.Z));
            return Unsafe.As<Vector256<float>, float>(ref toReturn);
        }

        public static unsafe float OperatorAVXPointer(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VAvxPointer>(setupData);
            VAvxPointer accumulator = default;
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
            var axy = Avx.Add(Unsafe.As<Vector256, Vector256<float>>(ref accumulator.X), Unsafe.As<Vector256, Vector256<float>>(ref accumulator.Y));
            var toReturn = Avx.Add(axy, Unsafe.As<Vector256, Vector256<float>>(ref accumulator.Z));
            return Unsafe.As<Vector256<float>, float>(ref toReturn);
        }
    }

}
