using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Text;

namespace IntrinsicsTesting
{

    public unsafe ref struct VAvxRefStruct
    {
        public Vector256 X;
        public Vector256 Y;
        public Vector256 Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static VAvxRefStruct operator +(VAvxRefStruct a, VAvxRefStruct b)
        {
            var ax = Avx.LoadVector256((float*)&a.X);
            var ay = Avx.LoadVector256((float*)&a.Y);
            var az = Avx.LoadVector256((float*)&a.Z);
            var bx = Avx.LoadVector256((float*)&b.X);
            var by = Avx.LoadVector256((float*)&b.Y);
            var bz = Avx.LoadVector256((float*)&b.Z);
            var rx = Avx.Add(ax, bx);
            var ry = Avx.Add(ay, by);
            var rz = Avx.Add(az, bz);
            VAvxRefStruct result;
            Avx.Store((float*)&result.X, rx);
            Avx.Store((float*)&result.Y, ry);
            Avx.Store((float*)&result.Z, rz);
            return result;
        }
        
        public static unsafe float OperatorAVXRefStruct(void* setupData, int innerIterationCount)
        {
            var values = (VAvxRefStruct*)setupData;
            VAvxRefStruct accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                var value = values[j];
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
