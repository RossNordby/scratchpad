using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace IntrinsicsTesting
{
    public struct VNumerics2
    {
        public Vector<float> X;
        public Vector<float> Y;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in VNumerics2 a, in VNumerics2 b, out VNumerics2 result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static VNumerics2 operator +(in VNumerics2 a, in VNumerics2 b)
        {
            VNumerics2 result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            return result;
        }


        public static unsafe float ManuallyInlinedNumerics2(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics2>(setupData);
            Vector<float> accumulatorX = new Vector<float>(0f);
            Vector<float> accumulatorY = new Vector<float>(0f);
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var r0x = value.X + value.X;
                var r0y = value.Y + value.Y;
                var r1x = value.X + value.X;
                var r1y = value.Y + value.Y;
                var r2x = value.X + value.X;
                var r2y = value.Y + value.Y;
                var r3x = value.X + value.X;
                var r3y = value.Y + value.Y;

                var i0x = r0x + r1x;
                var i0y = r0y + r1y;
                var i1x = r2x + r3x;
                var i1y = r2y + r3y;

                var i2x = i0x + i1x;
                var i2y = i0y + i1y;

                accumulatorX = i2x + accumulatorX;
                accumulatorY = i2y + accumulatorY;
            }
            var toReturn = accumulatorX + accumulatorY;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float ManuallyInlinedNumerics2WithLoadCaching(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics2>(setupData);
            Vector<float> accumulatorX = new Vector<float>(0f);
            Vector<float> accumulatorY = new Vector<float>(0f);
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var x = value.X;
                var y = value.Y;
                var r0x = x + x;
                var r0y = y + y;
                var r1x = x + x;
                var r1y = y + y;
                var r2x = x + x;
                var r2y = y + y;
                var r3x = x + x;
                var r3y = y + y;

                var i0x = r0x + r1x;
                var i0y = r0y + r1y;
                var i1x = r2x + r3x;
                var i1y = r2y + r3y;

                var i2x = i0x + i1x;
                var i2y = i0y + i1y;

                accumulatorX = i2x + accumulatorX;
                accumulatorY = i2y + accumulatorY;
            }
            var toReturn = accumulatorX + accumulatorY;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float AddFunctionNumerics2(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics2>(setupData);
            VNumerics2 accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                VNumerics2.Add(value, value, out var r0);
                VNumerics2.Add(value, value, out var r1);
                VNumerics2.Add(value, value, out var r2);
                VNumerics2.Add(value, value, out var r3);
                VNumerics2.Add(r0, r1, out var i0);
                VNumerics2.Add(r2, r3, out var i1);
                VNumerics2.Add(i0, i1, out var i2);
                VNumerics2.Add(i2, accumulator, out accumulator);
            }
            var toReturn = accumulator.X + accumulator.Y;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float OperatorNumerics2(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics2>(setupData);
            VNumerics2 accumulator = default;
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
            var toReturn = accumulator.X + accumulator.Y;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }
    }

}
