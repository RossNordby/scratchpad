using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace IntrinsicsTesting
{
    public struct VNumerics3
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in VNumerics3 a, in VNumerics3 b, out VNumerics3 result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static VNumerics3 operator +(in VNumerics3 a, in VNumerics3 b)
        {
            VNumerics3 result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            return result;
        }


        public static unsafe float ManuallyInlinedNumerics3(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics3>(setupData);
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

        public static unsafe float ManuallyInlinedNumerics3WithLoadCaching(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics3>(setupData);
            Vector<float> accumulatorX = new Vector<float>(0f);
            Vector<float> accumulatorY = new Vector<float>(0f);
            Vector<float> accumulatorZ = new Vector<float>(0f);
            for (int i = 0; i < innerIterationCount; ++i)
            {
                ref var value = ref Unsafe.Add(ref baseValue, i);
                var x = value.X;
                var y = value.Y;
                var z = value.Z;
                var r0x = x + x;
                var r0y = y + y;
                var r0z = z + z;
                var r1x = x + x;
                var r1y = y + y;
                var r1z = z + z;
                var r2x = x + x;
                var r2y = y + y;
                var r2z = z + z;
                var r3x = x + x;
                var r3y = y + y;
                var r3z = z + z;

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

        public static unsafe float AddFunctionNumerics3(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics3>(setupData);
            VNumerics3 accumulator = default;
            for (int j = 0; j < innerIterationCount; ++j)
            {
                ref var value = ref Unsafe.Add(ref baseValue, j);
                VNumerics3.Add(value, value, out var r0);
                VNumerics3.Add(value, value, out var r1);
                VNumerics3.Add(value, value, out var r2);
                VNumerics3.Add(value, value, out var r3);
                VNumerics3.Add(r0, r1, out var i0);
                VNumerics3.Add(r2, r3, out var i1);
                VNumerics3.Add(i0, i1, out var i2);
                VNumerics3.Add(i2, accumulator, out accumulator);
            }
            var axy = accumulator.X + accumulator.Y;
            var toReturn = axy + accumulator.Z;
            return Unsafe.As<Vector<float>, float>(ref toReturn);
        }

        public static unsafe float OperatorNumerics3(void* setupData, int innerIterationCount)
        {
            ref var baseValue = ref Unsafe.AsRef<VNumerics3>(setupData);
            VNumerics3 accumulator = default;
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
    }

}
