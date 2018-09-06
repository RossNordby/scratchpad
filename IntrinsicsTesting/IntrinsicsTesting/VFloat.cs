using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace IntrinsicsTesting
{
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
    }
}
