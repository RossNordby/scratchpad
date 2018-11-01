using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    //We'll be using platform intrinsics rather than the System.Numerics jit intrinsics.
    [StructLayout(LayoutKind.Sequential, Size = 16)]
    public struct Vector3AOS
    {
        public float X;
        public float Y;
        public float Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CrossScalar(in Vector3AOS a, in Vector3AOS b, out Vector3AOS result)
        {
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DotScalar(in Vector3AOS a, in Vector3AOS b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ScaleScalar(in Vector3AOS v, float scale, out Vector3AOS result)
        {
            result.X = v.X * scale;
            result.Y = v.Y * scale;
            result.Z = v.Z * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static byte ShuffleControl(int z, int y, int x, int w)
        {
            return (byte)((z << 6) | (y << 4) | (x << 2) | w);
        }

        //NOTE: I didn't actually spend any time verifying that these work. They have the correct instructions with the proper dependency chains but I might have messed up the control masks.
        //Might want to double check if you use them!
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross4ShuffleSSE(in Vector128<float> a, in Vector128<float> b, out Vector128<float> result)
        {
            const byte yzxControl = (3 << 6) | (0 << 4) | (2 << 2) | 1;
            const byte zxyControl = (3 << 6) | (1 << 4) | (0 << 2) | 2;
            var shuffleA0 = Sse.Shuffle(a, a, yzxControl);
            var shuffleB0 = Sse.Shuffle(b, b, zxyControl);
            var shuffleA1 = Sse.Shuffle(a, a, zxyControl);
            var shuffleB1 = Sse.Shuffle(b, b, yzxControl);
            var left = Sse.Multiply(shuffleA0, shuffleB0);
            var right = Sse.Multiply(shuffleA1, shuffleB1);
            result = Sse.Subtract(left, right);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross3ShuffleSSE(in Vector128<float> a, in Vector128<float> b, out Vector128<float> result)
        {
            const byte yzxControl = (3 << 6) | (0 << 4) | (2 << 2) | 1;
            var shuffleA = Sse.Shuffle(a, a, yzxControl);
            var shuffleB = Sse.Shuffle(b, b, yzxControl);
            result = Sse.Subtract(Sse.Multiply(a, shuffleB), Sse.Multiply(b, shuffleA));
            result = Sse.Shuffle(result, result, yzxControl);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DotSSE(in Vector128<float> a, in Vector128<float> b, out Vector128<float> result)
        {
            result = Sse41.DotProduct(a, b, 0b00010111);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ScaleSSE(in Vector128<float> v, Vector128<float> scale, out Vector128<float> result)
        {
            result = Sse.Multiply(v, scale);
        }
    }
}
