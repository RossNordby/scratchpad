using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using static BepuScatter.Tracing.VectorCasts;
namespace BepuScatter.Tracing
{
    [StructLayout(LayoutKind.Sequential, Size = 32)]
    public struct Wide<T>
    {
        public T Value;
    }
    public static class Wide
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Add(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Add(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.Add(To128(ref Unsafe.Add(ref a.Value, 4)), To128(ref Unsafe.Add(ref b.Value, 4)));
            }
            else
            {
                result.Value = a.Value + b.Value;
                Unsafe.Add(ref result.Value, 1) = Unsafe.Add(ref a.Value, 1) + Unsafe.Add(ref b.Value, 1);
                Unsafe.Add(ref result.Value, 2) = Unsafe.Add(ref a.Value, 2) + Unsafe.Add(ref b.Value, 2);
                Unsafe.Add(ref result.Value, 3) = Unsafe.Add(ref a.Value, 3) + Unsafe.Add(ref b.Value, 3);
                Unsafe.Add(ref result.Value, 4) = Unsafe.Add(ref a.Value, 4) + Unsafe.Add(ref b.Value, 4);
                Unsafe.Add(ref result.Value, 5) = Unsafe.Add(ref a.Value, 5) + Unsafe.Add(ref b.Value, 5);
                Unsafe.Add(ref result.Value, 6) = Unsafe.Add(ref a.Value, 6) + Unsafe.Add(ref b.Value, 6);
                Unsafe.Add(ref result.Value, 7) = Unsafe.Add(ref a.Value, 7) + Unsafe.Add(ref b.Value, 7);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Subtract(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Subtract(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.Subtract(To128(ref Unsafe.Add(ref a.Value, 4)), To128(ref Unsafe.Add(ref b.Value, 4)));
            }
            else
            {
                result.Value = a.Value * b.Value;
                Unsafe.Add(ref result.Value, 1) = Unsafe.Add(ref a.Value, 1) - Unsafe.Add(ref b.Value, 1);
                Unsafe.Add(ref result.Value, 2) = Unsafe.Add(ref a.Value, 2) - Unsafe.Add(ref b.Value, 2);
                Unsafe.Add(ref result.Value, 3) = Unsafe.Add(ref a.Value, 3) - Unsafe.Add(ref b.Value, 3);
                Unsafe.Add(ref result.Value, 4) = Unsafe.Add(ref a.Value, 4) - Unsafe.Add(ref b.Value, 4);
                Unsafe.Add(ref result.Value, 5) = Unsafe.Add(ref a.Value, 5) - Unsafe.Add(ref b.Value, 5);
                Unsafe.Add(ref result.Value, 6) = Unsafe.Add(ref a.Value, 6) - Unsafe.Add(ref b.Value, 6);
                Unsafe.Add(ref result.Value, 7) = Unsafe.Add(ref a.Value, 7) - Unsafe.Add(ref b.Value, 7);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Multiply(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Multiply(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.Multiply(To128(ref Unsafe.Add(ref a.Value, 4)), To128(ref Unsafe.Add(ref b.Value, 4)));
            }
            else
            {
                result.Value = a.Value * b.Value;
                Unsafe.Add(ref result.Value, 1) = Unsafe.Add(ref a.Value, 1) * Unsafe.Add(ref b.Value, 1);
                Unsafe.Add(ref result.Value, 2) = Unsafe.Add(ref a.Value, 2) * Unsafe.Add(ref b.Value, 2);
                Unsafe.Add(ref result.Value, 3) = Unsafe.Add(ref a.Value, 3) * Unsafe.Add(ref b.Value, 3);
                Unsafe.Add(ref result.Value, 4) = Unsafe.Add(ref a.Value, 4) * Unsafe.Add(ref b.Value, 4);
                Unsafe.Add(ref result.Value, 5) = Unsafe.Add(ref a.Value, 5) * Unsafe.Add(ref b.Value, 5);
                Unsafe.Add(ref result.Value, 6) = Unsafe.Add(ref a.Value, 6) * Unsafe.Add(ref b.Value, 6);
                Unsafe.Add(ref result.Value, 7) = Unsafe.Add(ref a.Value, 7) * Unsafe.Add(ref b.Value, 7);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Divide(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Divide(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Divide(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.Divide(To128(ref Unsafe.Add(ref a.Value, 4)), To128(ref Unsafe.Add(ref b.Value, 4)));
            }
            else
            {
                result.Value = a.Value / b.Value;
                Unsafe.Add(ref result.Value, 1) = Unsafe.Add(ref a.Value, 1) / Unsafe.Add(ref b.Value, 1);
                Unsafe.Add(ref result.Value, 2) = Unsafe.Add(ref a.Value, 2) / Unsafe.Add(ref b.Value, 2);
                Unsafe.Add(ref result.Value, 3) = Unsafe.Add(ref a.Value, 3) / Unsafe.Add(ref b.Value, 3);
                Unsafe.Add(ref result.Value, 4) = Unsafe.Add(ref a.Value, 4) / Unsafe.Add(ref b.Value, 4);
                Unsafe.Add(ref result.Value, 5) = Unsafe.Add(ref a.Value, 5) / Unsafe.Add(ref b.Value, 5);
                Unsafe.Add(ref result.Value, 6) = Unsafe.Add(ref a.Value, 6) / Unsafe.Add(ref b.Value, 6);
                Unsafe.Add(ref result.Value, 7) = Unsafe.Add(ref a.Value, 7) / Unsafe.Add(ref b.Value, 7);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Min(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Min(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result, 4)) = Sse42.Min(To128(ref Unsafe.Add(ref a, 4)), To128(ref Unsafe.Add(ref b, 4)));
            }
            else
            {
                result.Value = MathF.Min(a.Value, b.Value);
                Unsafe.Add(ref result.Value, 1) = MathF.Min(Unsafe.Add(ref a.Value, 1), Unsafe.Add(ref b.Value, 1));
                Unsafe.Add(ref result.Value, 2) = MathF.Min(Unsafe.Add(ref a.Value, 2), Unsafe.Add(ref b.Value, 2));
                Unsafe.Add(ref result.Value, 3) = MathF.Min(Unsafe.Add(ref a.Value, 3), Unsafe.Add(ref b.Value, 3));
                Unsafe.Add(ref result.Value, 4) = MathF.Min(Unsafe.Add(ref a.Value, 4), Unsafe.Add(ref b.Value, 4));
                Unsafe.Add(ref result.Value, 5) = MathF.Min(Unsafe.Add(ref a.Value, 5), Unsafe.Add(ref b.Value, 5));
                Unsafe.Add(ref result.Value, 6) = MathF.Min(Unsafe.Add(ref a.Value, 6), Unsafe.Add(ref b.Value, 6));
                Unsafe.Add(ref result.Value, 7) = MathF.Min(Unsafe.Add(ref a.Value, 7), Unsafe.Add(ref b.Value, 7));
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(ref Wide<float> a, ref Wide<float> b, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Max(To256(ref a), To256(ref b));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Max(To128(ref a), To128(ref b));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.Max(To128(ref Unsafe.Add(ref a.Value, 4)), To128(ref Unsafe.Add(ref b.Value, 4)));
            }
            else
            {
                result.Value = MathF.Max(a.Value, b.Value);
                Unsafe.Add(ref result.Value, 1) = MathF.Max(Unsafe.Add(ref a.Value, 1), Unsafe.Add(ref b.Value, 1));
                Unsafe.Add(ref result.Value, 2) = MathF.Max(Unsafe.Add(ref a.Value, 2), Unsafe.Add(ref b.Value, 2));
                Unsafe.Add(ref result.Value, 3) = MathF.Max(Unsafe.Add(ref a.Value, 3), Unsafe.Add(ref b.Value, 3));
                Unsafe.Add(ref result.Value, 4) = MathF.Max(Unsafe.Add(ref a.Value, 4), Unsafe.Add(ref b.Value, 4));
                Unsafe.Add(ref result.Value, 5) = MathF.Max(Unsafe.Add(ref a.Value, 5), Unsafe.Add(ref b.Value, 5));
                Unsafe.Add(ref result.Value, 6) = MathF.Max(Unsafe.Add(ref a.Value, 6), Unsafe.Add(ref b.Value, 6));
                Unsafe.Add(ref result.Value, 7) = MathF.Max(Unsafe.Add(ref a.Value, 7), Unsafe.Add(ref b.Value, 7));
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sqrt(ref Wide<float> v, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.Sqrt(To256(ref v));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result) = Sse42.Sqrt(To128(ref v));
                To128(ref Unsafe.Add(ref result, 4)) = Sse42.Sqrt(To128(ref Unsafe.Add(ref v, 4)));
            }
            else
            {
                result.Value = MathF.Sqrt(v.Value);
                Unsafe.Add(ref result.Value, 1) = MathF.Sqrt(Unsafe.Add(ref v.Value, 1));
                Unsafe.Add(ref result.Value, 2) = MathF.Sqrt(Unsafe.Add(ref v.Value, 2));
                Unsafe.Add(ref result.Value, 3) = MathF.Sqrt(Unsafe.Add(ref v.Value, 3));
                Unsafe.Add(ref result.Value, 4) = MathF.Sqrt(Unsafe.Add(ref v.Value, 4));
                Unsafe.Add(ref result.Value, 5) = MathF.Sqrt(Unsafe.Add(ref v.Value, 5));
                Unsafe.Add(ref result.Value, 6) = MathF.Sqrt(Unsafe.Add(ref v.Value, 6));
                Unsafe.Add(ref result.Value, 7) = MathF.Sqrt(Unsafe.Add(ref v.Value, 7));
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(ref Wide<int> condition, ref Wide<float> left, ref Wide<float> right, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.BlendVariable(To256(ref left), To256(ref right), To256<int, float>(ref condition));
            }
            else if (Sse42.IsSupported)
            {
                To128(ref result.Value) = Sse42.BlendVariable(To128(ref left), To128(ref right), To128<int, float>(ref condition));
                To128(ref Unsafe.Add(ref result.Value, 4)) = Sse42.BlendVariable(To128(ref Unsafe.Add(ref left.Value, 4)), To128(ref Unsafe.Add(ref right.Value, 4)), To128<int, float>(ref Unsafe.Add(ref condition.Value, 4)));
            }
            else
            {
                result.Value = condition.Value >= 0 ? left.Value : right.Value;
                Unsafe.Add(ref result.Value, 1) = Unsafe.Add(ref condition.Value, 1) >= 0 ? Unsafe.Add(ref left.Value, 1) : Unsafe.Add(ref right.Value, 1);
                Unsafe.Add(ref result.Value, 2) = Unsafe.Add(ref condition.Value, 2) >= 0 ? Unsafe.Add(ref left.Value, 2) : Unsafe.Add(ref right.Value, 2);
                Unsafe.Add(ref result.Value, 3) = Unsafe.Add(ref condition.Value, 3) >= 0 ? Unsafe.Add(ref left.Value, 3) : Unsafe.Add(ref right.Value, 3);
                Unsafe.Add(ref result.Value, 4) = Unsafe.Add(ref condition.Value, 4) >= 0 ? Unsafe.Add(ref left.Value, 4) : Unsafe.Add(ref right.Value, 4);
                Unsafe.Add(ref result.Value, 5) = Unsafe.Add(ref condition.Value, 5) >= 0 ? Unsafe.Add(ref left.Value, 5) : Unsafe.Add(ref right.Value, 5);
                Unsafe.Add(ref result.Value, 6) = Unsafe.Add(ref condition.Value, 6) >= 0 ? Unsafe.Add(ref left.Value, 6) : Unsafe.Add(ref right.Value, 6);
                Unsafe.Add(ref result.Value, 7) = Unsafe.Add(ref condition.Value, 7) >= 0 ? Unsafe.Add(ref left.Value, 7) : Unsafe.Add(ref right.Value, 7);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Broadcast(float scalar, out Wide<float> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.BroadcastScalarToVector256(&scalar);
            }
            else if (Sse42.IsSupported)
            {
                //TODO: Does Vector256.Create work without AVX?
                To128(ref result) = Vector128.Create(scalar);
                To128(ref Unsafe.Add(ref result.Value, 4)) = Vector128.Create(scalar);
            }
            else
            {
                result.Value = scalar;
                Unsafe.Add(ref result.Value, 1) = scalar;
                Unsafe.Add(ref result.Value, 2) = scalar;
                Unsafe.Add(ref result.Value, 3) = scalar;
                Unsafe.Add(ref result.Value, 4) = scalar;
                Unsafe.Add(ref result.Value, 5) = scalar;
                Unsafe.Add(ref result.Value, 6) = scalar;
                Unsafe.Add(ref result.Value, 7) = scalar;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Broadcast(int scalar, out Wide<int> result)
        {
            result.Value = 0;
            if (Avx2.IsSupported)
            {
                To256(ref result) = Avx2.BroadcastScalarToVector256(&scalar);
            }
            else if (Sse42.IsSupported)
            {
                //TODO: Does Vector256.Create work without AVX?
                To128(ref result) = Vector128.Create(scalar);
                To128(ref Unsafe.Add(ref result.Value, 4)) = Vector128.Create(scalar);
            }
            else
            {
                result.Value = scalar;
                Unsafe.Add(ref result.Value, 1) = scalar;
                Unsafe.Add(ref result.Value, 2) = scalar;
                Unsafe.Add(ref result.Value, 3) = scalar;
                Unsafe.Add(ref result.Value, 4) = scalar;
                Unsafe.Add(ref result.Value, 5) = scalar;
                Unsafe.Add(ref result.Value, 6) = scalar;
                Unsafe.Add(ref result.Value, 7) = scalar;
            }
        }
    }
}
