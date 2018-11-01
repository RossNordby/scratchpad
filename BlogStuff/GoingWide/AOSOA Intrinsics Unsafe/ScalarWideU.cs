using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    [StructLayout(LayoutKind.Explicit, Size = BundleSize * 4)]
    public unsafe struct ScalarWideU
    {
        public const int BundleSize = 8;
        const bool AllowAVX = true;
        const bool AllowSSE = true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ScalarWideU* aBundle, ScalarWideU* bBundle, ScalarWideU* resultBundle)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector256<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector256<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector256<float>>(resultBundle);
                r = Avx.Add(a, b);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 1) = Avx.Add(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector128<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector128<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector128<float>>(resultBundle);
                r = Sse.Add(a, b);
                Unsafe.Add(ref r, 1) = Sse.Add(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 2) = Sse.Add(Unsafe.Add(ref a, 2), Unsafe.Add(ref b, 2));
                    Unsafe.Add(ref r, 3) = Sse.Add(Unsafe.Add(ref a, 3), Unsafe.Add(ref b, 3));
                }
            }
            else
            {
                ref var a = ref Unsafe.AsRef<float>(aBundle);
                ref var b = ref Unsafe.AsRef<float>(bBundle);
                ref var r = ref Unsafe.AsRef<float>(resultBundle);
                r = a + b;
                Unsafe.Add(ref r, 1) = Unsafe.Add(ref a, 1) + Unsafe.Add(ref b, 1);
                Unsafe.Add(ref r, 2) = Unsafe.Add(ref a, 2) + Unsafe.Add(ref b, 2);
                Unsafe.Add(ref r, 3) = Unsafe.Add(ref a, 3) + Unsafe.Add(ref b, 3);
                Unsafe.Add(ref r, 4) = Unsafe.Add(ref a, 4) + Unsafe.Add(ref b, 4);
                Unsafe.Add(ref r, 5) = Unsafe.Add(ref a, 5) + Unsafe.Add(ref b, 5);
                Unsafe.Add(ref r, 6) = Unsafe.Add(ref a, 6) + Unsafe.Add(ref b, 6);
                Unsafe.Add(ref r, 7) = Unsafe.Add(ref a, 7) + Unsafe.Add(ref b, 7);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 8) = Unsafe.Add(ref a, 8) + Unsafe.Add(ref b, 8);
                    Unsafe.Add(ref r, 9) = Unsafe.Add(ref a, 9) + Unsafe.Add(ref b, 9);
                    Unsafe.Add(ref r, 10) = Unsafe.Add(ref a, 10) + Unsafe.Add(ref b, 10);
                    Unsafe.Add(ref r, 11) = Unsafe.Add(ref a, 11) + Unsafe.Add(ref b, 11);
                    Unsafe.Add(ref r, 12) = Unsafe.Add(ref a, 12) + Unsafe.Add(ref b, 12);
                    Unsafe.Add(ref r, 13) = Unsafe.Add(ref a, 13) + Unsafe.Add(ref b, 13);
                    Unsafe.Add(ref r, 14) = Unsafe.Add(ref a, 14) + Unsafe.Add(ref b, 14);
                    Unsafe.Add(ref r, 15) = Unsafe.Add(ref a, 15) + Unsafe.Add(ref b, 15);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ScalarWideU* aBundle, ScalarWideU* bBundle, ScalarWideU* resultBundle)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector256<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector256<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector256<float>>(resultBundle);
                r = Avx.Subtract(a, b);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 1) = Avx.Subtract(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector128<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector128<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector128<float>>(resultBundle);
                r = Sse.Subtract(a, b);
                Unsafe.Add(ref r, 1) = Sse.Subtract(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 2) = Sse.Subtract(Unsafe.Add(ref a, 2), Unsafe.Add(ref b, 2));
                    Unsafe.Add(ref r, 3) = Sse.Subtract(Unsafe.Add(ref a, 3), Unsafe.Add(ref b, 3));
                }
            }
            else
            {
                ref var a = ref Unsafe.AsRef<float>(aBundle);
                ref var b = ref Unsafe.AsRef<float>(bBundle);
                ref var r = ref Unsafe.AsRef<float>(resultBundle);
                r = a - b;
                Unsafe.Add(ref r, 1) = Unsafe.Add(ref a, 1) - Unsafe.Add(ref b, 1);
                Unsafe.Add(ref r, 2) = Unsafe.Add(ref a, 2) - Unsafe.Add(ref b, 2);
                Unsafe.Add(ref r, 3) = Unsafe.Add(ref a, 3) - Unsafe.Add(ref b, 3);
                Unsafe.Add(ref r, 4) = Unsafe.Add(ref a, 4) - Unsafe.Add(ref b, 4);
                Unsafe.Add(ref r, 5) = Unsafe.Add(ref a, 5) - Unsafe.Add(ref b, 5);
                Unsafe.Add(ref r, 6) = Unsafe.Add(ref a, 6) - Unsafe.Add(ref b, 6);
                Unsafe.Add(ref r, 7) = Unsafe.Add(ref a, 7) - Unsafe.Add(ref b, 7);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 8) = Unsafe.Add(ref a, 8) - Unsafe.Add(ref b, 8);
                    Unsafe.Add(ref r, 9) = Unsafe.Add(ref a, 9) - Unsafe.Add(ref b, 9);
                    Unsafe.Add(ref r, 10) = Unsafe.Add(ref a, 10) - Unsafe.Add(ref b, 10);
                    Unsafe.Add(ref r, 11) = Unsafe.Add(ref a, 11) - Unsafe.Add(ref b, 11);
                    Unsafe.Add(ref r, 12) = Unsafe.Add(ref a, 12) - Unsafe.Add(ref b, 12);
                    Unsafe.Add(ref r, 13) = Unsafe.Add(ref a, 13) - Unsafe.Add(ref b, 13);
                    Unsafe.Add(ref r, 14) = Unsafe.Add(ref a, 14) - Unsafe.Add(ref b, 14);
                    Unsafe.Add(ref r, 15) = Unsafe.Add(ref a, 15) - Unsafe.Add(ref b, 15);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ScalarWideU* aBundle, ScalarWideU* bBundle, ScalarWideU* resultBundle)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector256<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector256<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector256<float>>(resultBundle);
                r = Avx.Multiply(a, b);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 1) = Avx.Multiply(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                ref var a = ref Unsafe.AsRef<Vector128<float>>(aBundle);
                ref var b = ref Unsafe.AsRef<Vector128<float>>(bBundle);
                ref var r = ref Unsafe.AsRef<Vector128<float>>(resultBundle);
                r = Sse.Subtract(a, b);
                Unsafe.Add(ref r, 1) = Sse.Multiply(Unsafe.Add(ref a, 1), Unsafe.Add(ref b, 1));
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 2) = Sse.Multiply(Unsafe.Add(ref a, 2), Unsafe.Add(ref b, 2));
                    Unsafe.Add(ref r, 3) = Sse.Multiply(Unsafe.Add(ref a, 3), Unsafe.Add(ref b, 3));
                }
            }
            else
            {
                ref var a = ref Unsafe.AsRef<float>(aBundle);
                ref var b = ref Unsafe.AsRef<float>(bBundle);
                ref var r = ref Unsafe.AsRef<float>(resultBundle);
                r = a * b;
                Unsafe.Add(ref r, 1) = Unsafe.Add(ref a, 1) * Unsafe.Add(ref b, 1);
                Unsafe.Add(ref r, 2) = Unsafe.Add(ref a, 2) * Unsafe.Add(ref b, 2);
                Unsafe.Add(ref r, 3) = Unsafe.Add(ref a, 3) * Unsafe.Add(ref b, 3);
                Unsafe.Add(ref r, 4) = Unsafe.Add(ref a, 4) * Unsafe.Add(ref b, 4);
                Unsafe.Add(ref r, 5) = Unsafe.Add(ref a, 5) * Unsafe.Add(ref b, 5);
                Unsafe.Add(ref r, 6) = Unsafe.Add(ref a, 6) * Unsafe.Add(ref b, 6);
                Unsafe.Add(ref r, 7) = Unsafe.Add(ref a, 7) * Unsafe.Add(ref b, 7);
                if (BundleSize == 16)
                {
                    Unsafe.Add(ref r, 8) = Unsafe.Add(ref a, 8) * Unsafe.Add(ref b, 8);
                    Unsafe.Add(ref r, 9) = Unsafe.Add(ref a, 9) * Unsafe.Add(ref b, 9);
                    Unsafe.Add(ref r, 10) = Unsafe.Add(ref a, 10) * Unsafe.Add(ref b, 10);
                    Unsafe.Add(ref r, 11) = Unsafe.Add(ref a, 11) * Unsafe.Add(ref b, 11);
                    Unsafe.Add(ref r, 12) = Unsafe.Add(ref a, 12) * Unsafe.Add(ref b, 12);
                    Unsafe.Add(ref r, 13) = Unsafe.Add(ref a, 13) * Unsafe.Add(ref b, 13);
                    Unsafe.Add(ref r, 14) = Unsafe.Add(ref a, 14) * Unsafe.Add(ref b, 14);
                    Unsafe.Add(ref r, 15) = Unsafe.Add(ref a, 15) * Unsafe.Add(ref b, 15);
                }
            }
        }
    }
}
