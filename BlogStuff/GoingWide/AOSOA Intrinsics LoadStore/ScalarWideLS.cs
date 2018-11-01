using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    [StructLayout(LayoutKind.Explicit, Size = BundleSize * 4)]
    public unsafe struct ScalarWideLS
    {
        public const int BundleSize = 8;
        const bool AllowAVX = true;
        const bool AllowSSE = true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ScalarWideLS* a, ScalarWideLS* b, ScalarWideLS* result)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                Avx.Store((float*)result, Avx.Add(Avx.LoadVector256((float*)a), Avx.LoadVector256((float*)b)));
                if (BundleSize == 16)
                {
                    Avx.Store((float*)result + 8, Avx.Add(Avx.LoadVector256((float*)a + 8), Avx.LoadVector256((float*)b + 8)));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                Sse.Store((float*)result, Sse.Add(Sse.LoadVector128((float*)a), Sse.LoadVector128((float*)b)));
                Sse.Store((float*)result + 4, Sse.Add(Sse.LoadVector128((float*)a + 4), Sse.LoadVector128((float*)b + 4)));
                if (BundleSize == 16)
                {
                    Sse.Store((float*)result + 8, Sse.Add(Sse.LoadVector128((float*)a + 8), Sse.LoadVector128((float*)b + 8)));
                    Sse.Store((float*)result + 12, Sse.Add(Sse.LoadVector128((float*)a + 12), Sse.LoadVector128((float*)b + 12)));
                }
            }
            else
            {
                var af = (float*)a;
                var bf = (float*)b;
                var rf = (float*)result;
                *rf = *af + *bf;
                rf[1] = af[1] + bf[1];
                rf[2] = af[2] + bf[2];
                rf[3] = af[3] + bf[3];
                rf[4] = af[4] + bf[4];
                rf[5] = af[5] + bf[5];
                rf[6] = af[6] + bf[6];
                rf[7] = af[7] + bf[7];
                if (BundleSize == 16)
                {
                    rf[8] = af[8] + bf[8];
                    rf[9] = af[9] + bf[9];
                    rf[10] = af[10] + bf[10];
                    rf[11] = af[11] + bf[11];
                    rf[12] = af[12] + bf[12];
                    rf[13] = af[13] + bf[13];
                    rf[14] = af[14] + bf[14];
                    rf[15] = af[15] + bf[15];
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ScalarWideLS* a, ScalarWideLS* b, ScalarWideLS* result)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                Avx.Store((float*)result, Avx.Subtract(Avx.LoadVector256((float*)a), Avx.LoadVector256((float*)b)));
                if (BundleSize == 16)
                {
                    Avx.Store((float*)result + 8, Avx.Subtract(Avx.LoadVector256((float*)a + 8), Avx.LoadVector256((float*)b + 8)));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                Sse.Store((float*)result, Sse.Subtract(Sse.LoadVector128((float*)a), Sse.LoadVector128((float*)b)));
                Sse.Store((float*)result + 4, Sse.Subtract(Sse.LoadVector128((float*)a + 4), Sse.LoadVector128((float*)b + 4)));
                if (BundleSize == 16)
                {
                    Sse.Store((float*)result + 8, Sse.Subtract(Sse.LoadVector128((float*)a + 8), Sse.LoadVector128((float*)b + 8)));
                    Sse.Store((float*)result + 12, Sse.Subtract(Sse.LoadVector128((float*)a + 12), Sse.LoadVector128((float*)b + 12)));
                }
            }
            else
            {
                var af = (float*)a;
                var bf = (float*)b;
                var rf = (float*)result;
                *rf = *af - *bf;
                rf[1] = af[1] - bf[1];
                rf[2] = af[2] - bf[2];
                rf[3] = af[3] - bf[3];
                rf[4] = af[4] - bf[4];
                rf[5] = af[5] - bf[5];
                rf[6] = af[6] - bf[6];
                rf[7] = af[7] - bf[7];
                if (BundleSize == 16)
                {
                    rf[8] = af[8] - bf[8];
                    rf[9] = af[9] - bf[9];
                    rf[10] = af[10] - bf[10];
                    rf[11] = af[11] - bf[11];
                    rf[12] = af[12] - bf[12];
                    rf[13] = af[13] - bf[13];
                    rf[14] = af[14] - bf[14];
                    rf[15] = af[15] - bf[15];
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ScalarWideLS* a, ScalarWideLS* b, ScalarWideLS* result)
        {
            if (AllowAVX && Avx.IsSupported)
            {
                Avx.Store((float*)result, Avx.Multiply(Avx.LoadVector256((float*)a), Avx.LoadVector256((float*)b)));
                if (BundleSize == 16)
                {
                    Avx.Store((float*)result + 8, Avx.Multiply(Avx.LoadVector256((float*)a + 8), Avx.LoadVector256((float*)b + 8)));
                }
            }
            else if (AllowSSE && Sse.IsSupported)
            {
                Sse.Store((float*)result, Sse.Multiply(Sse.LoadVector128((float*)a), Sse.LoadVector128((float*)b)));
                Sse.Store((float*)result + 4, Sse.Multiply(Sse.LoadVector128((float*)a + 4), Sse.LoadVector128((float*)b + 4)));
                if (BundleSize == 16)
                {
                    Sse.Store((float*)result + 8, Sse.Multiply(Sse.LoadVector128((float*)a + 8), Sse.LoadVector128((float*)b + 8)));
                    Sse.Store((float*)result + 12, Sse.Multiply(Sse.LoadVector128((float*)a + 12), Sse.LoadVector128((float*)b + 12)));
                }
            }
            else
            {
                var af = (float*)a;
                var bf = (float*)b;
                var rf = (float*)result;
                *rf = *af * *bf;
                rf[1] = af[1] * bf[1];
                rf[2] = af[2] * bf[2];
                rf[3] = af[3] * bf[3];
                rf[4] = af[4] * bf[4];
                rf[5] = af[5] * bf[5];
                rf[6] = af[6] * bf[6];
                rf[7] = af[7] * bf[7];
                if (BundleSize == 16)
                {
                    rf[8] = af[8] * bf[8];
                    rf[9] = af[9] * bf[9];
                    rf[10] = af[10] * bf[10];
                    rf[11] = af[11] * bf[11];
                    rf[12] = af[12] * bf[12];
                    rf[13] = af[13] * bf[13];
                    rf[14] = af[14] * bf[14];
                    rf[15] = af[15] * bf[15];
                }
            }
        }
    }
}
