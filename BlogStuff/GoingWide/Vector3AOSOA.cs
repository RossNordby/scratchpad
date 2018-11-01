using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3AOSOA
    {
        public ScalarWide X;
        public ScalarWide Y;
        public ScalarWide Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(Vector3AOSOA* a, Vector3AOSOA* b, Vector3AOSOA* result)
        {
            ScalarWide.Add(&a->X, &b->X, &result->X);
            ScalarWide.Add(&a->Y, &b->Y, &result->Y);
            ScalarWide.Add(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(Vector3AOSOA* a, Vector3AOSOA* b, Vector3AOSOA* result)
        {
            ScalarWide.Subtract(&a->X, &b->X, &result->X);
            ScalarWide.Subtract(&a->Y, &b->Y, &result->Y);
            ScalarWide.Subtract(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(Vector3AOSOA* a, Vector3AOSOA* b, Vector3AOSOA* result)
        {
            Vector3AOSOA left, right;
            ScalarWide.Multiply(&a->Y, &b->Z, &left.X);
            ScalarWide.Multiply(&a->Z, &b->X, &left.Y);
            ScalarWide.Multiply(&a->X, &b->Y, &left.Z);
            ScalarWide.Multiply(&a->Z, &b->Y, &right.X);
            ScalarWide.Multiply(&a->X, &b->Z, &right.Y);
            ScalarWide.Multiply(&a->Y, &b->X, &right.Z);
            Subtract(&left, &right, result);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(Vector3AOSOA* a, Vector3AOSOA* b, ScalarWide* result)
        {
            ScalarWide x, y, z;
            ScalarWide.Multiply(&a->X, &b->X, &x);
            ScalarWide.Multiply(&a->Y, &b->Y, &y);
            ScalarWide.Multiply(&a->Z, &b->Z, &z);
            ScalarWide.Add(&x, &y, result);
            ScalarWide.Add(result, &z, result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(Vector3AOSOA* v, ScalarWide* scale, Vector3AOSOA* result)
        {
            ScalarWide.Multiply(&v->X, scale, &result->X);
            ScalarWide.Multiply(&v->Y, scale, &result->Y);
            ScalarWide.Multiply(&v->Z, scale, &result->Z);
        }

    }
}
