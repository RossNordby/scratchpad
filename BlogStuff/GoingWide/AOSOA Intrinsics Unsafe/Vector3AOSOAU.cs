using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3AOSOAU
    {
        public ScalarWideU X;
        public ScalarWideU Y;
        public ScalarWideU Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(Vector3AOSOAU* a, Vector3AOSOAU* b, Vector3AOSOAU* result)
        {
            ScalarWideU.Add(&a->X, &b->X, &result->X);
            ScalarWideU.Add(&a->Y, &b->Y, &result->Y);
            ScalarWideU.Add(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(Vector3AOSOAU* a, Vector3AOSOAU* b, Vector3AOSOAU* result)
        {
            ScalarWideU.Subtract(&a->X, &b->X, &result->X);
            ScalarWideU.Subtract(&a->Y, &b->Y, &result->Y);
            ScalarWideU.Subtract(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(Vector3AOSOAU* a, Vector3AOSOAU* b, Vector3AOSOAU* result)
        {
            Vector3AOSOAU left, right;
            ScalarWideU.Multiply(&a->Y, &b->Z, &left.X);
            ScalarWideU.Multiply(&a->Z, &b->X, &left.Y);
            ScalarWideU.Multiply(&a->X, &b->Y, &left.Z);
            ScalarWideU.Multiply(&a->Z, &b->Y, &right.X);
            ScalarWideU.Multiply(&a->X, &b->Z, &right.Y);
            ScalarWideU.Multiply(&a->Y, &b->X, &right.Z);
            Subtract(&left, &right, result);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(Vector3AOSOAU* a, Vector3AOSOAU* b, ScalarWideU* result)
        {
            ScalarWideU x, y, z;
            ScalarWideU.Multiply(&a->X, &b->X, &x);
            ScalarWideU.Multiply(&a->Y, &b->Y, &y);
            ScalarWideU.Multiply(&a->Z, &b->Z, &z);
            ScalarWideU.Add(&x, &y, result);
            ScalarWideU.Add(result, &z, result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(Vector3AOSOAU* v, ScalarWideU* scale, Vector3AOSOAU* result)
        {
            ScalarWideU.Multiply(&v->X, scale, &result->X);
            ScalarWideU.Multiply(&v->Y, scale, &result->Y);
            ScalarWideU.Multiply(&v->Z, scale, &result->Z);
        }

    }
}
