using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
namespace BepuScatter.Tracing
{
    public static class VectorCasts
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector256<TTo> To256<TFrom, TTo>(ref TFrom v) where TFrom : unmanaged where TTo : unmanaged
        {
            return ref Unsafe.As<TFrom, Vector256<TTo>>(ref v);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector256<T> To256<T>(ref T v) where T : unmanaged
        {
            return ref Unsafe.As<T, Vector256<T>>(ref v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector128<TTo> To128<TFrom, TTo>(ref TFrom v) where TFrom : unmanaged where TTo : unmanaged
        {
            return ref Unsafe.As<TFrom, Vector128<TTo>>(ref v);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector128<T> To128<T>(ref T v) where T : unmanaged
        {
            return ref Unsafe.As<T, Vector128<T>>(ref v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector256<TTo> To256<TFrom, TTo>(ref Wide<TFrom> v) where TFrom : unmanaged where TTo : unmanaged
        {
            return ref Unsafe.As<TFrom, Vector256<TTo>>(ref v.Value);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector256<T> To256<T>(ref Wide<T> v) where T : unmanaged
        {
            return ref Unsafe.As<T, Vector256<T>>(ref v.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector128<TTo> To128<TFrom, TTo>(ref Wide<TFrom> v) where TFrom : unmanaged where TTo : unmanaged
        {
            return ref Unsafe.As<TFrom, Vector128<TTo>>(ref v.Value);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector128<T> To128<T>(ref Wide<T> v) where T : unmanaged
        {
            return ref Unsafe.As<T, Vector128<T>>(ref v.Value);
        }
    }
}
