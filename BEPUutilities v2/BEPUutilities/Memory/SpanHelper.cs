using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BEPUutilities2.Memory
{
    public static class SpanHelper
    {
        /// <summary>
        /// Tests if a generic parameter is primitive.
        /// </summary>
        /// <typeparam name="T">Type to check for primitiveness.</typeparam>
        /// <returns>True if the type is one of the primitive types, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPrimitive<T>()
        {
            //TODO: Test for jit specialization.
            return IsPrimitive(typeof(T));
        }
        /// <summary>
        /// Tests if a type is primitive.
        /// </summary>
        /// <param name="type">Type to check for primitiveness.</typeparam>
        /// <returns>True if the type is one of the primitive types, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPrimitive(Type type)
        {
            return
                type == typeof(bool) ||
                type == typeof(byte) ||
                type == typeof(sbyte) ||
                type == typeof(ushort) ||
                type == typeof(short) ||
                type == typeof(uint) ||
                type == typeof(int) ||
                type == typeof(ulong) ||
                type == typeof(long) ||
                type == typeof(IntPtr) ||
                type == typeof(UIntPtr) ||
                type == typeof(char) ||
                type == typeof(double) ||
                type == typeof(float);
        }

        [Conditional("DEBUG")]
        private static void Validate<T, TSourceSpan, TTargetSpan>(ref TSourceSpan source, int sourceIndex, ref TTargetSpan target, int targetIndex, int count)
            where TSourceSpan : ISpan<T>
            where TTargetSpan : ISpan<T>
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref PointerSpan<T> source, int sourceIndex, ref PointerSpan<T> target, int targetIndex, int count)
        {
            Validate<T, PointerSpan<T>, PointerSpan<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                source.Memory + sourceIndex * Unsafe.SizeOf<T>(),
                target.Memory + targetIndex * Unsafe.SizeOf<T>(),
                byteCount, byteCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref ManagedSpan<T> source, int sourceIndex, ref ManagedSpan<T> target, int targetIndex, int count)
        {
            Validate<T, ManagedSpan<T>, ManagedSpan<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var byteCount = count * Unsafe.SizeOf<T>();
            Array.Copy(source.Array, sourceIndex, target.Array, targetIndex, count);
        }

        //Copies between spans of different types should be extremely rare in practice.
        //Due to that rareness, and the complexity involved in the overlap-isn't-a-problem guarantees of the function, just bite the bullet and pin.
        //This will have slightly worse performance, but it doesn't matter much.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref PointerSpan<T> source, int sourceIndex, ref ManagedSpan<T> target, int targetIndex, int count)
        {
            Validate<T, PointerSpan<T>, ManagedSpan<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var arrayHandle = GCHandle.Alloc(target.Array, GCHandleType.Pinned);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                source.Memory + sourceIndex * Unsafe.SizeOf<T>(),
                Unsafe.AsPointer(ref target[targetIndex]),
                byteCount, byteCount);
            arrayHandle.Free();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref ManagedSpan<T> source, int sourceIndex, ref PointerSpan<T> target, int targetIndex, int count)
        {
            Validate<T, ManagedSpan<T>, PointerSpan<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var arrayHandle = GCHandle.Alloc(source.Array, GCHandleType.Pinned);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                Unsafe.AsPointer(ref source[sourceIndex]),
                target.Memory + targetIndex * Unsafe.SizeOf<T>(),
                byteCount, byteCount);
            arrayHandle.Free();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void CopyFallback<T, TSourceSpan, TTargetSpan>(ref TSourceSpan source, int sourceIndex, ref TTargetSpan target, int targetIndex, int count)
            where TSourceSpan : ISpan<T>
            where TTargetSpan : ISpan<T>
        {
            Validate<T, TSourceSpan, TTargetSpan>(ref source, sourceIndex, ref target, targetIndex, count);
            var sourcePinned = source.TryPin(out var sourceHandle);
            var targetPinned = target.TryPin(out var targetHandle);

            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                Unsafe.AsPointer(ref source[sourceIndex]),
                Unsafe.AsPointer(ref target[targetIndex]),
                byteCount, byteCount);

            if (sourcePinned)
                sourceHandle.Free();
            if (targetPinned)
                targetHandle.Free();
        }


    }
}
