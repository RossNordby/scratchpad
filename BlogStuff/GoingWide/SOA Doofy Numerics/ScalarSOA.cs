using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace GoingWide
{
    public struct ScalarSOA
    {
        public Buffer<float> Values;

        [Conditional("DEBUG")]
        static void Validate(in ScalarSOA a, in ScalarSOA b, in ScalarSOA result)
        {
            Debug.Assert(result.Values.Length >= a.Values.Length && result.Values.Length >= b.Values.Length && (result.Values.Length & (Vector<float>.Count - 1)) == 0,
                "We assume that the result is bundle divisible and all inputs are large enough to feed it.");
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref ScalarSOA a, ref ScalarSOA b, ref ScalarSOA result)
        {
            Validate(a, b, result);
            var lengthInBundles = result.Values.Length / Vector<float>.Count;
            ref var aBundleBase = ref Unsafe.As<float, Vector<float>>(ref a.Values[0]);
            ref var bBundleBase = ref Unsafe.As<float, Vector<float>>(ref b.Values[0]);
            ref var resultBundleBase = ref Unsafe.As<float, Vector<float>>(ref result.Values[0]);
            for (int i = 0; i < lengthInBundles; ++i)
            {
                Unsafe.Add(ref resultBundleBase, i) = Unsafe.Add(ref aBundleBase, i) + Unsafe.Add(ref bBundleBase, i);
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref ScalarSOA a, ref ScalarSOA b, ref ScalarSOA result)
        {
            Validate(a, b, result);
            var lengthInBundles = result.Values.Length / Vector<float>.Count;
            ref var aBundleBase = ref Unsafe.As<float, Vector<float>>(ref a.Values[0]);
            ref var bBundleBase = ref Unsafe.As<float, Vector<float>>(ref b.Values[0]);
            ref var resultBundleBase = ref Unsafe.As<float, Vector<float>>(ref result.Values[0]);
            for (int i = 0; i < lengthInBundles; ++i)
            {
                Unsafe.Add(ref resultBundleBase, i) = Unsafe.Add(ref aBundleBase, i) - Unsafe.Add(ref bBundleBase, i);
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref ScalarSOA a, ref ScalarSOA b, ref ScalarSOA result)
        {
            Validate(a, b, result);
            var lengthInBundles = result.Values.Length / Vector<float>.Count;
            ref var aBundleBase = ref Unsafe.As<float, Vector<float>>(ref a.Values[0]);
            ref var bBundleBase = ref Unsafe.As<float, Vector<float>>(ref b.Values[0]);
            ref var resultBundleBase = ref Unsafe.As<float, Vector<float>>(ref result.Values[0]);
            for (int i = 0; i < lengthInBundles; ++i)
            {
                Unsafe.Add(ref resultBundleBase, i) = Unsafe.Add(ref aBundleBase, i) * Unsafe.Add(ref bBundleBase, i);
            }
        }
    }
}
