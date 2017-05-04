using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Collections
{
    /// <summary>
    /// Provides optimized equality comparison and hashing for primitive types.
    /// </summary>
    /// <typeparam name="T">Type to compare and hash.</typeparam>
    public struct PrimitiveComparer<T> : IEqualityComparerRef<T>, IComparerRef<T>
    {
        //using T4 templates? pfah

        public int Compare(ref T a, ref T b)
        {
            if (typeof(T) == typeof(bool))
            {
                return Unsafe.As<T, bool>(ref a).CompareTo(Unsafe.As<T, bool>(ref b));
            }
            if (typeof(T) == typeof(byte))
            {
                return Unsafe.As<T, byte>(ref a).CompareTo(Unsafe.As<T, byte>(ref b));
            }
            if (typeof(T) == typeof(sbyte))
            {
                return Unsafe.As<T, sbyte>(ref a).CompareTo(Unsafe.As<T, sbyte>(ref b));
            }
            if (typeof(T) == typeof(short))
            {
                return Unsafe.As<T, short>(ref a).CompareTo(Unsafe.As<T, short>(ref b));
            }
            if (typeof(T) == typeof(ushort))
            {
                return Unsafe.As<T, ushort>(ref a).CompareTo(Unsafe.As<T, ushort>(ref b));
            }
            if (typeof(T) == typeof(int))
            {
                return Unsafe.As<T, int>(ref a).CompareTo(Unsafe.As<T, int>(ref b));
            }
            if (typeof(T) == typeof(uint))
            {
                return Unsafe.As<T, uint>(ref a).CompareTo(Unsafe.As<T, uint>(ref b));
            }
            if (typeof(T) == typeof(long))
            {
                return Unsafe.As<T, long>(ref a).CompareTo(Unsafe.As<T, long>(ref b));
            }
            if (typeof(T) == typeof(ulong))
            {
                return Unsafe.As<T, ulong>(ref a).CompareTo(Unsafe.As<T, ulong>(ref b));
            }
            if (typeof(T) == typeof(IntPtr))
            {
                unsafe
                {
                    var aTemp = Unsafe.As<T, IntPtr>(ref a).ToPointer();
                    var bTemp = Unsafe.As<T, IntPtr>(ref b).ToPointer();
                    return aTemp < bTemp ? -1 : aTemp > bTemp ? 1 : 0;
                }
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                unsafe
                {
                    var aTemp = Unsafe.As<T, UIntPtr>(ref a).ToPointer();
                    var bTemp = Unsafe.As<T, UIntPtr>(ref b).ToPointer();
                    return aTemp < bTemp ? -1 : aTemp > bTemp ? 1 : 0;
                }
            }
            if (typeof(T) == typeof(char))
            {
                return Unsafe.As<T, char>(ref a).CompareTo(Unsafe.As<T, char>(ref b));
            }
            if (typeof(T) == typeof(double))
            {
                return Unsafe.As<T, double>(ref a).CompareTo(Unsafe.As<T, double>(ref b));
            }
            if (typeof(T) == typeof(float))
            {
                return Unsafe.As<T, float>(ref a).CompareTo(Unsafe.As<T, float>(ref b));
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref T a, ref T b)
        {
            if (typeof(T) == typeof(bool))
            {
                return Unsafe.As<T, bool>(ref a) == Unsafe.As<T, bool>(ref b);
            }
            if (typeof(T) == typeof(byte))
            {
                return Unsafe.As<T, byte>(ref a) == Unsafe.As<T, byte>(ref b);
            }
            if (typeof(T) == typeof(sbyte))
            {
                return Unsafe.As<T, sbyte>(ref a) == Unsafe.As<T, sbyte>(ref b);
            }
            if (typeof(T) == typeof(short))
            {
                return Unsafe.As<T, short>(ref a) == Unsafe.As<T, short>(ref b);
            }
            if (typeof(T) == typeof(ushort))
            {
                return Unsafe.As<T, ushort>(ref a) == Unsafe.As<T, ushort>(ref b);
            }
            if (typeof(T) == typeof(int))
            {
                return Unsafe.As<T, int>(ref a) == Unsafe.As<T, int>(ref b);
            }
            if (typeof(T) == typeof(uint))
            {
                return Unsafe.As<T, uint>(ref a) == Unsafe.As<T, uint>(ref b);
            }
            if (typeof(T) == typeof(long))
            {
                return Unsafe.As<T, long>(ref a) == Unsafe.As<T, long>(ref b);
            }
            if (typeof(T) == typeof(ulong))
            {
                return Unsafe.As<T, ulong>(ref a) == Unsafe.As<T, ulong>(ref b);
            }
            if (typeof(T) == typeof(IntPtr))
            {
                return Unsafe.As<T, IntPtr>(ref a) == Unsafe.As<T, IntPtr>(ref b);
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                return Unsafe.As<T, UIntPtr>(ref a) == Unsafe.As<T, UIntPtr>(ref b);
            }
            if (typeof(T) == typeof(char))
            {
                return Unsafe.As<T, char>(ref a) == Unsafe.As<T, char>(ref b);
            }
            if (typeof(T) == typeof(double))
            {
                return Unsafe.As<T, double>(ref a) == Unsafe.As<T, double>(ref b);
            }
            if (typeof(T) == typeof(float))
            {
                return Unsafe.As<T, float>(ref a) == Unsafe.As<T, float>(ref b);
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref T item)
        {
            //Note: the jit is able to inline the GetHashCodes; no need for custom implementations.
            if (typeof(T) == typeof(bool))
            {
                return Unsafe.As<T, bool>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(byte))
            {
                return Unsafe.As<T, byte>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(sbyte))
            {
                return Unsafe.As<T, sbyte>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(short))
            {
                return Unsafe.As<T, short>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(ushort))
            {
                return Unsafe.As<T, ushort>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(int))
            {
                return Unsafe.As<T, int>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(uint))
            {
                return Unsafe.As<T, uint>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(long))
            {
                return Unsafe.As<T, long>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(ulong))
            {
                return Unsafe.As<T, ulong>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(IntPtr))
            {
                return Unsafe.As<T, IntPtr>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                return Unsafe.As<T, UIntPtr>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(char))
            {
                return Unsafe.As<T, char>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(double))
            {
                return Unsafe.As<T, double>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(float))
            {
                return Unsafe.As<T, float>(ref item).GetHashCode();
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return 0;
        }
    }
}
