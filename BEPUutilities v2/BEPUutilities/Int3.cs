using System;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// A set of 3 integers, useful for spatial hashing.
    /// </summary>
    public struct Int3 : IEquatable<Int3>
    {
        public int X;
        public int Y;
        public int Z;

        public override int GetHashCode()
        {
            const long p1 = 961748927L;
            const long p2 = 961748941L;
            const long p3 = 982451653L;
            return (int)((X * p1) ^ (Y * p2) ^ (Z * p3));
        }

        public override bool Equals(object obj)
        {
            return this.Equals((Int3)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Int3 other)
        {
            return other.X == X && other.Y == Y && other.Z == Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Int3 lhs, Int3 rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y && lhs.Z == rhs.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Int3 lhs, Int3 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y || lhs.Z != rhs.Z;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"{{{X}, {Y}, {Z}}}";
        }
    }

}
