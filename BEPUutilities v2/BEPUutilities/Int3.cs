using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2
{
    /// <summary>
    /// A set of 3 integers, useful for spatial hashing.
    /// </summary>
    public struct Int3 : IEquatable<Int3>
    {
        public int X;
        public int Y;
        public int Z;

        public unsafe override int GetHashCode()
        {
            //int result = X;
            //result = 31 * result + Y;
            //result = 31 * result + Z;
            //return result;

            //const long p1 = 961748927L;
            //const long p2 = 899809343L;
            //const long p3 = 715225741L;
            ////const long p1 = (1L << 62) - 57L;
            ////const long p2 = (1L << 61) - 259L;
            ////const long p3 = (1L << 60) - 107L;
            ////const long mod = (1L << 53) - 111L;
            ////return (int)((X * p1) ^ (Y * p2) ^ (Z * p3));
            //return (int)((((X * p1 + Y) * p2 + Z) * p3));


            //const uint p1 = 961748927u;
            //const uint p2 = 899809343u;
            //const uint p3 = 715225741u;
            //const long p1 = (1L << 62) - 57L;
            //const long p2 = (1L << 61) - 259L;
            //const long p3 = (1L << 60) - 107L;
            //const long mod = (1L << 53) - 111L;
            //return (int)((X * p1) ^ (Y * p2) ^ (Z * p3));
            //return (int)(((((uint)X * p1 + (uint)Y) * p2 + (uint)Z) * p3));

            //const ulong p1 = 961748927UL;
            //const ulong p2 = 899809343UL;
            //const ulong p3 = 715225741UL;
            //return (int)((ulong)X * unchecked(p1 * p2 * p3) + (ulong)Y * (p2 * p3) + (ulong)Z * p3);

            const ulong p1 = 961748927UL;
            const ulong p2 = 899809343UL;
            const ulong p3 = 715225741UL;
            var hash64 = (ulong)X * unchecked(p1 * p2 * p3) + (ulong)Y * (p2 * p3) + (ulong)Z * p3;
            return (int)(hash64 ^ (hash64 >> 32));

            //var a = 6;
            //var b = 13;
            //var c = 25;
            //var ux = (uint)X;
            //var uy = (uint)Y;
            //var uz = (uint)Z;
            //var scrambled =
            //    ((ux << a) | (ux >> (32 - a))) ^
            //    ((uy << b) | (uy >> (32 - b))) ^
            //    ((uz << c) | (uz >> (32 - c)));
            //return (int)scrambled;

            //const int a = 6;
            //const int b = 13;
            //const int c = 25;
            //var scrambled =
            //    (((uint)(X << a)) | (((uint)X) >> (32 - a))) ^
            //    (((uint)(Y << b)) | (((uint)Y) >> (32 - b))) ^
            //    (((uint)(Z << c)) | (((uint)Z) >> (32 - c)));
            //return (int)scrambled;

            //const int a = 6;
            //const int b = 13;
            //const int c = 25;
            //var upperA = (uint)(X << a);
            //var upperB = (uint)(Y << b);
            //var upperC = (uint)(Z << c);
            //var lowerA = ((uint)X) >> (32 - a);
            //var lowerB = ((uint)Y) >> (32 - b);
            //var lowerC = ((uint)Z) >> (32 - c);
            //var mixedA = upperA | lowerA;
            //var mixedB = upperB | lowerB;
            //var mixedC = upperC | lowerC;
            //return (int)(mixedA ^ mixedB ^ mixedC);

            //const int a = 6;
            //const int b = 13;
            //const int c = 25;
            //const uint pa = 961748927u;
            //const uint pb = 899809343u;
            //const uint pc = 715225741u;
            //var upperA = (uint)(X << a);
            //var upperB = (uint)(Y << b);
            //var upperC = (uint)(Z << c);
            //var lowerA = ((uint)X) >> (32 - a);
            //var lowerB = ((uint)Y) >> (32 - b);
            //var lowerC = ((uint)Z) >> (32 - c);
            //var mixedA = pa * (upperA | lowerA);
            //var mixedB = pb * (upperB | lowerB);
            //var mixedC = pc * (upperC | lowerC);
            //return (int)(mixedA ^ mixedB ^ mixedC);

            //const int a = 6;
            //const int b = 13;
            //const int c = 25;
            //const uint pa = 961748927u;
            //const uint pb = 899809343u;
            //const uint pc = 715225741u;
            //var ux = (uint)X * pa;
            //var uy = (uint)Y * pb;
            //var uz = (uint)Z * pc;
            //var upperA = (ux << a);
            //var upperB = (uy << b);
            //var upperC = (uz << c);
            //var lowerA = (ux) >> (32 - a);
            //var lowerB = (uy) >> (32 - b);
            //var lowerC = (uz) >> (32 - c);
            //var mixedA = (upperA | lowerA);
            //var mixedB = (upperB | lowerB);
            //var mixedC = (upperC | lowerC);
            //return (int)(mixedA ^ mixedB ^ mixedC);

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
