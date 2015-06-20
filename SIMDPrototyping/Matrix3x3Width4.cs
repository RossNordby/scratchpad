using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public struct Matrix3x3Width4
    {
        public Vector4 M11;
        public Vector4 M12;
        public Vector4 M13;

        public Vector4 M21;
        public Vector4 M22;
        public Vector4 M23;

        public Vector4 M31;
        public Vector4 M32;
        public Vector4 M33;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3x3Width4(ref Matrix3x3 m1, ref Matrix3x3 m2, ref Matrix3x3 m3, ref Matrix3x3 m4)
        {
            M11 = new Vector4(m1.X.X, m2.X.X, m3.X.X, m4.X.X);
            M12 = new Vector4(m1.X.Y, m2.X.Y, m3.X.Y, m4.X.Y);
            M13 = new Vector4(m1.X.Z, m2.X.Z, m3.X.Z, m4.X.Z);

            M21 = new Vector4(m1.Y.X, m2.Y.X, m3.Y.X, m4.Y.X);
            M22 = new Vector4(m1.Y.Y, m2.Y.Y, m3.Y.Y, m4.Y.Y);
            M23 = new Vector4(m1.Y.Z, m2.Y.Z, m3.Y.Z, m4.Y.Z);

            M31 = new Vector4(m1.Z.X, m2.Z.X, m3.Z.X, m4.Z.X);
            M32 = new Vector4(m1.Z.Y, m2.Z.Y, m3.Z.Y, m4.Z.Y);
            M33 = new Vector4(m1.Z.Z, m2.Z.Z, m3.Z.Z, m4.Z.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3Width4 v, ref Matrix3x3Width4 m, out Vector3Width4 result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21 + v.Z * m.M31;
            result.Y = v.X * m.M12 + v.Y * m.M22 + v.Z * m.M32;
            result.Z = v.X * m.M13 + v.Y * m.M23 + v.Z * m.M33;
        }
    }
}
