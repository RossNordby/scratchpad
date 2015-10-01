using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct Matrix3x3
    {
        [FieldOffset(0)]
        public Vector3 X;
        [FieldOffset(16)]
        public Vector3 Y;
        [FieldOffset(32)]
        public Vector3 Z;



        public static void Get(out Matrix3x3 m)
        {
            m = new Matrix3x3();
            m.X = new Vector3();
            m.Y = new Vector3();
            m.Z = new Vector3();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Transform(Vector3 v, Matrix3x3 m)
        {
            var x = new Vector3(v.X);
            var y = new Vector3(v.Y);
            var z = new Vector3(v.Z);

            return m.X * x + m.Y * y + m.Z * z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3 v, ref Matrix3x3 m, out Vector3 result)
        {
            var x = new Vector3(v.X);
            var y = new Vector3(v.Y);
            var z = new Vector3(v.Z);

            result = m.X * x + m.Y * y + m.Z * z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Transform2(Vector3 v, Matrix3x3 m)
        {
            var x = new Vector3(m.X.X, m.Y.X, m.Z.X);
            var y = new Vector3(m.X.Y, m.Y.Y, m.Z.Y);
            var z = new Vector3(m.X.Z, m.Y.Z, m.Z.Z);

            return new Vector3(Vector3.Dot(v, x), Vector3.Dot(v, y), Vector3.Dot(v, z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform2(ref Vector3 v, ref Matrix3x3 m, out Vector3 result)
        {
            var x = new Vector3(m.X.X, m.Y.X, m.Z.X);
            var y = new Vector3(m.X.Y, m.Y.Y, m.Z.Y);
            var z = new Vector3(m.X.Z, m.Y.Z, m.Z.Z);

            result = new Vector3(Vector3.Dot(v, x), Vector3.Dot(v, y), Vector3.Dot(v, z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 TransformTranspose(Vector3 v, Matrix3x3 m)
        {
            return new Vector3(Vector3.Dot(v, m.X), Vector3.Dot(v, m.Y), Vector3.Dot(v, m.Z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformTranspose(ref Vector3 v, ref Matrix3x3 m, out Vector3 result)
        {
            result = new Vector3(Vector3.Dot(v, m.X), Vector3.Dot(v, m.Y), Vector3.Dot(v, m.Z));
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public Vector3 TransformTranspose(Vector3 v, Matrix3x3 m)
        //{
        //    return new Vector3(Vector3.Dot(v, m.X), Vector3.Dot(v, m.Y), Vector3.Dot(v, m.Z));
        //}


    }
}
