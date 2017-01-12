using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{

    public struct Matrix3x3Wide
    {
        public Vector<float> M11;
        public Vector<float> M12;
        public Vector<float> M13;
        public Vector<float> M21;
        public Vector<float> M22;
        public Vector<float> M23;
        public Vector<float> M31;
        public Vector<float> M32;
        public Vector<float> M33;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector3Wide v, ref Matrix3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21 + v.Z * m.M31;
            result.Y = v.X * m.M12 + v.Y * m.M22 + v.Z * m.M32;
            result.Z = v.X * m.M13 + v.Y * m.M23 + v.Z * m.M33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3Wide v, ref Matrix3x3Wide m, out Vector3Wide result)
        {
            TransformWithoutOverlap(ref v, ref m, out var temp);
            result = temp;
        }

    }
}
