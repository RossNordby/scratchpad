using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// Provides SIMD-aware 4x4 matrix math.
    /// </summary>
    public struct MatrixSIMD
    {
        //For now, call these columns.
        public Vector4 X;
        public Vector4 Y;
        public Vector4 Z;
        public Vector4 W;

        public static MatrixSIMD Identity
        {
            get
            {
                MatrixSIMD result;
                result.X = new Vector4(1, 0, 0, 0);
                result.Y = new Vector4(0, 1, 0, 0);
                result.Z = new Vector4(0, 0, 1, 0);
                result.W = new Vector4(0, 0, 0, 1);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformRowVector(ref Vector4 v, ref MatrixSIMD m, out Vector4 result)
        {
            //Assume row vector.
            //It multiplies across the columns.
            result = new Vector4(
                Vector4.Dot(v, m.X),
                Vector4.Dot(v, m.Y),
                Vector4.Dot(v, m.Z),
                Vector4.Dot(v, m.W));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformColumnVector(ref MatrixSIMD m, ref Vector4 v, out Vector4 result)
        {
            //Assume column vector.
            //It multiplies across the rows.
            var x = new Vector4(v.X);
            var y = new Vector4(v.Y);
            var z = new Vector4(v.Z);
            var w = new Vector4(v.W);
            result = m.X * x + m.Y * y + m.Z * z + m.W * w;
        }

    }
}
