﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct Matrix3x3Wide
    {
        /// <summary>
        /// First row of the matrix.
        /// </summary>
        public Vector3Wide X;
        /// <summary>
        /// Second row of the matrix.
        /// </summary>
        public Vector3Wide Y;
        /// <summary>
        /// Third row of the matrix.
        /// </summary>
        public Vector3Wide Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(ref Matrix3x3Wide a, ref Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.Y.X + a.X.Z * b.Z.X;
            result.X.Y = a.X.X * b.X.Y + a.X.Y * b.Y.Y + a.X.Z * b.Z.Y;
            result.X.Z = a.X.X * b.X.Z + a.X.Y * b.Y.Z + a.X.Z * b.Z.Z;

            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.Y.X + a.Y.Z * b.Z.X;
            result.Y.Y = a.Y.X * b.X.Y + a.Y.Y * b.Y.Y + a.Y.Z * b.Z.Y;
            result.Y.Z = a.Y.X * b.X.Z + a.Y.Y * b.Y.Z + a.Y.Z * b.Z.Z;

            result.Z.X = a.Z.X * b.X.X + a.Z.Y * b.Y.X + a.Z.Z * b.Z.X;
            result.Z.Y = a.Z.X * b.X.Y + a.Z.Y * b.Y.Y + a.Z.Z * b.Z.Y;
            result.Z.Z = a.Z.X * b.X.Z + a.Z.Y * b.Y.Z + a.Z.Z * b.Z.Z;
        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(ref Matrix3x3Wide a, ref Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.X.Y + a.X.Z * b.X.Z;
            result.X.Y = a.X.X * b.Y.X + a.X.Y * b.Y.Y + a.X.Z * b.Y.Z;
            result.X.Z = a.X.X * b.Z.X + a.X.Y * b.Z.Y + a.X.Z * b.Z.Z;

            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.X.Y + a.Y.Z * b.X.Z;
            result.Y.Y = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y + a.Y.Z * b.Y.Z;
            result.Y.Z = a.Y.X * b.Z.X + a.Y.Y * b.Z.Y + a.Y.Z * b.Z.Z;

            result.Z.X = a.Z.X * b.X.X + a.Z.Y * b.X.Y + a.Z.Z * b.X.Z;
            result.Z.Y = a.Z.X * b.Y.X + a.Z.Y * b.Y.Y + a.Z.Z * b.Y.Z;
            result.Z.Z = a.Z.X * b.Z.X + a.Z.Y * b.Z.Y + a.Z.Z * b.Z.Z;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector3Wide v, ref Matrix3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.Y.X + v.Z * m.Z.X;
            result.Y = v.X * m.X.Y + v.Y * m.Y.Y + v.Z * m.Z.Y;
            result.Z = v.X * m.X.Z + v.Y * m.Y.Z + v.Z * m.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3Wide v, ref Matrix3x3Wide m, out Vector3Wide result)
        {
            TransformWithoutOverlap(ref v, ref m, out var temp);
            result = temp;
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scalar value.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scaling value to apply to the matrix's components.</param>
        /// <param name="result">Resulting matrix with scaled components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Matrix3x3Wide m, ref Vector<float> scale, out Matrix3x3Wide result)
        {
            result.X.X = m.X.X * scale;
            result.X.Y = m.X.Y * scale;
            result.X.Z = m.X.Z * scale;
            result.Y.X = m.Y.X * scale;
            result.Y.Y = m.Y.Y * scale;
            result.Y.Z = m.Y.Z * scale;
            result.Z.X = m.Z.X * scale;
            result.Z.Y = m.Z.Y * scale;
            result.Z.Z = m.Z.Z * scale;
        }

    }
}
