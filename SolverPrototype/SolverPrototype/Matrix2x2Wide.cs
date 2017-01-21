﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{

    public struct Matrix2x2Wide
    {
        /// <summary>
        /// First row of the matrix.
        /// </summary>
        public Vector2Wide X;
        /// <summary>
        /// Second row of the matrix.
        /// </summary>
        public Vector2Wide Y;


        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(ref Matrix2x2Wide a, ref Matrix2x2Wide b, out Matrix2x2Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.X.Y;
            result.X.Y = a.X.X * b.Y.X + a.X.Y * b.Y.Y;
            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.X.Y;
            result.Y.Y = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector3Wide v, ref Matrix2x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.Y.X;
            result.Y = v.X * m.X.Y + v.Y * m.Y.Y;
            result.Z = v.X * m.X.Z + v.Y * m.Y.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3Wide v, ref Matrix2x3Wide m, out Vector3Wide result)
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
        public static void Scale(ref Matrix2x2Wide m, ref Vector<float> scale, out Matrix2x2Wide result)
        {
            result.X.X = m.X.X * scale;
            result.X.Y = m.X.Y * scale;
            result.Y.X = m.Y.X * scale;
            result.Y.Y = m.Y.Y * scale;
        }

        /// <summary>
        /// Adds the components of one matrix to another.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two given matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Matrix2x2Wide a, ref Matrix2x2Wide b, out Matrix2x2Wide result)
        {
            Vector2Wide.Add(ref a.X, ref b.X, out result.X);
            Vector2Wide.Add(ref a.Y, ref b.Y, out result.Y);
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="matrix">Matrix to be inverted.</param>
        /// <param name="result">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertWithoutOverlap(ref Matrix2x2Wide m, out Matrix2x2Wide inverse)
        {
            var determinantInverse = Vector<float>.One / (m.X.X * m.Y.Y - m.X.Y * m.Y.X);
            inverse.X.X = m.Y.Y * determinantInverse;
            inverse.X.Y = -m.X.Y * determinantInverse;

            inverse.Y.X = -m.Y.X * determinantInverse;
            inverse.Y.Y = m.X.X * determinantInverse;

        }
    }
}
