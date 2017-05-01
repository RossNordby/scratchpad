﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct QuaternionWide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;
        public Vector<float> W;


        /// <summary>
        /// Constructs a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix to create the quaternion from.</param>
        /// <param name="q">Quaternion based on the rotation matrix.</param>
        public static void CreateFromRotationMatrix(ref Matrix3x3Wide r, out QuaternionWide q)
        {
            //Since we can't branch, we're going to end up calculating the possible states of all branches.
            //This requires doing more ALU work than the branching implementation, but there are a lot of common terms across the branches, and (random-ish) branches aren't free.
            //Overall, this turns out to be about 2x-2.5x more expensive per call than the scalar version, but it handles multiple lanes, so it's a net win.
            var oneAddX = Vector<float>.One + r.M11;
            var oneSubX = Vector<float>.One - r.M11;
            var yAddZ = r.M22 + r.M33;
            var ySubZ = r.M22 - r.M33;
            var tX = oneAddX - yAddZ;
            var tY = oneSubX + ySubZ;
            var tZ = oneSubX - ySubZ;
            var tW = oneAddX + yAddZ;

            //There are two layers of conditions- inner, and outer. We have to first select each of the two inner halves- upper, and lower-
            //and then we will select which of the two inners to use for the outer.
            var useUpper = Vector.LessThan(r.M33, Vector<float>.Zero);
            var useUpperUpper = Vector.GreaterThan(r.M11, r.M22);
            var useLowerUpper = Vector.LessThan(r.M11, -r.M22);
            var t = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, tY),
                    Vector.ConditionalSelect(useLowerUpper, tZ, tW));
            var xyAddYx = r.M12 + r.M21;
            var yzSubZy = r.M23 - r.M32;
            var zxAddXz = r.M31 + r.M13;
            q.X = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, xyAddYx),
                    Vector.ConditionalSelect(useLowerUpper, zxAddXz, yzSubZy));
            var yzAddZy = r.M23 + r.M32;
            var zxSubXz = r.M31 - r.M13;
            q.Y = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, xyAddYx, tY),
                    Vector.ConditionalSelect(useLowerUpper, yzAddZy, zxSubXz));
            var xySubYx = r.M12 - r.M21;
            q.Z = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, zxAddXz, yzAddZy),
                    Vector.ConditionalSelect(useLowerUpper, tZ, xySubYx));
            q.W = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, yzSubZy, zxSubXz),
                    Vector.ConditionalSelect(useLowerUpper, xySubYx, tW));

            var scale = new Vector<float>(0.5f) / Vector.SquareRoot(t);
            Scale(ref q, ref scale, out q);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref QuaternionWide q, ref Vector<float> scale, out QuaternionWide result)
        {
            result.X = q.X * scale;
            result.Y = q.Y * scale;
            result.Z = q.Z * scale;
            result.W = q.W * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLengthSquared(ref QuaternionWide q, out Vector<float> lengthSquared)
        {
            lengthSquared = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLength(ref QuaternionWide q, out Vector<float> length)
        {
            length = Vector.SquareRoot(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(ref QuaternionWide q, out QuaternionWide normalized)
        {
            var inverseNorm = Vector<float>.One / Vector.SquareRoot(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            normalized.X = q.X * inverseNorm;
            normalized.Y = q.Y * inverseNorm;
            normalized.Z = q.Z * inverseNorm;
            normalized.W = q.W * inverseNorm;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref QuaternionWide q, out QuaternionWide negated)
        {
            negated.X = -q.X;
            negated.Y = -q.Y;
            negated.Z = -q.Z;
            negated.W = -q.W;
        }

        /// <summary>
        /// Computes the quaternion rotation between two normalized vectors.
        /// </summary>
        /// <param name="v1">First unit-length vector.</param>
        /// <param name="v2">Second unit-length vector.</param>
        /// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
        public static void GetQuaternionBetweenNormalizedVectors(ref Vector3Wide v1, ref Vector3Wide v2, out QuaternionWide q)
        {
            Vector3Wide.Dot(ref v1, ref v2, out var dot);
            //For non-normal vectors, the multiplying the axes length squared would be necessary:
            //float w = dot + Sqrt(v1.LengthSquared() * v2.LengthSquared());


            //There exists an ambiguity at dot == -1. If the directions point away from each other, there are an infinite number of shortest paths.
            //One must be chosen arbitrarily. Here, we choose one by projecting onto the plane whose normal is associated with the smallest magnitude.
            //Since this is a SIMD operation, the special case is always executed and its result is conditionally selected.

            Vector3Wide.CrossWithoutOverlap(ref v1, ref v2, out var cross);
            var useNormalCase = Vector.GreaterThan(dot, new Vector<float>(-0.999999f));
            var absX = Vector.Abs(v1.X);
            var absY = Vector.Abs(v1.Y);
            var absZ = Vector.Abs(v1.Z);
            var xIsSmallest = Vector.BitwiseAnd(Vector.LessThan(absX, absY), Vector.LessThan(absX, absZ));
            var yIsSmaller = Vector.LessThan(absY, absZ);
            q.X = Vector.ConditionalSelect(useNormalCase, cross.X, Vector.ConditionalSelect(xIsSmallest, Vector<float>.Zero, Vector.ConditionalSelect(yIsSmaller, -v1.Z, -v1.Y)));
            q.Y = Vector.ConditionalSelect(useNormalCase, cross.Y, Vector.ConditionalSelect(xIsSmallest, -v1.Z, Vector.ConditionalSelect(yIsSmaller, Vector<float>.Zero, v1.X)));
            q.Z = Vector.ConditionalSelect(useNormalCase, cross.Z, Vector.ConditionalSelect(xIsSmallest, v1.Y, Vector.ConditionalSelect(yIsSmaller, v1.X, Vector<float>.Zero)));
            q.W = Vector.ConditionalSelect(useNormalCase, dot + Vector<float>.One, Vector<float>.Zero);

            Normalize(ref q, out q);
        }

        /// <summary>
        /// Transforms the vector using a quaternion. Assumes that the memory backing the input and output do not overlap.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector3Wide v, ref QuaternionWide rotation, out Vector3Wide result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var wx2 = rotation.W * x2;
            var wy2 = rotation.W * y2;
            var wz2 = rotation.W * z2;
            result.X = v.X * (Vector<float>.One - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
            result.Y = v.X * (xy2 + wz2) + v.Y * (Vector<float>.One - xx2 - zz2) + v.Z * (yz2 - wx2);
            result.Z = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (Vector<float>.One - xx2 - yy2);

        }

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3Wide v, ref QuaternionWide rotation, out Vector3Wide result)
        {
            var tempV = v;
            TransformWithoutOverlap(ref tempV, ref rotation, out result);
        }

        /// <summary>
        /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
        /// transformed = (v * a) * b. Assumes that the memory backing the input and output do not overlap.
        /// </summary>
        /// <param name="a">First quaternion to concatenate.</param>
        /// <param name="b">Second quaternion to concatenate.</param>
        /// <param name="result">Product of the concatenation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConcatenateWithoutOverlap(ref QuaternionWide a, ref QuaternionWide b, out QuaternionWide result)
        {
            result.X = a.W * b.X + a.X * b.W + a.Z * b.Y - a.Y * b.Z;
            result.Y = a.W * b.Y + a.Y * b.W + a.X * b.Z - a.Z * b.X;
            result.Z = a.W * b.Z + a.Z * b.W + a.Y * b.X - a.X * b.Y;
            result.W = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z;
        }

        /// <summary>
        /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
        /// transformed = (v * a) * b.
        /// </summary>
        /// <param name="a">First quaternion to concatenate.</param>
        /// <param name="b">Second quaternion to concatenate.</param>
        /// <param name="result">Product of the concatenation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Concatenate(ref QuaternionWide oa, ref QuaternionWide ob, out QuaternionWide result)
        {
            var tempA = oa;
            var tempB = ob;
            ConcatenateWithoutOverlap(ref tempA, ref tempB, out result);
        }

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <param name="result">Conjugated quaternion.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Conjugate(ref QuaternionWide quaternion, out QuaternionWide result)
        {
            result.X = -quaternion.X;
            result.Y = -quaternion.Y;
            result.Z = -quaternion.Z;
            result.W = quaternion.W;
        }
    }
}