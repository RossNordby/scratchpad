using System;
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
            //Overall, this turns out to be almost identical in per-call performance to the scalar version,
            //but this version calculates Vector<float>.Count quaternions at the same time, so it's much, much faster when applicable.
            var oneAddX = Vector<float>.One + r.X.X;
            var oneSubX = Vector<float>.One - r.X.X;
            var yAddZ = r.Y.Y + r.Z.Z;
            var ySubZ = r.Y.Y - r.Z.Z;
            var tX = oneAddX - yAddZ;
            var tY = oneSubX + ySubZ;
            var tZ = oneSubX - ySubZ;
            var tW = oneAddX + yAddZ;

            var xyAddYx = r.X.Y + r.Y.X;
            var xySubYx = r.X.Y - r.Y.X;
            var yzAddZy = r.Y.Z + r.Z.Y;
            var yzSubZy = r.Y.Z - r.Z.Y;
            var zxAddXz = r.Z.X + r.X.Z;
            var zxSubXz = r.Z.X - r.X.Z;

            //There are two layers of conditions- inner, and outer. We have to first select each of the two inner halves- upper, and lower-
            //and then we will select which of the two inners to use for the outer.
            var useUpper = Vector.LessThan(r.Z.Z, Vector<float>.Zero);
            var useUpperUpper = Vector.GreaterThan(r.X.X, r.Y.Y);
            var useLowerUpper = Vector.LessThan(r.X.X, Vector.Negate(r.Y.Y));
            var t = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, tY),
                    Vector.ConditionalSelect(useLowerUpper, tZ, tW));
            q.X = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, xyAddYx),
                    Vector.ConditionalSelect(useLowerUpper, zxAddXz, yzSubZy));
            q.Y = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, xyAddYx, tY),
                    Vector.ConditionalSelect(useLowerUpper, yzAddZy, zxSubXz));
            q.Z = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, zxAddXz, yzAddZy),
                    Vector.ConditionalSelect(useLowerUpper, tZ, xySubYx));
            q.W = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, yzSubZy, zxSubXz),
                    Vector.ConditionalSelect(useLowerUpper, xySubYx, tW));

            var scale = new Vector<float>(0.5f) / Vector.SquareRoot(t);
            Scale(ref q, ref scale, out q);


            //This branchless variant works, but encounters precision issues when the trace (1 + XX + YY + ZZ) approaches 0.
            //q.X = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One + r.X.X - r.Y.Y - r.Z.Z)) * 0.5f;
            //q.Y = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - r.X.X + r.Y.Y - r.Z.Z)) * 0.5f;
            //q.Z = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - r.X.X - r.Y.Y + r.Z.Z)) * 0.5f;
            //q.W = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One + r.X.X + r.Y.Y + r.Z.Z)) * 0.5f;
            //var xSignSource = r.Y.Z - r.Z.Y;
            //var ySignSource = r.Z.X - r.X.Z;
            //var zSignSource = r.X.Y - r.Y.X;
            //const int mask = unchecked((int)0x80000000);
            //var maskVector = new Vector<int>(mask);
            //var inverseMaskVector = new Vector<int>(~mask);
            //q.X = Vector.AsVectorSingle(Vector.BitwiseOr(Vector.BitwiseAnd(Vector.AsVectorInt32(xSignSource), maskVector), Vector.BitwiseAnd(Vector.AsVectorInt32(q.X), inverseMaskVector)));
            //q.Y = Vector.AsVectorSingle(Vector.BitwiseOr(Vector.BitwiseAnd(Vector.AsVectorInt32(ySignSource), maskVector), Vector.BitwiseAnd(Vector.AsVectorInt32(q.Y), inverseMaskVector)));
            //q.Z = Vector.AsVectorSingle(Vector.BitwiseOr(Vector.BitwiseAnd(Vector.AsVectorInt32(zSignSource), maskVector), Vector.BitwiseAnd(Vector.AsVectorInt32(q.Z), inverseMaskVector)));
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
    }
}