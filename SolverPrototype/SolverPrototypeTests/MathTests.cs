using BEPUutilities2;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    public static class MathTests
    {

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double TestQuaternion()
        {
            var random = new Random(5);
            const int iterations = 10000000;
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                Quaternion q;
                q.X = (float)random.NextDouble() * 2 - 1;
                q.Y = (float)random.NextDouble() * 2 - 1;
                q.Z = (float)random.NextDouble() * 2 - 1;
                q.W = (float)random.NextDouble() * 2 - 1;

                Quaternion.Normalize(ref q, out q);

                Matrix3x3.CreateFromQuaternion(ref q, out var r);
                Quaternion.CreateFromRotationMatrix(ref r, out var qTest);

                //const float epsilon = 1e-6f;
                //var lengthX = r.X.Length();
                //var lengthY = r.Y.Length();
                //var lengthZ = r.Z.Length();
                //Debug.Assert(
                //    Math.Abs(1 - lengthX) < epsilon &&
                //    Math.Abs(1 - lengthY) < epsilon &&
                //    Math.Abs(1 - lengthZ) < epsilon);


                //if (qTest.X * q.X < 0)
                //{
                //    Quaternion.Negate(ref qTest, out qTest);
                //}
                //Debug.Assert(
                //    Math.Abs(qTest.X - q.X) < epsilon &&
                //    Math.Abs(qTest.Y - q.Y) < epsilon &&
                //    Math.Abs(qTest.Z - q.Z) < epsilon &&
                //    Math.Abs(qTest.W - q.W) < epsilon);
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        static double TestQuaternionWide()
        {
            var random = new Random(5);
            const int iterations = 10000000;
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                QuaternionWide q;
                q.X = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.Y = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.Z = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.W = new Vector<float>((float)random.NextDouble() * 2 - 1);

                QuaternionWide.Normalize(ref q, out q);

                Matrix3x3Wide.CreateFromQuaternion(ref q, out var r);
                QuaternionWide.CreateFromRotationMatrix(ref r, out var qTest);

                //const float epsilon = 1e-6f;
                //Vector3Wide.Length(ref r.X, out var lengthX);
                //Vector3Wide.Length(ref r.Y, out var lengthY);
                //Vector3Wide.Length(ref r.Z, out var lengthZ);
                //Debug.Assert(
                //    Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthX), new Vector<float>(epsilon)) &&
                //    Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthY), new Vector<float>(epsilon)) &&
                //    Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthZ), new Vector<float>(epsilon)));


                //if (qTest.X[0] * q.X[0] < 0)
                //{
                //    QuaternionWide.Negate(ref qTest, out qTest);
                //}
                //Debug.Assert(
                //    Vector.LessThanAll(Vector.Abs(qTest.X - q.X), new Vector<float>(epsilon)) &&
                //    Vector.LessThanAll(Vector.Abs(qTest.Y - q.Y), new Vector<float>(epsilon)) &&
                //    Vector.LessThanAll(Vector.Abs(qTest.Z - q.Z), new Vector<float>(epsilon)) &&
                //    Vector.LessThanAll(Vector.Abs(qTest.W - q.W), new Vector<float>(epsilon)));
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;

        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            Console.WriteLine($"Scalar: {TestQuaternion()}");
            Console.WriteLine($"SIMD: {TestQuaternionWide()}");
        }
    }
}
