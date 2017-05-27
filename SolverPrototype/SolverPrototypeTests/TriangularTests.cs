using BEPUutilities2;
using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototypeTests
{
    public static class TriangularTests
    {
        interface ITest
        {
            void Do();
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double TimeTest<T>(int innerIterations, ref T test) where T : ITest
        {
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < innerIterations; ++i)
            {
                test.Do();
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }

        struct TriangularSandwich : ITest
        {
            public Triangular3x3 triangular;
            public Matrix3x3 rotation;
            public Triangular3x3 result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Triangular3x3.RotationSandwich(ref rotation, ref triangular, out result);
            }

        }
        struct TriangularSandwichWide : ITest
        {
            public Triangular3x3Wide triangular;
            public Matrix3x3Wide rotation;
            public Triangular3x3Wide result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Triangular3x3Wide.RotationSandwich(ref rotation, ref triangular, out result);
            }
        }
        struct SymmetricSandwich : ITest
        {
            public Matrix3x3 symmetric;
            public Matrix3x3 rotation;
            public Matrix3x3 result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3.MultiplyTransposed(ref rotation, ref symmetric, out var intermediate);
                Matrix3x3.Multiply(ref intermediate, ref symmetric, out result);
            }
        }
        struct SymmetricSandwichWide : ITest
        {
            public Matrix3x3Wide symmetric;
            public Matrix3x3Wide rotation;
            public Matrix3x3Wide result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3Wide.MultiplyTransposedWithoutOverlap(ref rotation, ref symmetric, out var intermediateWide);
                Matrix3x3Wide.MultiplyWithoutOverlap(ref intermediateWide, ref rotation, out result);
            }
        }

        const float epsilon = 5e-6f;
        static void Compare(ref Matrix3x3 m, ref Triangular3x3 t)
        {
            if (Math.Abs(m.X.Y - m.Y.X) +
                Math.Abs(m.X.Z - m.Z.X) +
                Math.Abs(m.Y.Z - m.Z.Y) > epsilon)
            {
                throw new Exception("Matrix not symmetric; shouldn't compare against a symmetric triangular matrix.");
            }
            if (Math.Abs(m.X.X - t.M11) +
                Math.Abs(m.Y.X - t.M21) +
                Math.Abs(m.Y.X - t.M21) +
                Math.Abs(m.Z.X - t.M31) +
                Math.Abs(m.Z.Y - t.M32) +
                Math.Abs(m.Z.Z - t.M33) > epsilon)
            {
                throw new Exception("Too much error in Matrix3x3 vs Triangular3x3.");
            }
        }
        static void Compare(ref Matrix3x3 m, ref Matrix3x3Wide wide)
        {
            if (Math.Abs(m.X.X - wide.X.X[0]) +
                Math.Abs(m.X.Y - wide.X.Y[0]) +
                Math.Abs(m.X.Z - wide.X.Z[0]) +
                Math.Abs(m.Y.X - wide.Y.X[0]) +
                Math.Abs(m.Y.Y - wide.Y.Y[0]) +
                Math.Abs(m.Y.Z - wide.Y.Z[0]) +
                Math.Abs(m.Z.X - wide.Z.X[0]) +
                Math.Abs(m.Z.Y - wide.Z.Y[0]) +
                Math.Abs(m.Z.Z - wide.Z.Z[0]) > epsilon)
            {
                throw new Exception("Too much error in Matrix3x3 vs Matrix3x3Wide.");
            }
        }
        static void Compare(ref Triangular3x3 t, ref Triangular3x3Wide wide)
        {
            if (Math.Abs(t.M11 - wide.M11[0]) +
                Math.Abs(t.M21 - wide.M21[0]) +
                Math.Abs(t.M21 - wide.M21[0]) +
                Math.Abs(t.M31 - wide.M31[0]) +
                Math.Abs(t.M32 - wide.M32[0]) +
                Math.Abs(t.M33 - wide.M33[0]) > epsilon)
            {
                throw new Exception("Too much error in Triangular3x3 vs Triangular3x3Wide.");
            }
        }

        public static void Test()
        {
            var random = new Random(5);
            var timer = new Stopwatch();
            var symmetricTime = 0.0;
            var symmetricWideTime = 0.0;
            var triangularTime = 0.0;
            var triangularWideTime = 0.0;
            for (int i = 0; i < 1000; ++i)
            {
                var axis = new Vector3((float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1);
                var rotation = Matrix3x3.CreateFromAxisAngle(axis, (float)random.NextDouble());
                Matrix3x3Wide rotationWide;
                Vector3Wide.CreateFrom(ref rotation.X, out rotationWide.X);
                Vector3Wide.CreateFrom(ref rotation.Y, out rotationWide.Y);
                Vector3Wide.CreateFrom(ref rotation.Z, out rotationWide.Z);

                var triangular = new Triangular3x3
                {
                    M11 = (float)random.NextDouble() * 5 + 1,
                    M21 = (float)random.NextDouble() * 1 + 1,
                    M22 = (float)random.NextDouble() * 5 + 1,
                    M31 = (float)random.NextDouble() * 1 + 1,
                    M32 = (float)random.NextDouble() * 1 + 1,
                    M33 = (float)random.NextDouble() * 5 + 1,
                };
                Triangular3x3Wide triangularWide;
                triangularWide.M11 = new Vector<float>(triangular.M11);
                triangularWide.M21 = new Vector<float>(triangular.M21);
                triangularWide.M22 = new Vector<float>(triangular.M22);
                triangularWide.M31 = new Vector<float>(triangular.M31);
                triangularWide.M32 = new Vector<float>(triangular.M32);
                triangularWide.M33 = new Vector<float>(triangular.M33);

                var symmetric = new Matrix3x3
                {
                    X = new Vector3(triangular.M11, triangular.M21, triangular.M31),
                    Y = new Vector3(triangular.M21, triangular.M22, triangular.M32),
                    Z = new Vector3(triangular.M31, triangular.M32, triangular.M33),
                };
                Matrix3x3Wide symmetricWide;
                Vector3Wide.CreateFrom(ref symmetric.X, out symmetricWide.X);
                Vector3Wide.CreateFrom(ref symmetric.Y, out symmetricWide.Y);
                Vector3Wide.CreateFrom(ref symmetric.Z, out symmetricWide.Z);

                var symmetricSandwich = new SymmetricSandwich() { rotation = rotation, symmetric = symmetric };
                var symmetricSandwichWide = new SymmetricSandwichWide() { rotation = rotationWide, symmetric = symmetricWide };
                var triangularSandwich = new TriangularSandwich() { rotation = rotation, triangular = triangular };
                var triangularSandwichWide = new TriangularSandwichWide() { rotation = rotationWide, triangular = triangularWide };


                const int innerIterations = 1000;
                symmetricTime += TimeTest(innerIterations, ref symmetricSandwich);
                symmetricWideTime += TimeTest(innerIterations, ref symmetricSandwichWide);
                triangularTime += TimeTest(innerIterations, ref triangularSandwich);
                triangularWideTime += TimeTest(innerIterations, ref triangularSandwichWide);

                Compare(ref symmetricSandwich.result, ref symmetricSandwichWide.result);
                Compare(ref symmetricSandwich.result, ref triangularSandwich.result);
                Compare(ref triangularSandwich.result, ref triangularSandwichWide.result);
            }

            Console.WriteLine($"Symmetric:       {symmetricTime}");
            Console.WriteLine($"Symmetric wide:  {symmetricTime}");
            Console.WriteLine($"Triangular:      {symmetricTime}");
            Console.WriteLine($"Triangular wide: {symmetricTime}");


        }
    }

}