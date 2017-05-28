﻿using BEPUutilities2;
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
            test.Do();
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < innerIterations; ++i)
            {
                test.Do();
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }

        static double MeasureError(double a, double b)
        {
            if (Math.Abs(a) > 1e-7)
                return Math.Abs(b / a - 1);
            return Math.Abs(b - a);
        }
        const double epsilon = 1e-3f;
        static void Compare(float a, ref Vector<float> b)
        {
            var error = MeasureError(a, b[0]);
            if (error > epsilon)
            {
                throw new Exception("Too much error on wide vs scalar 1x1.");
            }
        }
        static void Compare(ref Matrix3x3 m, ref Triangular3x3Wide t)
        {
            var se12 = MeasureError(m.X.Y, m.Y.X);
            var se13 = MeasureError(m.X.Z, m.Z.X);
            var se23 = MeasureError(m.Y.Z, m.Z.Y);
            if (se12 > epsilon ||
                se13 > epsilon ||
                se23 > epsilon)
            {
                throw new Exception("Matrix not symmetric; shouldn't compare against a symmetric triangular matrix.");
            }
            var e11 = MeasureError(m.X.X, t.M11[0]);
            var e21 = MeasureError(m.Y.X, t.M21[0]);
            var e22 = MeasureError(m.Y.X, t.M21[0]);
            var e31 = MeasureError(m.Z.X, t.M31[0]);
            var e32 = MeasureError(m.Z.Y, t.M32[0]);
            var e33 = MeasureError(m.Z.Z, t.M33[0]);
            if (e11 > epsilon ||
                e21 > epsilon ||
                e22 > epsilon ||
                e31 > epsilon ||
                e32 > epsilon ||
                e33 > epsilon)
            {
                throw new Exception("Too much error in Matrix3x3 vs Triangular3x3Wide.");
            }
        }
        static void Compare(ref Matrix3x3 m, ref Matrix3x3Wide wide)
        {
            var e11 = MeasureError(m.X.X, wide.X.X[0]);
            var e12 = MeasureError(m.X.Y, wide.X.Y[0]);
            var e13 = MeasureError(m.X.Z, wide.X.Z[0]);
            var e21 = MeasureError(m.Y.X, wide.Y.X[0]);
            var e22 = MeasureError(m.Y.Y, wide.Y.Y[0]);
            var e23 = MeasureError(m.Y.Z, wide.Y.Z[0]);
            var e31 = MeasureError(m.Z.X, wide.Z.X[0]);
            var e32 = MeasureError(m.Z.Y, wide.Z.Y[0]);
            var e33 = MeasureError(m.Z.Z, wide.Z.Z[0]);

            if (e11 > epsilon ||
                e12 > epsilon ||
                e13 > epsilon ||
                e21 > epsilon ||
                e22 > epsilon ||
                e23 > epsilon ||
                e31 > epsilon ||
                e32 > epsilon ||
                e33 > epsilon)
            {
                throw new Exception("Too much error in Matrix3x3 vs Matrix3x3Wide.");
            }
        }

        struct TriangularWideVectorSandwich : ITest
        {
            public Triangular3x3Wide triangular;
            public Vector3Wide v;
            public Vector<float> result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Triangular3x3Wide.VectorSandwich(ref v, ref triangular, out result);
            }
        }
        struct SymmetricVectorSandwich : ITest
        {
            public Matrix3x3 symmetric;
            public Vector3 v;
            public float result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3.Transform(ref v, ref symmetric, out var intermediate);
                result = Vector3.Dot(v, intermediate);
            }
        }
        struct SymmetricWideVectorSandwich : ITest
        {
            public Matrix3x3Wide symmetric;
            public Vector3Wide v;
            public Vector<float> result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3Wide.Transform(ref v, ref symmetric, out var intermediate);
                Vector3Wide.Dot(ref v, ref intermediate, out result);
            }
        }

        struct TriangularRotationSandwichWide : ITest
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
        struct SymmetricRotationSandwich : ITest
        {
            public Matrix3x3 symmetric;
            public Matrix3x3 rotation;
            public Matrix3x3 result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3.MultiplyTransposed(ref rotation, ref symmetric, out var intermediate);
                Matrix3x3.Multiply(ref intermediate, ref rotation, out result);
            }
        }
        struct SymmetricRotationSandwichWide : ITest
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
        struct TriangularInvertWide : ITest
        {
            public Triangular3x3Wide triangular;
            public Triangular3x3Wide result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Triangular3x3Wide.SymmetricInvert(ref triangular, out result);
            }
        }
        struct SymmetricInvert : ITest
        {
            public Matrix3x3 symmetric;
            public Matrix3x3 result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3.Invert(ref symmetric, out result);
            }
        }
        struct SymmetricInvertWide : ITest
        {
            public Matrix3x3Wide symmetric;
            public Matrix3x3Wide result;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do()
            {
                Matrix3x3Wide.Invert(ref symmetric, out result);
            }
        }




        public static void Test()
        {
            var random = new Random(4);
            var timer = new Stopwatch();
            var symmetricVectorSandwichTime = 0.0;
            var symmetricWideVectorSandwichTime = 0.0;
            var triangularWideVectorSandwichTime = 0.0;
            var symmetricRotationSandwichTime = 0.0;
            var symmetricWideRotationSandwichTime = 0.0;
            var triangularWideRotationSandwichTime = 0.0;
            var symmetricInvertTime = 0.0;
            var symmetricWideInvertTime = 0.0;
            var triangularWideInvertTime = 0.0;
            for (int i = 0; i < 1000; ++i)
            {
                var axis = Vector3.Normalize(new Vector3((float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1));
                Vector3Wide.CreateFrom(ref axis, out var axisWide);
                var rotation = Matrix3x3.CreateFromAxisAngle(axis, (float)random.NextDouble());
                Matrix3x3Wide rotationWide;
                Vector3Wide.CreateFrom(ref rotation.X, out rotationWide.X);
                Vector3Wide.CreateFrom(ref rotation.Y, out rotationWide.Y);
                Vector3Wide.CreateFrom(ref rotation.Z, out rotationWide.Z);

                var triangular = new Triangular3x3
                {
                    M11 = (float)random.NextDouble() * 2 + 1,
                    M21 = (float)random.NextDouble() * 1 + 1,
                    M22 = (float)random.NextDouble() * 2 + 1,
                    M31 = (float)random.NextDouble() * 1 + 1,
                    M32 = (float)random.NextDouble() * 1 + 1,
                    M33 = (float)random.NextDouble() * 2 + 1,
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

                var symmetricVectorSandwich = new SymmetricVectorSandwich() { v = axis, symmetric = symmetric };
                var symmetricWideVectorSandwich = new SymmetricWideVectorSandwich() { v = axisWide, symmetric = symmetricWide };
                var triangularWideVectorSandwich = new TriangularWideVectorSandwich() { v = axisWide, triangular = triangularWide };
                var symmetricSandwich = new SymmetricRotationSandwich() { rotation = rotation, symmetric = symmetric };
                var symmetricWideSandwich = new SymmetricRotationSandwichWide() { rotation = rotationWide, symmetric = symmetricWide };
                var triangularWideSandwich = new TriangularRotationSandwichWide() { rotation = rotationWide, triangular = triangularWide };
                var symmetricInvert = new SymmetricInvert() { symmetric = symmetric };
                var symmetricWideInvert = new SymmetricInvertWide() { symmetric = symmetricWide };
                var triangularWideInvert = new TriangularInvertWide() { triangular = triangularWide };


                const int innerIterations = 100000;
                symmetricVectorSandwichTime += TimeTest(innerIterations, ref symmetricVectorSandwich);
                symmetricWideVectorSandwichTime += TimeTest(innerIterations, ref symmetricWideVectorSandwich);
                triangularWideVectorSandwichTime += TimeTest(innerIterations, ref triangularWideVectorSandwich);
                symmetricRotationSandwichTime += TimeTest(innerIterations, ref symmetricSandwich);
                symmetricWideRotationSandwichTime += TimeTest(innerIterations, ref symmetricWideSandwich);
                triangularWideRotationSandwichTime += TimeTest(innerIterations, ref triangularWideSandwich);
                symmetricInvertTime += TimeTest(innerIterations, ref symmetricInvert);
                symmetricWideInvertTime += TimeTest(innerIterations, ref symmetricWideInvert);
                triangularWideInvertTime += TimeTest(innerIterations, ref triangularWideInvert);

                Compare(symmetricVectorSandwich.result, ref symmetricWideVectorSandwich.result);
                Compare(symmetricVectorSandwich.result, ref triangularWideVectorSandwich.result);
                Compare(ref symmetricSandwich.result, ref symmetricWideSandwich.result);
                Compare(ref symmetricSandwich.result, ref triangularWideSandwich.result);
                Compare(ref symmetricInvert.result, ref symmetricWideInvert.result);
                Compare(ref symmetricInvert.result, ref triangularWideInvert.result);
            }

            Console.WriteLine($"Symmetric vector sandwich:       {symmetricVectorSandwichTime}");
            Console.WriteLine($"Symmetric wide vector sandwich:  {symmetricWideVectorSandwichTime}");
            Console.WriteLine($"Triangular wide vector sandwich: {triangularWideVectorSandwichTime}");
            Console.WriteLine($"Symmetric rotation sandwich:       {symmetricRotationSandwichTime}");
            Console.WriteLine($"Symmetric wide rotation sandwich:  {symmetricWideRotationSandwichTime}");
            Console.WriteLine($"Triangular wide rotation sandwich: {triangularWideRotationSandwichTime}");
            Console.WriteLine($"Symmetric invert:       {symmetricInvertTime}");
            Console.WriteLine($"Symmetric wide invert:  {symmetricWideInvertTime}");
            Console.WriteLine($"Triangular wide invert: {triangularWideInvertTime}");


        }
    }

}