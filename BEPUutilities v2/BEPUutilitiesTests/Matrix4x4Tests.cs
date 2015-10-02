using BEPUutilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    public static class Matrix4x4Tests
    {

        public static float TestTransformScalar(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            Matrix m = Matrix.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                Matrix.Transform(ref v, ref m, out r0);
                Matrix.Transform(ref r0, ref m, out r1);
                Matrix.Transform(ref r1, ref m, out r0);
                Matrix.Transform(ref r0, ref m, out r1);
                Matrix.Transform(ref r1, ref m, out r0);
                Matrix.Transform(ref r0, ref m, out r1);
                Matrix.Transform(ref r1, ref m, out r0);
                Matrix.Transform(ref r0, ref m, out r1);
                Matrix.Transform(ref r1, ref m, out r0);
                Matrix.Transform(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }
        public static float TestTransformSystem(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            Matrix4x4 m = Matrix4x4.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                r0 = Vector4.Transform(v, m);
                r1 = Vector4.Transform(r0, m);
                r0 = Vector4.Transform(r1, m);
                r1 = Vector4.Transform(r0, m);
                r0 = Vector4.Transform(r1, m);
                r1 = Vector4.Transform(r0, m);
                r0 = Vector4.Transform(r1, m);
                r1 = Vector4.Transform(r0, m);
                r0 = Vector4.Transform(r1, m);
                r1 = Vector4.Transform(r0, m);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }
        public static float TestSIMDTransformTranspose(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            MatrixSIMD m = MatrixSIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                MatrixSIMD.TransformTranspose(ref v, ref m, out r0);
                MatrixSIMD.TransformTranspose(ref r0, ref m, out r1);
                MatrixSIMD.TransformTranspose(ref r1, ref m, out r0);
                MatrixSIMD.TransformTranspose(ref r0, ref m, out r1);
                MatrixSIMD.TransformTranspose(ref r1, ref m, out r0);
                MatrixSIMD.TransformTranspose(ref r0, ref m, out r1);
                MatrixSIMD.TransformTranspose(ref r1, ref m, out r0);
                MatrixSIMD.TransformTranspose(ref r0, ref m, out r1);
                MatrixSIMD.TransformTranspose(ref r1, ref m, out r0);
                MatrixSIMD.TransformTranspose(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }

        public static float TestSIMDTransform(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            MatrixSIMD m = MatrixSIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                MatrixSIMD.Transform(ref v, ref m, out r0);
                MatrixSIMD.Transform(ref r0, ref m, out r1);
                MatrixSIMD.Transform(ref r1, ref m, out r0);
                MatrixSIMD.Transform(ref r0, ref m, out r1);
                MatrixSIMD.Transform(ref r1, ref m, out r0);
                MatrixSIMD.Transform(ref r0, ref m, out r1);
                MatrixSIMD.Transform(ref r1, ref m, out r0);
                MatrixSIMD.Transform(ref r0, ref m, out r1);
                MatrixSIMD.Transform(ref r1, ref m, out r0);
                MatrixSIMD.Transform(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }

        public static float TestSystemMultiply(int iterationCount)
        {
            Matrix4x4 m1 = Matrix4x4.Identity;
            Matrix4x4 m2 = Matrix4x4.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix4x4 r0, r1;
                r0 = Matrix4x4.Multiply(m1, m2);
                r1 = Matrix4x4.Multiply(r0, m2);
                r0 = Matrix4x4.Multiply(r1, m2);
                r1 = Matrix4x4.Multiply(r0, m2);
                r0 = Matrix4x4.Multiply(r1, m2);
                r1 = Matrix4x4.Multiply(r0, m2);
                r0 = Matrix4x4.Multiply(r1, m2);
                r1 = Matrix4x4.Multiply(r0, m2);
                r0 = Matrix4x4.Multiply(r1, m2);
                r1 = Matrix4x4.Multiply(r0, m2);
                accumulator += 0.000001f * r1.M11;
            }
            return accumulator;
        }

        public static float TestScalarMultiply(int iterationCount)
        {
            Matrix m1 = Matrix.Identity;
            Matrix m2 = Matrix.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix r0, r1;
                Matrix.Multiply(ref m1, ref m2, out r0);
                Matrix.Multiply(ref r0, ref m2, out r1);
                Matrix.Multiply(ref r1, ref m2, out r0);
                Matrix.Multiply(ref r0, ref m2, out r1);
                Matrix.Multiply(ref r1, ref m2, out r0);
                Matrix.Multiply(ref r0, ref m2, out r1);
                Matrix.Multiply(ref r1, ref m2, out r0);
                Matrix.Multiply(ref r0, ref m2, out r1);
                Matrix.Multiply(ref r1, ref m2, out r0);
                Matrix.Multiply(ref r0, ref m2, out r1);
                accumulator += 0.000001f * r1.M11;
            }
            return accumulator;
        }

        public static float TestSIMDMultiply(int iterationCount)
        {
            MatrixSIMD m1 = MatrixSIMD.Identity;
            MatrixSIMD m2 = MatrixSIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                MatrixSIMD r0, r1;
                MatrixSIMD.Multiply(ref m1, ref m2, out r0);
                MatrixSIMD.Multiply(ref r0, ref m2, out r1);
                MatrixSIMD.Multiply(ref r1, ref m2, out r0);
                MatrixSIMD.Multiply(ref r0, ref m2, out r1);
                MatrixSIMD.Multiply(ref r1, ref m2, out r0);
                MatrixSIMD.Multiply(ref r0, ref m2, out r1);
                MatrixSIMD.Multiply(ref r1, ref m2, out r0);
                MatrixSIMD.Multiply(ref r0, ref m2, out r1);
                MatrixSIMD.Multiply(ref r1, ref m2, out r0);
                MatrixSIMD.Multiply(ref r0, ref m2, out r1);
                accumulator += 0.000001f * r1.X.X;
            }
            return accumulator;
        }

        public unsafe static void TestMultiplyCorrectness()
        {
            Random random = new Random(5);
            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                MatrixSIMD simdA, simdB;
                Matrix4x4 systemA, systemB;
                Matrix scalarA, scalarB;
                var simdPointerA = (float*)&simdA;
                var systemPointerA = (float*)&systemA;
                var scalarPointerA = (float*)&scalarA;
                var simdPointerB = (float*)&simdB;
                var scalarPointerB = (float*)&scalarB;
                var systemPointerB = (float*)&systemB;
                for (int i = 0; i < 16; ++i)
                {
                    scalarPointerA[i] = systemPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 20 - 10);
                    scalarPointerB[i] = systemPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 20 - 10);
                }

                MatrixSIMD simdResult;
                MatrixSIMD.Multiply(ref simdA, ref simdB, out simdResult);
                var systemResult = Matrix4x4.Multiply(systemA, systemB);
                Matrix scalarResult;
                Matrix.Multiply(ref scalarA, ref scalarB, out scalarResult);
                var simdPointerResult = (float*)&simdResult;
                var scalarPointerResult = (float*)&scalarResult;
                var systemPointerResult = (float*)&systemResult;

                for (int i = 0; i < 16; ++i)
                {
                    const float threshold = 0;
                    var simdScalarError = Math.Abs(simdPointerResult[i] - scalarPointerResult[i]);
                    var simdSystemError = Math.Abs(simdPointerResult[i] - systemPointerResult[i]);
                    if (simdScalarError > threshold ||
                        simdSystemError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }
        }

        public static void Test()
        {
            TestMultiplyCorrectness();
            const int iterationCount = 1000000;
            Helper.Test("Transform Scalar", Matrix4x4Tests.TestTransformScalar, iterationCount);
            Helper.Test("Transform System", Matrix4x4Tests.TestTransformSystem, iterationCount);
            Helper.Test("Transform SIMD", Matrix4x4Tests.TestSIMDTransform, iterationCount);
            Helper.Test("TransformTranspose SIMD", Matrix4x4Tests.TestSIMDTransformTranspose, iterationCount);

            Helper.Test("Multiply System", Matrix4x4Tests.TestSystemMultiply, iterationCount);
            Helper.Test("Multiply Scalar", Matrix4x4Tests.TestScalarMultiply, iterationCount);
            Helper.Test("Multiply SIMD", Matrix4x4Tests.TestSIMDMultiply, iterationCount);
        }
    }
}
