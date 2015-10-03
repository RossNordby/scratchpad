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
            const int iterationCount = 100000;
            Random random = new Random(5);
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
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
                    scalarPointerA[i] = systemPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = systemPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
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
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerResult[i] - scalarPointerResult[i]);
                    var simdSystemError = Math.Abs(simdPointerResult[i] - systemPointerResult[i]);
                    if (simdScalarError > threshold ||
                        simdSystemError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
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
                    scalarPointerA[i] = systemPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = systemPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
                }
                
                MatrixSIMD.Multiply(ref simdA, ref simdB, out simdA);
                systemA = Matrix4x4.Multiply(systemA, systemB);
                Matrix.Multiply(ref scalarA, ref scalarB, out scalarA);


                for (int i = 0; i < 16; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerA[i] - scalarPointerA[i]);
                    var simdSystemError = Math.Abs(simdPointerA[i] - systemPointerA[i]);
                    if (simdScalarError > threshold ||
                        simdSystemError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
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
                    scalarPointerA[i] = systemPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = systemPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
                }

                //MatrixSIMD.Multiply(ref simdA, ref simdB);
                MatrixSIMD.Multiply(ref simdA, ref simdB, out simdB);
                systemB = Matrix4x4.Multiply(systemA, systemB);
                Matrix.Multiply(ref scalarA, ref scalarB, out scalarB);


                for (int i = 0; i < 16; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerB[i] - scalarPointerB[i]);
                    var simdSystemError = Math.Abs(simdPointerB[i] - systemPointerB[i]);
                    if (simdScalarError > threshold ||
                        simdSystemError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }


            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                MatrixSIMD simd;
                Matrix4x4 system;
                Matrix scalar;
                var simdPointer = (float*)&simd;
                var systemPointer = (float*)&system;
                var scalarPointer = (float*)&scalar;
                for (int i = 0; i < 16; ++i)
                {
                    scalarPointer[i] = systemPointer[i] = simdPointer[i] = (float)(random.NextDouble() * 4 - 2);
                }

                MatrixSIMD.Multiply(ref simd, ref simd, out simd);
                system = Matrix4x4.Multiply(system, system);
                Matrix.Multiply(ref scalar, ref scalar, out scalar);

                for (int i = 0; i < 16; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointer[i] - scalarPointer[i]);
                    var simdSystemError = Math.Abs(simdPointer[i] - systemPointer[i]);
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
            const int iterationCount = 10000000;

            Helper.Test("Multiply SIMD", TestSIMDMultiply, iterationCount);
            Helper.Test("Multiply Scalar", TestScalarMultiply, iterationCount);
            Helper.Test("Multiply System", TestSystemMultiply, iterationCount);

            Helper.Test("Transform SIMD", TestSIMDTransform, iterationCount);
            Helper.Test("TransformTranspose SIMD", TestSIMDTransformTranspose, iterationCount);
            Helper.Test("Transform Scalar", TestTransformScalar, iterationCount);
            Helper.Test("Transform System", TestTransformSystem, iterationCount);

        }
    }
}
