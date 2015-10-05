using BEPUutilities;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    public static class Matrix3x3Tests
    {

        public static float TestTransformScalar(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Matrix3x3.Transform(ref v, ref m, out r0);
                Matrix3x3.Transform(ref r0, ref m, out r1);
                Matrix3x3.Transform(ref r1, ref m, out r0);
                Matrix3x3.Transform(ref r0, ref m, out r1);
                Matrix3x3.Transform(ref r1, ref m, out r0);
                Matrix3x3.Transform(ref r0, ref m, out r1);
                Matrix3x3.Transform(ref r1, ref m, out r0);
                Matrix3x3.Transform(ref r0, ref m, out r1);
                Matrix3x3.Transform(ref r1, ref m, out r0);
                Matrix3x3.Transform(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }
        public static float TestSIMDTransformTranspose(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Matrix3x3SIMD.TransformTranspose(ref v, ref m, out r0);
                Matrix3x3SIMD.TransformTranspose(ref r0, ref m, out r1);
                Matrix3x3SIMD.TransformTranspose(ref r1, ref m, out r0);
                Matrix3x3SIMD.TransformTranspose(ref r0, ref m, out r1);
                Matrix3x3SIMD.TransformTranspose(ref r1, ref m, out r0);
                Matrix3x3SIMD.TransformTranspose(ref r0, ref m, out r1);
                Matrix3x3SIMD.TransformTranspose(ref r1, ref m, out r0);
                Matrix3x3SIMD.TransformTranspose(ref r0, ref m, out r1);
                Matrix3x3SIMD.TransformTranspose(ref r1, ref m, out r0);
                Matrix3x3SIMD.TransformTranspose(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        public static float TestSIMDTransform(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Matrix3x3SIMD.Transform(ref v, ref m, out r0);
                Matrix3x3SIMD.Transform(ref r0, ref m, out r1);
                Matrix3x3SIMD.Transform(ref r1, ref m, out r0);
                Matrix3x3SIMD.Transform(ref r0, ref m, out r1);
                Matrix3x3SIMD.Transform(ref r1, ref m, out r0);
                Matrix3x3SIMD.Transform(ref r0, ref m, out r1);
                Matrix3x3SIMD.Transform(ref r1, ref m, out r0);
                Matrix3x3SIMD.Transform(ref r0, ref m, out r1);
                Matrix3x3SIMD.Transform(ref r1, ref m, out r0);
                Matrix3x3SIMD.Transform(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        public static float TestScalarMultiply(int iterationCount)
        {
            Matrix3x3 m1 = Matrix3x3.Identity;
            Matrix3x3 m2 = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Multiply(ref m1, ref m2, out r0);
                Matrix3x3.Multiply(ref r0, ref m2, out r1);
                Matrix3x3.Multiply(ref r1, ref m2, out r0);
                Matrix3x3.Multiply(ref r0, ref m2, out r1);
                Matrix3x3.Multiply(ref r1, ref m2, out r0);
                Matrix3x3.Multiply(ref r0, ref m2, out r1);
                Matrix3x3.Multiply(ref r1, ref m2, out r0);
                Matrix3x3.Multiply(ref r0, ref m2, out r1);
                Matrix3x3.Multiply(ref r1, ref m2, out r0);
                Matrix3x3.Multiply(ref r0, ref m2, out r1);
                accumulator += 0.000001f * r1.M11;
            }
            return accumulator;
        }

        public static float TestSIMDMultiply(int iterationCount)
        {
            Matrix3x3SIMD m1 = Matrix3x3SIMD.Identity;
            Matrix3x3SIMD m2 = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3SIMD r0, r1;
                Matrix3x3SIMD.Multiply(ref m1, ref m2, out r0);
                Matrix3x3SIMD.Multiply(ref r0, ref m2, out r1);
                Matrix3x3SIMD.Multiply(ref r1, ref m2, out r0);
                Matrix3x3SIMD.Multiply(ref r0, ref m2, out r1);
                Matrix3x3SIMD.Multiply(ref r1, ref m2, out r0);
                Matrix3x3SIMD.Multiply(ref r0, ref m2, out r1);
                Matrix3x3SIMD.Multiply(ref r1, ref m2, out r0);
                Matrix3x3SIMD.Multiply(ref r0, ref m2, out r1);
                Matrix3x3SIMD.Multiply(ref r1, ref m2, out r0);
                Matrix3x3SIMD.Multiply(ref r0, ref m2, out r1);
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
                Matrix3x3SIMD simdA, simdB;
                Matrix3x3 scalarA, scalarB;
                var simdPointerA = (float*)&simdA;
                var scalarPointerA = (float*)&scalarA;
                var simdPointerB = (float*)&simdB;
                var scalarPointerB = (float*)&scalarB;
                for (int i = 0; i < 9; ++i)
                {
                    scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
                }

                Matrix3x3SIMD simdResult;
                Matrix3x3SIMD.Multiply(ref simdA, ref simdB, out simdResult);
                Matrix3x3 scalarResult;
                Matrix3x3.Multiply(ref scalarA, ref scalarB, out scalarResult);
                var simdPointerResult = (float*)&simdResult;
                var scalarPointerResult = (float*)&scalarResult;

                for (int i = 0; i < 9; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerResult[i] - scalarPointerResult[i]);
                    if (simdScalarError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                Matrix3x3SIMD simdA, simdB;
                Matrix3x3 scalarA, scalarB;
                var simdPointerA = (float*)&simdA;
                var scalarPointerA = (float*)&scalarA;
                var simdPointerB = (float*)&simdB;
                var scalarPointerB = (float*)&scalarB;
                for (int i = 0; i < 9; ++i)
                {
                    scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
                }

                Matrix3x3SIMD.Multiply(ref simdA, ref simdB, out simdA);
                Matrix3x3.Multiply(ref scalarA, ref scalarB, out scalarA);


                for (int i = 0; i < 9; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerA[i] - scalarPointerA[i]);
                    if (simdScalarError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                Matrix3x3SIMD simdA, simdB;
                Matrix3x3 scalarA, scalarB;
                var simdPointerA = (float*)&simdA;
                var scalarPointerA = (float*)&scalarA;
                var simdPointerB = (float*)&simdB;
                var scalarPointerB = (float*)&scalarB;
                for (int i = 0; i < 9; ++i)
                {
                    scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
                    scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
                }

                Matrix3x3SIMD.Multiply(ref simdA, ref simdB, out simdB);
                Matrix3x3.Multiply(ref scalarA, ref scalarB, out scalarB);


                for (int i = 0; i < 9; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointerB[i] - scalarPointerB[i]);
                    if (simdScalarError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }


            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                Matrix3x3SIMD simd;
                Matrix3x3 scalar;
                var simdPointer = (float*)&simd;
                var scalarPointer = (float*)&scalar;
                for (int i = 0; i < 9; ++i)
                {
                    scalarPointer[i] = simdPointer[i] = (float)(random.NextDouble() * 4 - 2);
                }

                Matrix3x3SIMD.Multiply(ref simd, ref simd, out simd);
                Matrix3x3.Multiply(ref scalar, ref scalar, out scalar);

                for (int i = 0; i < 9; ++i)
                {
                    const float threshold = 1e-5f;
                    var simdScalarError = Math.Abs(simdPointer[i] - scalarPointer[i]);
                    if (simdScalarError > threshold)
                    {
                        Console.WriteLine($"Excess error for {i}");
                    }
                }
            }
        }

        public static float TestSIMDTranspose(int iterationCount)
        {
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3SIMD r0, r1;
                Matrix3x3SIMD.Transpose(ref m, out r0);
                Matrix3x3SIMD.Transpose(ref r0, out r1);
                Matrix3x3SIMD.Transpose(ref r1, out r0);
                Matrix3x3SIMD.Transpose(ref r0, out r1);
                Matrix3x3SIMD.Transpose(ref r1, out r0);
                Matrix3x3SIMD.Transpose(ref r0, out r1);
                Matrix3x3SIMD.Transpose(ref r1, out r0);
                Matrix3x3SIMD.Transpose(ref r0, out r1);
                Matrix3x3SIMD.Transpose(ref r1, out r0);
                Matrix3x3SIMD.Transpose(ref r0, out r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        public unsafe static float TestSIMDScalarPointerTranspose(int iterationCount)
        {
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3SIMD r0, r1;
                Matrix3x3SIMD.Transpose(&m, &r0);
                Matrix3x3SIMD.Transpose(&r0, &r1);
                Matrix3x3SIMD.Transpose(&r1, &r0);
                Matrix3x3SIMD.Transpose(&r0, &r1);
                Matrix3x3SIMD.Transpose(&r1, &r0);
                Matrix3x3SIMD.Transpose(&r0, &r1);
                Matrix3x3SIMD.Transpose(&r1, &r0);
                Matrix3x3SIMD.Transpose(&r0, &r1);
                Matrix3x3SIMD.Transpose(&r1, &r0);
                Matrix3x3SIMD.Transpose(&r0, &r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        public static float TestScalarTranspose(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Transpose(ref m, out r0);
                Matrix3x3.Transpose(ref r0, out r1);
                Matrix3x3.Transpose(ref r1, out r0);
                Matrix3x3.Transpose(ref r0, out r1);
                Matrix3x3.Transpose(ref r1, out r0);
                Matrix3x3.Transpose(ref r0, out r1);
                Matrix3x3.Transpose(ref r1, out r0);
                Matrix3x3.Transpose(ref r0, out r1);
                Matrix3x3.Transpose(ref r1, out r0);
                Matrix3x3.Transpose(ref r0, out r1);
                accumulator += r1.M11;

            }
            return accumulator;
        }


        public static float TestScalarInvert(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Invert(ref m, out r0);
                Matrix3x3.Invert(ref r0, out r1);
                Matrix3x3.Invert(ref r1, out r0);
                Matrix3x3.Invert(ref r0, out r1);
                Matrix3x3.Invert(ref r1, out r0);
                Matrix3x3.Invert(ref r0, out r1);
                Matrix3x3.Invert(ref r1, out r0);
                Matrix3x3.Invert(ref r0, out r1);
                Matrix3x3.Invert(ref r1, out r0);
                Matrix3x3.Invert(ref r0, out r1);
                accumulator += r1.M11;

            }
            return accumulator;
        }
        public static float TestSIMDInvert(int iterationCount)
        {
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3SIMD r0, r1;
                Matrix3x3SIMD.Invert(ref m, out r0);
                Matrix3x3SIMD.Invert(ref r0, out r1);
                Matrix3x3SIMD.Invert(ref r1, out r0);
                Matrix3x3SIMD.Invert(ref r0, out r1);
                Matrix3x3SIMD.Invert(ref r1, out r0);
                Matrix3x3SIMD.Invert(ref r0, out r1);
                Matrix3x3SIMD.Invert(ref r1, out r0);
                Matrix3x3SIMD.Invert(ref r0, out r1);
                Matrix3x3SIMD.Invert(ref r1, out r0);
                Matrix3x3SIMD.Invert(ref r0, out r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }
        public unsafe static float TestSIMDScalarInvert(int iterationCount)
        {
            Matrix3x3SIMD m = Matrix3x3SIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3SIMD r0, r1;
                Matrix3x3SIMD.Invert(&m, &r0);
                Matrix3x3SIMD.Invert(&r0, &r1);
                Matrix3x3SIMD.Invert(&r1, &r0);
                Matrix3x3SIMD.Invert(&r0, &r1);
                Matrix3x3SIMD.Invert(&r1, &r0);
                Matrix3x3SIMD.Invert(&r0, &r1);
                Matrix3x3SIMD.Invert(&r1, &r0);
                Matrix3x3SIMD.Invert(&r0, &r1);
                Matrix3x3SIMD.Invert(&r1, &r0);
                Matrix3x3SIMD.Invert(&r0, &r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        public unsafe static void TestInversionCorrectness()
        {
            Random random = new Random(5);
            for (int iterationIndex = 0; iterationIndex < 1000; ++iterationIndex)
            {
                Matrix3x3 scalar;
                Matrix3x3SIMD simd;
                Matrix3x3SIMD simdScalar;
                var scalarPointer = (float*)&scalar;
                var simdPointer = (float*)&simd;
                var simdScalarPointer = (float*)&simdScalar;

                //Create a guaranteed invertible matrix.
                scalar = Matrix3x3.CreateFromAxisAngle(
                    Vector3.Normalize(new Vector3(
                        0.1f + (float)random.NextDouble(),
                        0.1f + (float)random.NextDouble(),
                        0.1f + (float)random.NextDouble())),
                    (float)random.NextDouble());

                for (int i = 0; i < 9; ++i)
                {
                    simdScalarPointer[i] = simdPointer[i] = scalarPointer[i];
                }


                Matrix3x3.Invert(ref scalar, out scalar);
                Matrix3x3SIMD.Invert(ref simd, out simd);
                Matrix3x3SIMD.Invert(&simdScalar, &simdScalar);

                for (int i = 0; i < 9; ++i)
                {
                    var errorSimd = Math.Abs(simdPointer[i] - scalarPointer[i]);
                    var errorSimdScalar = Math.Abs(simdScalarPointer[i] - scalarPointer[i]);
                    Assert.IsTrue(errorSimd < 1e-5f);
                    Assert.IsTrue(errorSimdScalar < 1e-5f);
                }
            }
        }


        public unsafe static void Test()
        {
            Console.WriteLine("MATRIX3x3 RESULTS:");
            Console.WriteLine($"Size: {sizeof(Matrix3x3SIMD)}");
            TestMultiplyCorrectness();
            TestInversionCorrectness();
            const int iterationCount = 10000000;

            Helper.Test("Invert SIMD", TestSIMDInvert, iterationCount);
            Helper.Test("Invert SIMDScalar", TestSIMDScalarInvert, iterationCount);
            Helper.Test("Invert Scalar", TestScalarInvert, iterationCount);

            Helper.Test("Transpose SIMD", TestSIMDTranspose, iterationCount);
            Helper.Test("Transpose SIMDscalarpointer", TestSIMDScalarPointerTranspose, iterationCount);
            Helper.Test("Transpose Scalar", TestScalarTranspose, iterationCount);

            Helper.Test("Multiply SIMD", TestSIMDMultiply, iterationCount);
            Helper.Test("Multiply Scalar", TestScalarMultiply, iterationCount);

            Helper.Test("Transform SIMD", TestSIMDTransform, iterationCount);
            Helper.Test("TransformTranspose SIMD", TestSIMDTransformTranspose, iterationCount);
            Helper.Test("Transform Scalar", TestTransformScalar, iterationCount);

        }
    }
}
