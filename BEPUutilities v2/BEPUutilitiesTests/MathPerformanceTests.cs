using BEPUutilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    public static class MathPerformanceTests
    {

        public static float TestScalarMatrix(int iterationCount)
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
        public static float TestSystemMatrix(int iterationCount)
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
        public static float TestSIMDMatrixRowVector(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            MatrixSIMD m = MatrixSIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                MatrixSIMD.TransformRowVector(ref v, ref m, out r0);
                MatrixSIMD.TransformRowVector(ref r0, ref m, out r1);
                MatrixSIMD.TransformRowVector(ref r1, ref m, out r0);
                MatrixSIMD.TransformRowVector(ref r0, ref m, out r1);
                MatrixSIMD.TransformRowVector(ref r1, ref m, out r0);
                MatrixSIMD.TransformRowVector(ref r0, ref m, out r1);
                MatrixSIMD.TransformRowVector(ref r1, ref m, out r0);
                MatrixSIMD.TransformRowVector(ref r0, ref m, out r1);
                MatrixSIMD.TransformRowVector(ref r1, ref m, out r0);
                MatrixSIMD.TransformRowVector(ref r0, ref m, out r1);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }

        public static float TestSIMDMatrixColumnVector(int iterationCount)
        {
            Vector4 v = new Vector4(1, 2, 3, 4);
            MatrixSIMD m = MatrixSIMD.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector4 r0, r1;
                MatrixSIMD.TransformColumnVector(ref m, ref v, out r0);
                MatrixSIMD.TransformColumnVector(ref m, ref r0, out r1);
                MatrixSIMD.TransformColumnVector(ref m, ref r1, out r0);
                MatrixSIMD.TransformColumnVector(ref m, ref r0, out r1);
                MatrixSIMD.TransformColumnVector(ref m, ref r1, out r0);
                MatrixSIMD.TransformColumnVector(ref m, ref r0, out r1);
                MatrixSIMD.TransformColumnVector(ref m, ref r1, out r0);
                MatrixSIMD.TransformColumnVector(ref m, ref r0, out r1);
                MatrixSIMD.TransformColumnVector(ref m, ref r1, out r0);
                MatrixSIMD.TransformColumnVector(ref m, ref r0, out r1);
                accumulator += 0.000001f * r1.LengthSquared();
            }
            return accumulator;
        }
    }
}
