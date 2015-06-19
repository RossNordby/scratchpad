using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using BEPUVector3 = BEPUutilities.Vector3;
using BEPUMatrix3x3 = BEPUutilities.Matrix3x3;
using System.Numerics;
using System.Diagnostics;

namespace SIMDPrototyping
{
    class MicroFiddling
    {
        public static void Test()
        {
            Vector3 v1 = new Vector3(0, 0, 1);
            Vector3 v2 = new Vector3(0, 0, 2);
            Vector3 v3 = new Vector3(0, 0, 3);
            Vector3 v4 = new Vector3(0, 0, 4);
            Vector3 v5 = new Vector3(0, 0, 5);
            Vector3 v6 = new Vector3(0, 0, 6);
            Vector3 v7 = new Vector3(0, 0, 7);
            Vector3 v8 = new Vector3(0, 0, 8);
            Matrix3x3 m = new Matrix3x3 { X = new Vector3(1, 0, 0), Y = new Vector3(0, 1, 0), Z = new Vector3(0, 0, 1) };
            Matrix4x4 m4 = new Matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
            Vector3 t = Matrix3x3.Transform(v1, m);
            Matrix3x3.Transform(ref t, ref m, out t);
            Console.WriteLine($"t to preload: {t}");
            double time, endTime;

            const int testIterations = 100000000;

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 refAccumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                Vector3 t1, t2, t3, t4, t5, t6, t7, t8;
                Matrix3x3.Transform(ref v1, ref m, out t1);
                Matrix3x3.Transform(ref v2, ref m, out t2);
                Matrix3x3.Transform(ref v3, ref m, out t3);
                Matrix3x3.Transform(ref v4, ref m, out t4);
                Matrix3x3.Transform(ref v5, ref m, out t5);
                Matrix3x3.Transform(ref v6, ref m, out t6);
                Matrix3x3.Transform(ref v7, ref m, out t7);
                Matrix3x3.Transform(ref v8, ref m, out t8);
                refAccumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;


            Console.WriteLine($"ref time: {endTime - time}, acc: {refAccumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 nonrefAccumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                var t1 = Matrix3x3.Transform(v1, m);
                var t2 = Matrix3x3.Transform(v2, m);
                var t3 = Matrix3x3.Transform(v3, m);
                var t4 = Matrix3x3.Transform(v4, m);
                var t5 = Matrix3x3.Transform(v5, m);
                var t6 = Matrix3x3.Transform(v6, m);
                var t7 = Matrix3x3.Transform(v7, m);
                var t8 = Matrix3x3.Transform(v8, m);
                nonrefAccumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            Console.WriteLine($"nonref time: {endTime - time}, acc: {nonrefAccumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 ref2Accumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                Vector3 t1, t2, t3, t4, t5, t6, t7, t8;
                Matrix3x3.Transform2(ref v1, ref m, out t1);
                Matrix3x3.Transform2(ref v2, ref m, out t2);
                Matrix3x3.Transform2(ref v3, ref m, out t3);
                Matrix3x3.Transform2(ref v4, ref m, out t4);
                Matrix3x3.Transform2(ref v5, ref m, out t5);
                Matrix3x3.Transform2(ref v6, ref m, out t6);
                Matrix3x3.Transform2(ref v7, ref m, out t7);
                Matrix3x3.Transform2(ref v8, ref m, out t8);
                ref2Accumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;


            Console.WriteLine($"ref2 time: {endTime - time}, acc: {ref2Accumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 nonref2Accumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                var t1 = Matrix3x3.Transform2(v1, m);
                var t2 = Matrix3x3.Transform2(v2, m);
                var t3 = Matrix3x3.Transform2(v3, m);
                var t4 = Matrix3x3.Transform2(v4, m);
                var t5 = Matrix3x3.Transform2(v5, m);
                var t6 = Matrix3x3.Transform2(v6, m);
                var t7 = Matrix3x3.Transform2(v7, m);
                var t8 = Matrix3x3.Transform2(v8, m);
                nonref2Accumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            Console.WriteLine($"nonref2 time: {endTime - time}, acc: {nonref2Accumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 refTransposeAccumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                Vector3 t1, t2, t3, t4, t5, t6, t7, t8;
                Matrix3x3.TransformTranspose(ref v1, ref m, out t1);
                Matrix3x3.TransformTranspose(ref v2, ref m, out t2);
                Matrix3x3.TransformTranspose(ref v3, ref m, out t3);
                Matrix3x3.TransformTranspose(ref v4, ref m, out t4);
                Matrix3x3.TransformTranspose(ref v5, ref m, out t5);
                Matrix3x3.TransformTranspose(ref v6, ref m, out t6);
                Matrix3x3.TransformTranspose(ref v7, ref m, out t7);
                Matrix3x3.TransformTranspose(ref v8, ref m, out t8);
                refTransposeAccumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;


            Console.WriteLine($"refTranspose time: {endTime - time}, acc: {refTransposeAccumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 nonrefTransposeAccumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                var t1 = Matrix3x3.TransformTranspose(v1, m);
                var t2 = Matrix3x3.TransformTranspose(v2, m);
                var t3 = Matrix3x3.TransformTranspose(v3, m);
                var t4 = Matrix3x3.TransformTranspose(v4, m);
                var t5 = Matrix3x3.TransformTranspose(v5, m);
                var t6 = Matrix3x3.TransformTranspose(v6, m);
                var t7 = Matrix3x3.TransformTranspose(v7, m);
                var t8 = Matrix3x3.TransformTranspose(v8, m);
                nonrefTransposeAccumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            Console.WriteLine($"nonrefTranspose time: {endTime - time}, acc: {nonrefTransposeAccumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
            Vector3 netAccumulator = new Vector3();
            for (int i = 0; i < testIterations; ++i)
            {
                var t1 = Vector3.TransformNormal(v1, m4);
                var t2 = Vector3.TransformNormal(v2, m4);
                var t3 = Vector3.TransformNormal(v3, m4);
                var t4 = Vector3.TransformNormal(v4, m4);
                var t5 = Vector3.TransformNormal(v5, m4);
                var t6 = Vector3.TransformNormal(v6, m4);
                var t7 = Vector3.TransformNormal(v7, m4);
                var t8 = Vector3.TransformNormal(v8, m4);
                netAccumulator += t1 + t2 + t3 + t4 + t5 + t6 + t7 + t8;
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            Console.WriteLine($"net time: {endTime - time}, acc: {netAccumulator}");

            time = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            var v1b = new BEPUVector3(0, 0, 1);
            var v2b = new BEPUVector3(0, 0, 2);
            var v3b = new BEPUVector3(0, 0, 3);
            var v4b = new BEPUVector3(0, 0, 4);
            var v5b = new BEPUVector3(0, 0, 5);
            var v6b = new BEPUVector3(0, 0, 6);
            var v7b = new BEPUVector3(0, 0, 7);
            var v8b = new BEPUVector3(0, 0, 8);
            BEPUMatrix3x3 mb = new BEPUMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
            BEPUVector3 bepuAccumulator = new BEPUVector3();
            for (int i = 0; i < testIterations; ++i)
            {
                BEPUVector3 t1, t2, t3, t4, t5, t6, t7, t8;
                BEPUMatrix3x3.Transform(ref v1b, ref mb, out t1);
                BEPUMatrix3x3.Transform(ref v2b, ref mb, out t2);
                BEPUMatrix3x3.Transform(ref v3b, ref mb, out t3);
                BEPUMatrix3x3.Transform(ref v4b, ref mb, out t4);
                BEPUMatrix3x3.Transform(ref v5b, ref mb, out t5);
                BEPUMatrix3x3.Transform(ref v6b, ref mb, out t6);
                BEPUMatrix3x3.Transform(ref v7b, ref mb, out t7);
                BEPUMatrix3x3.Transform(ref v8b, ref mb, out t8);
                BEPUVector3.Add(ref bepuAccumulator, ref t1, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t2, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t3, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t4, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t5, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t6, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t7, out bepuAccumulator);
                BEPUVector3.Add(ref bepuAccumulator, ref t8, out bepuAccumulator);
            }
            endTime = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

            Console.WriteLine($"bepu time: {endTime - time}, acc: {bepuAccumulator}");


        }

        public static void Cross()
        {
            Vector3 a = new Vector3(1, 0, 0);
            Vector3 b = new Vector3(0, 1, 0);
            Vector3 crossCustom;
            Vector3Ex.Cross(ref a, ref b, out crossCustom);

            const int iterations = 100000000;
            var time = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < iterations; ++i)
            {
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
                Vector3Ex.Cross(ref a, ref crossCustom, out crossCustom);
            }
            var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;


            Console.WriteLine($"Cross custom: {endTime - time}, {crossCustom}");

            var cross = Vector3.Cross(a, b);
            time = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < iterations; ++i)
            {
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
                cross = Vector3.Cross(a, cross);
            }
            endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            Console.WriteLine($"Cross: {endTime - time}, {cross}");
            //MatrixFiddling.Test();
        }
    }
}
