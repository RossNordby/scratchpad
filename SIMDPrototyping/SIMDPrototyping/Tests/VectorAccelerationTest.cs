using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Tests
{
    public class VectorAccelerationTest
    {
        public static unsafe void Test()
        {
            const int iterations = 10000000;
            {
                var vectors = stackalloc Vector3[Vector<float>.Count];
                //var vectors = new Vector3[Vector<float>.Count];
                for (int j = 0; j < Vector<float>.Count; ++j)
                {
                    vectors[j] = new Vector3();
                }
                float[] x, y, z;
                x = new float[Vector<float>.Count];
                y = new float[Vector<float>.Count];
                z = new float[Vector<float>.Count];
                Vector3Wide acc = new Vector3Wide(x, y, z);
                Vector3Wide v1 = new Vector3Wide(x, y, z);
                Vector3Wide.Add(ref v1, ref acc, out acc);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < iterations; ++i)
                {
                    for (int j = 0; j < Vector<float>.Count; ++j)
                    {
                        x[j] = vectors[j].X;
                        y[j] = vectors[j].Y;
                        z[j] = vectors[j].Z;
                    }
                    v1 = new Vector3Wide(x, y, z);
                    Vector3Wide.Add(ref v1, ref acc, out acc);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"* Time: {endTime - startTime}, acc: {acc}, size: {Vector<float>.Count}");
            }

            {
                Vector3 v = new Vector3(0, 1, 2);
                Vector3Wide acc = new Vector3Wide(ref v);
                Vector3Wide v1 = new Vector3Wide(ref v);
                Vector3Wide.Add(ref v1, ref acc, out acc);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < iterations; ++i)
                {
                    v1 = new Vector3Wide(ref v);
                    Vector3Wide.Add(ref v1, ref acc, out acc);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"* Single Time: {endTime - startTime}, acc: {acc}, size: {Vector<float>.Count}");
            }

            {
                Vector3 a, b, c, d;

                a = b = c = d = new Vector3();
                Vector3Width4 acc = new Vector3Width4();
                Vector3Width4 v1 = new Vector3Width4(ref a, ref b, ref c, ref d);
                Vector3Width4.Add(ref v1, ref acc, out acc);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < iterations; ++i)
                {
                    v1 = new Vector3Width4(ref a, ref b, ref c, ref d);
                    Vector3Width4.Add(ref v1, ref acc, out acc);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Fixed Time: {endTime - startTime}, acc: {acc}");
            }


            
        }
    }
}
