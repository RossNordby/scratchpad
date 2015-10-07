using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using bVector3 = BEPUutilities.Vector3;
using bRay = BEPUutilities.Ray;
using bBoundingBox = BEPUutilities.BoundingBox;
using BEPUutilities2;
using System.Numerics;

namespace BEPUutilitiesTests
{
    [TestClass]
    class BoundingTests
    {
        public static float TestBoxRaySIMD(int iterationCount)
        {
            Ray ray;
            ray.Direction = new Vector3(1, 1, 1);
            ray.Position = new Vector3(-3);
            BoundingBox box;
            box.Min = new Vector3(-1);
            box.Max = new Vector3(1);
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                float t;
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                Ray.Intersects(ref ray, ref box, out t);
                accumulator += t;
            }
            return accumulator;
        }

        public static float TestBoxRayScalar(int iterationCount)
        {
            bRay ray;
            ray.Direction = new bVector3(1, 1, 1);
            ray.Position = new bVector3(-3, -3, -3);
            bBoundingBox box;
            box.Min = new bVector3(-1, -1, -1);
            box.Max = new bVector3(1, 1, 1);
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                float t;
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                ray.Intersects(ref box, out t);
                accumulator += t;
            }
            return accumulator;
        }

        public static bVector3 Convert(Vector3 v)
        {
            return new bVector3(v.X, v.Y, v.Z);
        }

        [TestMethod]
        public unsafe static void TestBoxRayCorrectness()
        {
            Random random = new Random(5);
            int intersectionCount = 0;
            for (int i = 0; i < 10000000; ++i)
            {
                Ray simdRay;
                BoundingBox simdBox;
                bRay scalarRay;
                bBoundingBox scalarBox;

                simdRay.Position = 20 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
                var boxPosition = 5 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
                var target = 5 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
                simdRay.Direction = target - simdRay.Position;

                var width = new Vector3(3 * (float)random.NextDouble());
                simdBox.Min = boxPosition - width;
                simdBox.Max = boxPosition + width;

                simdRay.Position = new Vector3(0, 0, 0);
                simdRay.Direction = new Vector3(1, 0, 0);
                simdBox.Min = new Vector3(0);
                simdBox.Max = new Vector3(1);

                scalarRay.Position = Convert(simdRay.Position);
                scalarRay.Direction = Convert(simdRay.Direction);
                scalarBox.Min = Convert(simdBox.Min);
                scalarBox.Max = Convert(simdBox.Max);

                float simdT;
                var simdIntersects = Ray.Intersects(ref simdRay, ref simdBox, out simdT);

                float scalarT;
                var scalarIntersects = scalarRay.Intersects(ref scalarBox, out scalarT);
                //Console.WriteLine($"Simd says: {simdIntersects}, scalar says {scalarIntersects}");
                Assert.IsTrue(simdIntersects == scalarIntersects);
                if (simdIntersects)
                {
                    var error = Math.Abs(simdT - scalarT);

                    Assert.IsTrue(error <= 1e-5f);
                    intersectionCount++;
                }
                //Just treat T as undefined if it's not intersecting. No need to check it.
            }
            Console.WriteLine($"intersectionCount: {intersectionCount}");
        }


        public static void Repro()
        {
            Vector3 boundingMin = new Vector3(0);
            Vector3 boundingMax = new Vector3(1);
            Vector3 rayPosition = new Vector3(0);
            Vector3 rayDirection = new Vector3(1, 0, 0);

            var positionMin = boundingMin - rayPosition;
            var positionMax = boundingMax - rayPosition;

            Vector3 tMin, tMax;
            tMin = positionMin / rayDirection;
            tMax = positionMax / rayDirection;

            var positiveFilter = new Vector3(float.MaxValue);
            var negativeFilter = new Vector3(float.MinValue);
            //Careful! parameter order matters here- this is designed to deal with NaNs.
            tMin = Vector3.Max(Vector3.Min(positiveFilter, tMin), negativeFilter);
            tMax = Vector3.Max(Vector3.Min(positiveFilter, tMax), negativeFilter);


            Vector3 tEarly = Vector3.Min(tMin, tMax);
            Vector3 tLate = Vector3.Max(tMin, tMax);

            //All intervals from tEarly to tLate must overlap for there to exist an intersection.
            //This would benefit from some more instructions...
            float t = Math.Max(0, Math.Max(Math.Max(tEarly.X, tEarly.Y), tEarly.Z));
            var earliestLate = Math.Min(Math.Min(tLate.X, tLate.Y), tLate.Z);

            //Console.WriteLine($"tMin: {tMin}, tMax: {tMax}");
            Console.WriteLine($"tEarly: {tEarly}, tLate: {tLate}");
            //Console.WriteLine($"t: {t}, earliestLate: {earliestLate}");
            Assert.IsTrue(t <= earliestLate);

        }

        public static void Test()
        {
            Repro();
            //TestBoxRayCorrectness();

            //const int iterations = 1000000;
            //Helper.Test("Box-Ray SIMD", TestBoxRaySIMD, iterations);
            //Helper.Test("Box-Ray Scalar", TestBoxRayScalar, iterations);
        }
    }
}
