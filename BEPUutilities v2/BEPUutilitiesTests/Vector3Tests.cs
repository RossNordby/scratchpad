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
    [TestClass]
    public static class Vector3Tests
    {
         public static float TestTransformScalar(int iterationCount)
        {
            Vector3 v1 = new Vector3(1, 2, 3);
            Vector3 v2 = new Vector3(1, 2, 3);
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Vector3x.Cross(ref v1, ref v2, out r0);
                Vector3x.Cross(ref r0, ref v2, out r1);
                Vector3x.Cross(ref r1, ref v2, out r0);
                Vector3x.Cross(ref r0, ref v2, out r1);
                Vector3x.Cross(ref r1, ref v2, out r0);
                Vector3x.Cross(ref r0, ref v2, out r1);
                Vector3x.Cross(ref r1, ref v2, out r0);
                Vector3x.Cross(ref r0, ref v2, out r1);
                Vector3x.Cross(ref r1, ref v2, out r0);
                Vector3x.Cross(ref r0, ref v2, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        public static void Test()
        {
            const int iterationCount = 10000000;
            Helper.Test("Cross Scalar", TestTransformScalar, iterationCount);
        }
    }
}
