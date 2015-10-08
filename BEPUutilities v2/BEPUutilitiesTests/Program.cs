using BEPUutilities2.DataStructures;
using BEPUutilities2.ResourceManagement;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace BEPUutilitiesTests
{
    class Program
    {
        static void Main(string[] args)
        {
            BoundingTests.Test();
            Console.WriteLine();
            AffineTests.Test();
            Console.WriteLine();
            Vector3Tests.Test();
            Console.WriteLine();
            Matrix3x3Tests.Test();
            Console.WriteLine();
            Matrix4x4Tests.Test();

        }

    }
}
