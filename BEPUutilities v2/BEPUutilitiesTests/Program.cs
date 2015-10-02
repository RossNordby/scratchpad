using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace BEPUutilitiesTests
{
    class Program
    {
        static void Main(string[] args)
        {
            const int iterationCount = 1000000;
            Helper.Test("Scalar Vector", MathPerformanceTests.TestScalarMatrix, iterationCount);
            Helper.Test("System Vector", MathPerformanceTests.TestSystemMatrix, iterationCount);
            Helper.Test("Row Vector SIMD", MathPerformanceTests.TestSIMDMatrixRowVector, iterationCount);
            Helper.Test("Column Vector SIMD", MathPerformanceTests.TestSIMDMatrixColumnVector, iterationCount);
        }
    }
}
