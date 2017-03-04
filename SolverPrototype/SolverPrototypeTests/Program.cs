using SolverPrototype;
using System;
using System.Numerics;

namespace SolverPrototypeTests
{
    class Program
    {
        static void Main(string[] args)
        {
            //VectorAccessTests.Test();
            //GatherScatterTests.Test();
            //LayoutTests.Test();
            //ContactPenetration1DOFTests.Test();
            //PenetrationConvergenceTests.Test();
            ContactManifoldConvergenceTests.Test();
            //SortTest.Test();
            //IslandCacheConvergence.Test();
            //SuballocationTests.Test();
        }
    }
}
