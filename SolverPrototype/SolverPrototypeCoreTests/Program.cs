using System;

namespace SolverPrototypeCoreTests
{
    class Program
    {
        static void Main(string[] args)
        {
            //var coreAssemblyInfo = System.Diagnostics.FileVersionInfo.GetVersionInfo(typeof(object).Assembly.Location);
            Console.WriteLine($"Core Hi");
            //AutoTester.Test();
            LocalsinitCodegen.Test();
        }
    }
}