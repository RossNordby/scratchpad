using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SolverPrototype
{
    class Program
    {
        static void Main(string[] args)
        {
            float[] tester = new float[4];
            Vector<float> vedtor = new Vector<float>(tester);
            var vedtor2 = new Vector<float>(2);
            vedtor += vedtor2;
            for (int i = 0; i < tester.Length; ++i)
                Console.WriteLine(tester[i]);
            for (int i = 0; i < Vector<float>.Count; ++i)
                Console.WriteLine(vedtor[i]);
            
        }
    }
}
