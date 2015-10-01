using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Tests
{
    public class TestResults
    {
        public string Description;
        public double[] Refine;
        public double[] SelfTest;
        public double[] Total;

        public int[] OverlapCounts;
        public float[] TreeCosts;

        public TestResults(string description, int frameCount)
        {
            Description = description;
            Refine = new double[frameCount];
            SelfTest = new double[frameCount];
            Total = new double[frameCount];
            OverlapCounts = new int[frameCount];
            TreeCosts = new float[frameCount];
        }

        public void Save(TextWriter textWriter)
        {
            textWriter.WriteLine(Description);
            textWriter.WriteLine("Refine Times:");
            for (int i = 0; i < Refine.Length; ++i)
            {
                textWriter.WriteLine(Refine[i]);
            }
            textWriter.WriteLine("Self Test Times:");
            for (int i = 0; i < SelfTest.Length; ++i)
            {
                textWriter.WriteLine(SelfTest[i]);
            }
            textWriter.WriteLine("Total Times:");
            for (int i = 0; i < Total.Length; ++i)
            {
                textWriter.WriteLine(Total[i]);
            }
            textWriter.WriteLine("Overlap Counts:");
            for (int i = 0; i < OverlapCounts.Length; ++i)
            {
                textWriter.WriteLine(OverlapCounts[i]);
            }
            textWriter.WriteLine("Tree Costs:");
            for (int i = 0; i < TreeCosts.Length; ++i)
            {
                textWriter.WriteLine(TreeCosts[i]);
            }
        }
    }
}
