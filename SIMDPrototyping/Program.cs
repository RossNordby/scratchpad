using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using SIMDPrototyping.Tests;
using SIMDPrototyping.Trees.Tests;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    class Program
    {

        static void Main()
        {
            Console.WriteLine("Vector hardware acceleration: " + Vector.IsHardwareAccelerated);

            //TreeTest.Test();

            //Console.ReadKey();


            int[] targetArray = new int[10000000];
            QuickQueue<int> queue = new QuickQueue<int>(BufferPools<int>.Thread);
            Random random = new Random(5);
            for (int i = 0; i < 262144; ++i)
            {
                for (int j = 0; j < 100; ++j)
                {
                    if (random.NextDouble() > 0.48)
                    {
                        if (queue.Count > 0)
                            queue.Dequeue();
                    }
                    else
                    {
                        queue.Enqueue(i);
                    }
                }
                //for (int j = 0; j < 2; ++j)
                //{
                //    queue.Enqueue(i);
                //}
                //for (int j = 0; j < 1; ++j)
                //{
                //    queue.Dequeue();
                //}
                queue.CopyTo(targetArray, 0);
                for (int j = 0; j < queue.Count; ++j)
                {
                    if (targetArray[j] != queue[j])
                    {
                        Console.WriteLine("Bug");
                    }
                }
            }


        }


    }
}
