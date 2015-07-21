using BEPUphysics.DataStructures;
using BEPUutilities.DataStructures;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Tests
{
    partial class TreeTest
    {
        public static void TestBEPU(TestCollidableBEPU[] leaves, BEPUutilities.BoundingBox[] queries, int queryCount, int selfTestCount)
        {

            GC.Collect();
            {
                var warmLeaves = GetLeavesBEPU(2, 2, 2, 10, 10);
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(warmLeaves);
                Console.WriteLine($"BEPU Cachewarm Build, root AABB: {tree.BoundingBox}");

                tree.Refit();

                RawList<TestCollidableBEPU> results = new RawList<TestCollidableBEPU>();
                BEPUutilities.BoundingBox aabb = new BEPUutilities.BoundingBox { Min = new BEPUutilities.Vector3(0, 0, 0), Max = new BEPUutilities.Vector3(1, 1, 1) };

                results.Count = 0;
                tree.GetOverlaps(aabb, results);
            }

            {
                
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Build Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Refit Time: {endTime - startTime}");
                
                RawList<TestCollidableBEPU> results = new RawList<TestCollidableBEPU>();
                var queryMask = queries.Length - 1;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    results.Count = 0;
                    tree.GetOverlaps(queries[i & queryMask], results);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Query Time: {endTime - startTime}, overlaps: {results.Count}");

            }

        }
    }
}
