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
        public static void TestBEPU(TestCollidableBEPU[] leaves, BEPUutilities.BoundingBox[] queries, int queryCount, int selfTestCount, int refitCount)
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

                var overlaps = new List<TreeOverlapPair<TestCollidableBEPU, TestCollidableBEPU>>(100);
                for (int i = 0; i < selfTestCount; ++i)
                {
                    tree.GetOverlaps(tree, overlaps);
                }
            }
            GC.Collect();

            {

                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Build Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < refitCount; ++i)
                {
                    tree.Refit();
                }
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

                //var overlaps = new RawList<TreeOverlapPair<TestCollidableBEPU, TestCollidableBEPU>>(280000);
                //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < selfTestCount; ++i)
                //{
                //    overlaps.Count = 0;
                //    tree.GetOverlaps(tree, overlaps);
                //}
                //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //Console.WriteLine($"BEPU SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");
            }

        }
    }
}
