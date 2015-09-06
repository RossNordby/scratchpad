using BEPUphysics.BroadPhaseSystems.Hierarchies;
using BEPUphysics.DataStructures;
using BEPUutilities;
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
        public static void TestDH(TestCollidableBEPU[] leaves, BEPUutilities.BoundingBox[] queries, ref BoundingBox positionBounds, 
            int queryCount, int selfTestCount, int refitCount, int frameCount, float dt)
        {

            GC.Collect();
            {

                DynamicHierarchy tree = new DynamicHierarchy();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Add(leaves[i]);
                }
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Remove(leaves[i]);
                }
            }
            GC.Collect();

            {

                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                DynamicHierarchy tree = new DynamicHierarchy();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Add(leaves[i]);
                }

                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"DH Build Time: {endTime - startTime}");
                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");

                tree.SingleThreadedRefitPhase();
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < refitCount; ++i)
                {
                    tree.SingleThreadedRefitPhase();
                    //Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"DH Refit Time: {endTime - startTime}");
                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");

                tree.SingleThreadedOverlapPhase();
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    tree.Overlaps.Clear();
                    tree.SingleThreadedOverlapPhase();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"DH selftest Time: {endTime - startTime}, overlaps: {tree.Overlaps.Count}");


                //**************** Dynamic Testing
                Random random = new Random(5);
               
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int t = 0; t < frameCount; ++t)
                {
                    //Update the positions of objects.
                    for (int i = 0; i < leaves.Length; ++i)
                    {
                        var leaf = leaves[i];

                        //Bounce off the walls.
                        if (leaf.Position.X < positionBounds.Min.X && leaf.Velocity.X < 0)
                            leaf.Velocity.X = -leaf.Velocity.X;
                        if (leaf.Position.Y < positionBounds.Min.Y && leaf.Velocity.Y < 0)
                            leaf.Velocity.Y = -leaf.Velocity.Y;
                        if (leaf.Position.Z < positionBounds.Min.Z && leaf.Velocity.Z < 0)
                            leaf.Velocity.Z = -leaf.Velocity.Z;

                        if (leaf.Position.X > positionBounds.Max.X && leaf.Velocity.X > 0)
                            leaf.Velocity.X = -leaf.Velocity.X;
                        if (leaf.Position.Y > positionBounds.Max.Y && leaf.Velocity.Y > 0)
                            leaf.Velocity.Y = -leaf.Velocity.Y;
                        if (leaf.Position.Z > positionBounds.Max.Z && leaf.Velocity.Z > 0)
                            leaf.Velocity.Z = -leaf.Velocity.Z;

                        leaf.Position += leaf.Velocity * dt;
                        leaf.UpdateBoundingBox();
                    }
                    var refineStartTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;


                   tree.SingleThreadedRefitPhase();


                    //tree.Refit();
                    //for (int i = 0; i < 1; ++i)
                    //{

                    //    subtreeReferences.Count = 0;
                    //    treeletInternalNodes.Count = 0;
                    //    tree.BinnedRefine(0, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                    //}
                    //tree.RemoveUnusedInternalNodes(ref spareNodes);

                    var refineEndTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    tree.Overlaps.Count = 0;
                    tree.SingleThreadedOverlapPhase();

                    var testEndTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    if (t % 16 == 0)
                    {
                        Console.WriteLine($"_________________{t}_________________");
                        Console.WriteLine($"Refine time:      {refineEndTime - refineStartTime}");
                        Console.WriteLine($"Test time:        {testEndTime - refineEndTime}");
                        Console.WriteLine($"TIME:             {testEndTime - refineStartTime}");
                        Console.WriteLine($"Cost metric:      {tree.MeasureCostMetric()}");
                        Console.WriteLine($"Overlaps:         {tree.Overlaps.Count}");
                        GC.Collect();
                    }
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");

                tree.Overlaps.Clear();
                tree.SingleThreadedOverlapPhase();
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    tree.Overlaps.Clear();
                    tree.SingleThreadedOverlapPhase();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"DH selftest Time2: {endTime - startTime}, overlaps: {tree.Overlaps.Count}");

            }

        }
    }
}
