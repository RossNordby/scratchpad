﻿using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        public class MultithreadedIntertreeTest<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
        {
            struct Job
            {
                public int A;
                public int B;
            }

            public BufferPool Pool;

            int NextNodePair;
            int leafThreshold;
            private QuickList<Job, Buffer<Job>> jobs;
            public int JobCount => jobs.Count;
            public Tree TreeA;
            public Tree TreeB;
            public TOverlapHandler[] OverlapHandlers;

            public MultithreadedIntertreeTest(BufferPool pool)
            {
                Pool = pool;
            }

            /// <summary>
            /// Prepares the jobs associated with a self test. Must be called before a dispatch over PairTest.
            /// </summary>
            /// <param name="overlapHandlers">Callbacks used to handle individual overlaps detected by the self test.</param>
            /// <param name="threadCount">Number of threads to prepare jobs for.</param>
            public void PrepareJobs(Tree treeA, Tree treeB, TOverlapHandler[] overlapHandlers, int threadCount)
            {
                Debug.Assert(OverlapHandlers.Length >= threadCount);
                const float jobMultiplier = 1.5f;
                var targetJobCount = Math.Max(1, jobMultiplier * threadCount);
                leafThreshold = (int)(tree.leafCount / targetJobCount);
                QuickList<Job, Buffer<Job>>.Create(Pool.SpecializeFor<Job>(), (int)(targetJobCount * 2), out jobs);
                NextNodePair = -1;
                this.OverlapHandlers = overlapHandlers;
                this.TreeA = treeA;
                this.TreeB = treeB;
                //Collect jobs.
                CollectJobsInNode(0, tree.leafCount, ref OverlapHandlers[0]);
            }

            /// <summary>
            /// Cleans up after a multithreaded self test.
            /// </summary>
            public void CompleteSelfTest()
            {
                jobs.Dispose(Pool.SpecializeFor<Job>());
            }

            public unsafe void ExecuteJob(int jobIndex, int workerIndex)
            {
                ref var overlap = ref jobs[jobIndex];
                if (overlap.A >= 0)
                {
                    if (overlap.B >= 0)
                    {
                        //Different internal nodes.
                        TreeA.GetOverlapsBetweenDifferentNodes(TreeA.nodes + overlap.A, TreeB.nodes + overlap.B, TreeB, ref OverlapHandlers[workerIndex]);
                    }
                    else
                    {
                        //A is an internal node, B is a leaf.
                        var leafIndex = Encode(overlap.B);
                        var leaf = TreeB.leaves + leafIndex;
                        ref var childOwningLeaf = ref (&TreeB.nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                        TreeB.TestLeafAgainstNode(leafIndex, ref childOwningLeaf.Min, ref childOwningLeaf.Max, overlap.A, TreeA, ref OverlapHandlers[workerIndex]);
                    }
                }
                else
                {
                    //A is a leaf, B is internal.
                    var leafIndex = Encode(overlap.A);
                    var leaf = TreeA.leaves + leafIndex;
                    ref var childOwningLeaf = ref (&TreeA.nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                    TreeA.TestLeafAgainstNode(leafIndex, ref childOwningLeaf.Min, ref childOwningLeaf.Max, overlap.B, TreeB, ref OverlapHandlers[workerIndex]);

                    //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                    //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
                }
            }
            /// <summary>
            /// Executes a single worker of the multithreaded self test.
            /// </summary>
            /// <param name="workerIndex">Index of the worker executing this set of tests.</param>
            public unsafe void PairTest(int workerIndex)
            {
                Debug.Assert(workerIndex >= 0 && workerIndex < OverlapHandlers.Length);
                int nextNodePairIndex;
                //To minimize the number of worker overlap lists, perform direct load balancing by manually grabbing the next indices.
                while ((nextNodePairIndex = Interlocked.Increment(ref NextNodePair)) < jobs.Count)
                {
                    ExecuteJob(nextNodePairIndex, workerIndex);
                }
            }

            unsafe void DispatchTestForLeaf(Tree nodeOwner, int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, int nodeLeafCount, ref TOverlapHandler results)
            {
                if (nodeIndex < 0)
                {
                    //Maintain the order of trees. Leaves from tree A should always be the first parameter.
                    if (nodeOwner == TreeA)
                        results.Handle(Encode(nodeIndex), leafIndex);
                    else
                        results.Handle(leafIndex, Encode(nodeIndex));
                }
                else
                {
                    if (nodeLeafCount <= leafThreshold)
                    {
                        //Maintain the order of trees. Leaves from tree A should always be the first parameter.
                        if (nodeOwner == TreeA)
                            jobs.Add(new Job { B = Encode(leafIndex), A = nodeIndex }, Pool.SpecializeFor<Job>());
                        else
                            jobs.Add(new Job { A = Encode(leafIndex), B = nodeIndex }, Pool.SpecializeFor<Job>());
                    }
                    else
                        TestLeafAgainstNode(nodeOwner, leafIndex, ref leafMin, ref leafMax, nodeIndex, ref results);
                }
            }

            unsafe void TestLeafAgainstNode(Tree nodeOwner, int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, ref TOverlapHandler results)
            {
                var node = nodeOwner.nodes + nodeIndex;
                ref var a = ref node->A;
                ref var b = ref node->B;
                //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
                //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
                //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
                //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
                var bIndex = b.Index;
                var bLeafCount = b.LeafCount;
                var aIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref a.Min, ref a.Max);
                var bIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref b.Min, ref b.Max);
                if (aIntersects)
                {
                    DispatchTestForLeaf(nodeOwner, leafIndex, ref leafMin, ref leafMax, a.Index, a.LeafCount, ref results);
                }
                if (bIntersects)
                {
                    DispatchTestForLeaf(nodeOwner, leafIndex, ref leafMin, ref leafMax, bIndex, bLeafCount, ref results);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void DispatchTestForNodes(ref NodeChild a, ref NodeChild b, ref TOverlapHandler results)
            {
                if (a.Index >= 0)
                {
                    if (b.Index >= 0)
                    {
                        if (a.LeafCount + b.LeafCount <= leafThreshold)
                            jobs.Add(new Job { A = a.Index, B = b.Index }, Pool.SpecializeFor<Job>());
                        else
                            GetJobsBetweenDifferentNodes(TreeA.nodes + a.Index, TreeB.nodes + b.Index, ref results);

                    }
                    else
                    {
                        //leaf B versus node A.
                        TestLeafAgainstNode(TreeA, Encode(b.Index), ref b.Min, ref b.Max, a.Index, ref results);
                    }
                }
                else if (b.Index >= 0)
                {
                    //leaf A versus node B.
                    TestLeafAgainstNode(TreeB, Encode(a.Index), ref a.Min, ref a.Max, b.Index, ref results);
                }
                else
                {
                    //Two leaves.
                    results.Handle(Encode(a.Index), Encode(b.Index));
                }
            }

            unsafe void GetJobsBetweenDifferentNodes(Node* a, Node* b, ref TOverlapHandler results)
            {
                //There are no shared children, so test them all.

                ref var aa = ref a->A;
                ref var ab = ref a->B;
                ref var ba = ref b->A;
                ref var bb = ref b->B;
                var aaIntersects = Intersects(ref aa, ref ba);
                var abIntersects = Intersects(ref aa, ref bb);
                var baIntersects = Intersects(ref ab, ref ba);
                var bbIntersects = Intersects(ref ab, ref bb);

                if (aaIntersects)
                {
                    DispatchTestForNodes(ref aa, ref ba, ref results);
                }
                if (abIntersects)
                {
                    DispatchTestForNodes(ref aa, ref bb, ref results);
                }
                if (baIntersects)
                {
                    DispatchTestForNodes(ref ab, ref ba, ref results);
                }
                if (bbIntersects)
                {
                    DispatchTestForNodes(ref ab, ref bb, ref results);
                }

            }
        }
    }
}
