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
        //TODO: 
        //There are a some issues inherited from the prototype that we'd like to address at some point:
        //1) Recursion. There's no reason to use recursion here.
        //2) Duplicate work with the single threaded variant. The current load balancing approach uses a single threaded pass to dive into the tree, and that logic
        //is basically identical. It would be great to have a zero overhead abstraction that unifies the two. Unclear how useful this is- it's possible that the abstraction
        //would end up being more complex than just two near-identical implementations.
        //3) Limited workstealing capacity. While we can dive arbitrarily far in the first pass, it increases the single threaded phase.
        //If the narrow phase relies on the broadphase for its work balancing (that is, the overlap handler directly triggers narrow phase work), 
        //you may need to dive so deeply to maintain load balance that the single threaded phase starts to limit parallelism meaningfully. 
        //Any constant cost less than ~5us is basically irrelevant, though- if you can collect 128 nodepairs to test in 5us, that would likely be enough to load balance the narrow phase
        //even on something like 16 cores. 
        //4) If the handler directly executes narrow phase work, overlaps handled during the single threaded collection phase could be nasty. This should be pretty rare for any nontrivial
        //tree, but it's still something to be aware of in corner cases.

        //To specifically address #3 above, consider explicit workstealing. When a worker is out of directly accessible work (its exhausted its own stack, and no more precollected roots exist),
        //it could snoop other worker stacks. This would introduce sync requirements on every stack. 
        //1) The stealer would probably start at claim 0 and walk forward. The largest jobs are at the top of the stack, which gives you the most bang for the sync work buck.
        //It would check the claims state of each stack entry- there would be a integer on each entry marking it as claimed or not. Once a candidate is found, compare exchange to claim it.
        //It would have to distinguish between 'stolen' blocks and locally claimed blocks. A thief can step over stolen blocks, but if it hits a locally claimed block, it has to stop.
        //2) While pushing new jobs to the local stack is free, victims must always check to confirm that a stack pop will not consume a job that has been stolen by another thread.
        //Given that shallow stack accesses will tend to be less work, the local thread should probably prefer claiming chunks of its stack at a time. It can do this simply by 
        //performing a compare exchange on a stack element the desired number of elements up the stack. Since thieves always work step by step without leaving any gaps, the local thread
        //can block them by claiming at any (unclaimed) point in the stack. All later stack entries can be unaffected. In practice, this means local threads should be able to 
        //avoid doing interlocked operations on the overwhelming majority of pop operations.

        //With such a scheme, you would still want to somehow collect an initial set of jobs to give workers something to munch on, but you don't need lots of jobs per worker anymore.
        //So, if you had a 128 core machine, you could get away with still having ~256 jobs- which you can probably collect in less than 20us even on lower frequency processors 
        //(like the ones you'd find in a 128 core machine).
        public class MultithreadedSelfTest<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
        {
            struct Overlap
            {
                public int A;
                public int B;
            }

            private Tree tree;
            private BufferPool<Overlap> overlapPool;

            int NextNodePair;
            int leafThreshold;
            private QuickList<Overlap, Buffer<Overlap>> nodePairsToTest;
            private Action<int> pairTestAction;

            public MultithreadedSelfTest()
            {
                pairTestAction = PairTest;
            }

            TOverlapHandler[] overlapHandlers;

            public void SelfTest(Tree tree, TOverlapHandler[] overlapHandlers, IThreadDispatcher threadDispatcher)
            {
                //If there are not multiple children, there's no need to recurse.
                //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
                if (tree.leafCount < 2)
                    return;

                Debug.Assert(overlapHandlers.Length >= threadDispatcher.ThreadCount);
                const float jobMultiplier = 1.5f;
                var targetJobCount = Math.Max(1, jobMultiplier * threadDispatcher.ThreadCount);
                leafThreshold = (int)(tree.leafCount / targetJobCount);
                this.overlapHandlers = overlapHandlers;
                this.tree = tree;
                this.overlapPool = tree.Pool.SpecializeFor<Overlap>();
                QuickList<Overlap, Buffer<Overlap>>.Create(tree.Pool.SpecializeFor<Overlap>(), (int)(targetJobCount * 2), out nodePairsToTest);
                NextNodePair = -1;
                //Collect jobs.
                CollectJobsInNode(0, tree.leafCount, ref overlapHandlers[0]);
                //Do the jobs!
                threadDispatcher.DispatchWorkers(pairTestAction);
                nodePairsToTest.Dispose(tree.Pool.SpecializeFor<Overlap>());
                this.tree = null;
                this.overlapPool = default(BufferPool<Overlap>);
                this.overlapHandlers = null;
            }

            unsafe void PairTest(int workerIndex)
            {
                int nextNodePairIndex;
                //To minimize the number of worker overlap lists, perform direct load balancing by manually grabbing the next indices.
                while ((nextNodePairIndex = Interlocked.Increment(ref NextNodePair)) < nodePairsToTest.Count)
                {
                    var overlap = nodePairsToTest[nextNodePairIndex];
                    if (overlap.A >= 0)
                    {
                        if (overlap.A == overlap.B)
                        {
                            //Same node.
                            tree.GetOverlapsInNode2(tree.nodes + overlap.A, ref overlapHandlers[workerIndex]);
                        }
                        else if (overlap.B >= 0)
                        {
                            //Different nodes.
                            tree.GetOverlapsBetweenDifferentNodes2(tree.nodes + overlap.A, tree.nodes + overlap.B, ref overlapHandlers[workerIndex]);
                        }
                        else
                        {
                            //A is an internal node, B is a leaf.
                            var leafIndex = Encode(overlap.B);
                            var leaf = tree.leaves + leafIndex;
                            ref var childOwningLeaf = ref (&tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                            tree.TestLeafAgainstNode2(leafIndex, ref childOwningLeaf.Min, ref childOwningLeaf.Max, overlap.A, ref overlapHandlers[workerIndex]);
                        }
                    }
                    else
                    {
                        //A is a leaf, B is internal.
                        var leafIndex = Encode(overlap.A);
                        var leaf = tree.leaves + leafIndex;
                        ref var childOwningLeaf = ref (&tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                        tree.TestLeafAgainstNode2(leafIndex, ref childOwningLeaf.Min, ref childOwningLeaf.Max, overlap.B, ref overlapHandlers[workerIndex]);

                        //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                        //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
                    }
                }
            }

            unsafe void DispatchTestForLeaf(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, int nodeLeafCount, ref TOverlapHandler results)
            {
                if (nodeIndex < 0)
                {
                    results.Handle(leafIndex, Encode(nodeIndex));
                }
                else
                {
                    if (nodeLeafCount <= leafThreshold)
                        nodePairsToTest.Add(new Overlap { A = Encode(leafIndex), B = nodeIndex }, overlapPool);
                    else
                        TestLeafAgainstNode(leafIndex, ref leafMin, ref leafMax, nodeIndex, ref results);
                }
            }

            unsafe void TestLeafAgainstNode(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, ref TOverlapHandler results)
            {
                var node = tree.nodes + nodeIndex;
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
                    DispatchTestForLeaf(leafIndex, ref leafMin, ref leafMax, a.Index, a.LeafCount, ref results);
                }
                if (bIntersects)
                {
                    DispatchTestForLeaf(leafIndex, ref leafMin, ref leafMax, bIndex, bLeafCount, ref results);
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
                            nodePairsToTest.Add(new Overlap { A = a.Index, B = b.Index }, overlapPool);
                        else
                            GetJobsBetweenDifferentNodes(tree.nodes + a.Index, tree.nodes + b.Index, ref results);

                    }
                    else
                    {
                        //leaf B versus node A.
                        TestLeafAgainstNode(Encode(b.Index), ref b.Min, ref b.Max, a.Index, ref results);
                    }
                }
                else if (b.Index >= 0)
                {
                    //leaf A versus node B.
                    TestLeafAgainstNode(Encode(a.Index), ref a.Min, ref a.Max, b.Index, ref results);
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

            unsafe void CollectJobsInNode(int nodeIndex, int leafCount, ref TOverlapHandler results)
            {
                if (leafCount <= leafThreshold)
                {
                    nodePairsToTest.Add(new Overlap { A = nodeIndex, B = nodeIndex }, overlapPool);
                    return;
                }

                var node = tree.nodes + nodeIndex;
                ref var a = ref node->A;
                ref var b = ref node->B;

                var ab = Intersects(ref a, ref b);

                if (a.Index >= 0)
                    CollectJobsInNode(a.Index, a.LeafCount, ref results);
                if (b.Index >= 0)
                    CollectJobsInNode(b.Index, b.LeafCount, ref results);

                if (ab)
                {
                    DispatchTestForNodes(ref a, ref b, ref results);
                }

            }
        }
    }
}
