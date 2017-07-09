using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using BEPUutilities.Threading;
using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
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

            int NextNodePair;
            private QuickList<Overlap, Buffer<Overlap>> nodePairsToTest;
            private Action<int> pairTestAction;

            public MultithreadedSelfTest()
            {
                pairTestAction = PairTest;
            }

            Buffer<TOverlapHandler> overlapHandlers;

            public void SelfTest(Tree tree, ref Buffer<TOverlapHandler> overlapHandlers, BufferPool pool, IThreadDispatcher threadDispatcher)
            {
                Debug.Assert(overlapHandlers.Allocated && overlapHandlers.Length >= threadDispatcher.ThreadCount);
                const float jobMultiplier = 1.5f;
                var targetJobCount = Math.Max(1, jobMultiplier * threadDispatcher.ThreadCount);
                var leafThreshold = tree.leafCount / targetJobCount;
                this.overlapHandlers = overlapHandlers;
                this.tree = tree;
                QuickList<Overlap, Buffer<Overlap>>.Create(pool.SpecializeFor<Overlap>(), (int)(targetJobCount * 2), out nodePairsToTest);
                threadDispatcher.DispatchWorkers(pairTestAction);
                this.tree = null;
                this.overlapHandlers = default(Buffer<TOverlapHandler>);
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
                            Tree.GetOverlapsInNode2(Tree.nodes + overlap.A, ref WorkerOverlaps[workerIndex]);
                        }
                        else if (overlap.B >= 0)
                        {
                            //Different nodes.
                            Tree.GetOverlapsBetweenDifferentNodes2(Tree.nodes + overlap.A, Tree.nodes + overlap.B, ref WorkerOverlaps[workerIndex]);
                        }
                        else
                        {
                            //A is an internal node, B is a leaf.
                            var leafIndex = Tree.Encode(overlap.B);
                            var leaf = Tree.leaves + leafIndex;
                            Tree.TestLeafAgainstNode2(leafIndex, ref (&Tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex], overlap.A, ref WorkerOverlaps[workerIndex]);
                        }
                    }
                    else
                    {
                        //A is a leaf, B is internal.
                        var leafIndex = Tree.Encode(overlap.A);
                        var leaf = Tree.leaves + leafIndex;
                        Tree.TestLeafAgainstNode2(leafIndex, ref (&Tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex], overlap.B, ref WorkerOverlaps[workerIndex]);

                        //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                        //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
                    }
                }
            }            
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestForCollectNodeLeafPairs<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, int nodeLeafCount, int collisionTestThreshold, ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (nodeIndex < 0)
            {
                //Add directly to the overlaps list; no reason to make a worker thread do it.
                results.Add(new Overlap { A = leafIndex, B = Encode(nodeIndex) });
            }
            else
            {
                if (Math.Log(nodeLeafCount) <= collisionTestThreshold)
                //if (nodeLeafCount <= collisionTestThreshold)
                {
                    nodePairsToTest.Add(new Overlap { A = Encode(leafIndex), B = nodeIndex });
                }
                else
                {
                    CollectNodeLeafPairs(leafIndex, ref leafBounds, nodeIndex, nodeLeafCount, collisionTestThreshold, ref nodePairsToTest, ref results);
                }
            }
        }
        unsafe void CollectNodeLeafPairs<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, int nodeLeafCount, int collisionTestThreshold, ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            var node = nodes + nodeIndex;

            var a = BoundingBox.Intersects(ref leafBounds, ref node->A);
            var b = BoundingBox.Intersects(ref leafBounds, ref node->B);

            var nodeChildA = node->ChildA;
            var nodeChildB = node->ChildB;

            if (a)
            {
                TestForCollectNodeLeafPairs(leafIndex, ref leafBounds, nodeChildA, node->LeafCountA, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
            if (b)
            {
                TestForCollectNodeLeafPairs(leafIndex, ref leafBounds, nodeChildB, node->LeafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestForCollectNodePairs<TResultList>(ref BoundingBox boundsA, ref BoundingBox boundsB, int childA, int childB, int leafCountA, int leafCountB, int collisionTestThreshold,
            ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (childA >= 0)
            {
                if (childB >= 0)
                {
                    if (Math.Max(leafCountA, leafCountB) <= collisionTestThreshold)
                    {
                        nodePairsToTest.Add(new Overlap { A = childA, B = childB });
                    }
                    else
                    {
                        CollectNodePairsFromDifferentNodes(nodes + childA, nodes + childB, collisionTestThreshold, ref nodePairsToTest, ref results);
                    }
                }
                else
                {
                    //leaf B versus node A.
                    TestForCollectNodeLeafPairs(Encode(childB), ref boundsB, childA, leafCountA, collisionTestThreshold, ref nodePairsToTest, ref results);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                TestForCollectNodeLeafPairs(Encode(childA), ref boundsA, childB, leafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
            else
            {
                //Two leaves.
                //Add directly to the overlaps list; no reason to make a worker thread do it.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        unsafe void CollectNodePairsFromDifferentNodes<TResultList>(Node* a, Node* b, int collisionTestThreshold, ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
            var bb = BoundingBox.Intersects(ref a->B, ref b->B);

            if (aa)
            {
                TestForCollectNodePairs(ref a->A, ref b->A, a->ChildA, b->ChildA, a->LeafCountA, b->LeafCountA, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
            if (ab)
            {
                TestForCollectNodePairs(ref a->A, ref b->B, a->ChildA, b->ChildB, a->LeafCountA, b->LeafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
            if (ba)
            {
                TestForCollectNodePairs(ref a->B, ref b->A, a->ChildB, b->ChildA, a->LeafCountB, b->LeafCountA, collisionTestThreshold, ref nodePairsToTest, ref results);
            }
            if (bb)
            {
                TestForCollectNodePairs(ref a->B, ref b->B, a->ChildB, b->ChildB, a->LeafCountB, b->LeafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);
            }

        }

        unsafe void CollectNodePairsInNode<TResultList>(int nodeIndex, int leafCount, int collisionTestThreshold, ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (leafCount <= collisionTestThreshold)
            {
                nodePairsToTest.Add(new Overlap { A = nodeIndex, B = nodeIndex });
                return;
            }
            var node = nodes + nodeIndex;
            var nodeChildA = node->ChildA;
            var nodeChildB = node->ChildB;

            var ab = BoundingBox.Intersects(ref node->A, ref node->B);

            if (nodeChildA >= 0)
                CollectNodePairsInNode(nodeChildA, node->LeafCountA, collisionTestThreshold, ref nodePairsToTest, ref results);
            if (nodeChildB >= 0)
                CollectNodePairsInNode(nodeChildB, node->LeafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);

            //Test all different nodes.
            if (ab)
            {
                TestForCollectNodePairs(ref node->A, ref node->B, nodeChildA, nodeChildB, node->LeafCountA, node->LeafCountB, collisionTestThreshold, ref nodePairsToTest, ref results);
            }

        }
        public unsafe void CollectNodePairs<TResultList>(int collisionTestThreshold, ref QuickList<Overlap> nodePairsToTest, ref TResultList results) where TResultList : IList<Overlap>
        {
            CollectNodePairsInNode(0, leafCount, collisionTestThreshold, ref nodePairsToTest, ref results);

        }

        public unsafe void GetSelfOverlaps(IParallelLooper looper, MultithreadedSelfTest context)
        {
            //If there are not multiple children, there's no need to recurse.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (nodes->ChildCount < 2)
                return;

            context.Prepare(this);

            int collisionTestThreshold = (int)(leafCount / (1.5f * looper.ThreadCount));
            CollectNodePairs(collisionTestThreshold, ref context.NodePairsToTest, ref context.WorkerOverlaps[0]);
            //CollectNodePairs2(looper.ThreadCount * 16, ref context.NodePairsToTest, ref context.WorkerOverlaps[0]);

            //Console.WriteLine($"number of pairs to test: {context.NodePairsToTest.Count}");
            looper.ForLoop(0, looper.ThreadCount, context.PairTestAction);

            context.CleanUp();
        }
    }
}
