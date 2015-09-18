using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using BEPUutilities.Threading;
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

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        public class SelfTestMultithreadedContext
        {
            public Tree Tree;
            public int ThreadCount
            {
                get; private set;
            }
            public BufferPool<Overlap> Pool
            {
                get; private set;
            }
            public QuickList<Overlap>[] WorkerOverlaps
            {
                get; private set;
            }
            int NextNodePair;
            public QuickList<Overlap> NodePairsToTest;
            public Action<int> PairTestAction
            {
                get; private set;
            }

            bool disposed;

            public SelfTestMultithreadedContext(int threadCount, BufferPool<Overlap> pool)
            {
                Initialize(threadCount, pool);
            }

            public void Initialize(int threadCount, BufferPool<Overlap> pool)
            {
                NodePairsToTest = new QuickList<Overlap>(pool);
                if (WorkerOverlaps == null || threadCount != ThreadCount)
                {
                    ThreadCount = threadCount;
                    WorkerOverlaps = new QuickList<Overlap>[ThreadCount];
                }
                for (int i = 0; i < ThreadCount; ++i)
                {
                    WorkerOverlaps[i] = new QuickList<Overlap>(pool);
                }
                PairTestAction = PairTest;
                disposed = false;
            }

            /// <summary>
            /// Prepares the context for immediate use.
            /// </summary>
            /// <param name="tree">Tree that the context will be operating on.</param>
            internal void Prepare(Tree tree)
            {
                Debug.Assert(!disposed);
                Tree = tree;
                for (int i = 0; i < WorkerOverlaps.Length; ++i)
                {
                    WorkerOverlaps[i].Count = 0;
                }
                NextNodePair = -1;
                NodePairsToTest.Count = 0;
            }

            /// <summary>
            /// Performs any cleaning necessary for the context to return to a pool safely (i.e. without holding resources to objects it doesn't own).
            /// </summary>
            internal void CleanUp()
            {
                Debug.Assert(!disposed);
                Tree = null;
            }

            unsafe void PairTest(int workerIndex)
            {
                Debug.Assert(!disposed);
                int nextNodePairIndex;
                //To minimize the number of worker overlap lists, perform direct load balancing by manually grabbing the next indices.
                while ((nextNodePairIndex = Interlocked.Increment(ref NextNodePair)) < NodePairsToTest.Count)
                {
                    var overlap = NodePairsToTest[nextNodePairIndex];
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

            /// <summary>
            /// Releases all resources held by the context.
            /// </summary>
            public void Dispose()
            {
                if (!disposed)
                {
                    CleanUp();
                    NodePairsToTest.Count = 0;
                    NodePairsToTest.Dispose();
                    for (int i = 0; i < ThreadCount; ++i)
                    {
                        WorkerOverlaps[i].Count = 0;
                        WorkerOverlaps[i].Dispose();
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
                //if (Math.Log(nodeLeafCount) <= collisionTestThreshold)
                if (nodeLeafCount <= collisionTestThreshold)
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

        public void GetSelfOverlaps(IParallelLooper looper, SelfTestMultithreadedContext context)
        {
            context.Prepare(this);

            //int collisionTestThreshold = leafCount / (2 * looper.ThreadCount);
            //CollectNodePairs(collisionTestThreshold, ref context.NodePairsToTest, ref context.WorkerOverlaps[0]);
            CollectNodePairs2(looper.ThreadCount * 8, ref context.NodePairsToTest, ref context.WorkerOverlaps[0]);

            //Console.WriteLine($"number of pairs to test: {context.NodePairsToTest.Count}");
            looper.ForLoop(0, looper.ThreadCount, context.PairTestAction);

            context.CleanUp();
        }
    }
}
