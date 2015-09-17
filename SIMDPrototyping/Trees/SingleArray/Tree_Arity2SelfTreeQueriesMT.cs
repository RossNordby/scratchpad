using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
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
            Action<int> PairTestAction;

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
            private void Prepare(Tree tree)
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
            private void CleanUp()
            {
                Debug.Assert(!disposed);
                Tree = null;
            }

            unsafe void PairTest(int i)
            {
                Debug.Assert(!disposed);
                int nextNodePairIndex;
                //To minimize the number of worker overlap lists, perform direct load balancing by manually grabbing the next indices.
                while ((nextNodePairIndex = Interlocked.Increment(ref NextNodePair)) < NodePairsToTest.Count)
                {
                    var overlap = NodePairsToTest[i];
                    if (overlap.A >= 0)
                    {
                        if (overlap.A == overlap.B)
                        {
                            //Same node.
                            Tree.GetOverlapsInNode2(Tree.nodes + overlap.A, ref WorkerOverlaps[i]);
                        }
                        else if (overlap.B >= 0)
                        {
                            //Different nodes.
                            Tree.GetOverlapsBetweenDifferentNodes2(Tree.nodes + overlap.A, Tree.nodes + overlap.B, ref WorkerOverlaps[i]);
                        }
                        else
                        {
                            //A is an internal node, B is a leaf.
                            var leafIndex = Tree.Encode(overlap.B);
                            var leaf = Tree.leaves + leafIndex;
                            Tree.TestLeafAgainstNode2(leafIndex, ref (&Tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex], overlap.A, ref WorkerOverlaps[i]);
                        }
                    }
                    else
                    {
                        //A is a leaf, B is internal.
                        var leafIndex = Tree.Encode(overlap.A);
                        var leaf = Tree.leaves + leafIndex;
                        Tree.TestLeafAgainstNode2(leafIndex, ref (&Tree.nodes[leaf->NodeIndex].A)[leaf->ChildIndex], overlap.B, ref WorkerOverlaps[i]);

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


    }
}
