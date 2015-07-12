using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Vectorized
{
    partial class Tree<T>
    {
        struct StreamingLeafGroup
        {
            public BoundingBoxWide BoundingBoxes;
            public Vector<int> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add(ref StreamingLeafGroup parent, int sourceIndex, int destinationIndex, Vector<int>[] masks)
            {
                //This is basically: a[i] = b[j]
                var leavesBroadcast = new Vector<int>(parent.Leaves[sourceIndex]);
                BoundingBoxWide boundsBroadcast;
                BoundingBoxWide.GetBoundingBox(ref parent.BoundingBoxes, sourceIndex, out boundsBroadcast);
                Leaves = Vector.ConditionalSelect(masks[destinationIndex], leavesBroadcast, Leaves);
                BoundingBoxWide.ConditionalSelect(ref masks[destinationIndex], ref boundsBroadcast, ref BoundingBoxes, out BoundingBoxes);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add(ref BoundingBoxWide boundingBox, ref Vector<int> leafIndex, ref Vector<int> destinationMask)
            {
                Leaves = Vector.ConditionalSelect(destinationMask, leafIndex, Leaves);
                BoundingBoxWide.ConditionalSelect(ref destinationMask, ref boundingBox, ref BoundingBoxes, out BoundingBoxes);
            }
        }
        struct StreamingTarget
        {
            public int LevelIndex;
            public int NodeIndex;
            public int LastLeavesCount;
            public QuickList<StreamingLeafGroup> LeafGroups;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add(ref StreamingTarget parent, int childGroupIndex, int childIndexInGroup, Vector<int>[] masks)
            {
                if (LastLeavesCount < Vector<float>.Count)
                {
                    LeafGroups.Elements[LeafGroups.Count - 1].Add(ref parent.LeafGroups.Elements[childGroupIndex], childIndexInGroup, LastLeavesCount, masks);
                    ++LastLeavesCount;
                }
                else
                {
                    var newLeaves = new StreamingLeafGroup();
                    newLeaves.Add(ref parent.LeafGroups.Elements[childGroupIndex], childIndexInGroup, 0, masks);
                    LeafGroups.Add(ref newLeaves);
                    LastLeavesCount = 1;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add(ref Vector<int> leafIndex, ref BoundingBoxWide boundingBox, Vector<int>[] masks)
            {
                if (LastLeavesCount < Vector<float>.Count)
                {
                    LeafGroups.Elements[LeafGroups.Count - 1].Add(ref boundingBox, ref leafIndex, ref masks[LastLeavesCount]);
                    ++LastLeavesCount;
                }
                else
                {
                    var newLeaves = new StreamingLeafGroup();
                    newLeaves.Add(ref boundingBox, ref leafIndex, ref masks[0]);
                    LeafGroups.Add(ref newLeaves);
                    LastLeavesCount = 1;
                }
            }

        }



        public unsafe void GetSelfOverlapsViaStreamingQueries<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            //var startTime = Stopwatch.GetTimestamp();
            var rootTarget = new StreamingTarget { LeafGroups = new QuickList<StreamingLeafGroup>(BufferPools<StreamingLeafGroup>.Locking, BufferPool<StreamingLeafGroup>.GetPoolIndex(LeafCount)) };
            rootTarget.LeafGroups.Add(new StreamingLeafGroup());
            for (int i = 0; i < LeafCount; ++i)
            {
                BoundingBoxWide leafWide;
                BoundingBoxWide.GetBoundingBox(ref Levels[leaves[i].LevelIndex].Nodes[leaves[i].NodeIndex].BoundingBoxes, leaves[i].ChildIndex, out leafWide);
                var leafIndexWide = new Vector<int>(i);
                rootTarget.Add(ref leafIndexWide, ref leafWide, singleMasks);
            }
            //var endTime = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Initial target construction time: {(endTime - startTime) / (double)Stopwatch.Frequency}");

            QuickQueue<StreamingTarget> targets = new QuickQueue<StreamingTarget>(BufferPools<StreamingTarget>.Locking, BufferPool<StreamingLeafGroup>.GetPoolIndex(LeafCount));
            targets.Enqueue(ref rootTarget);

            QuickList<int> fallbackResults = new QuickList<int>(BufferPools<int>.Locking);

            StreamingTarget target;
            while (targets.TryDequeueLast(out target))
            {
                const int GroupFallbackThreshold = 2; //unfortunately, this should be as high as possible right now because the regular query is faster, period.
                if (target.LeafGroups.Count <= GroupFallbackThreshold)
                {
                    var max = target.LastLeavesCount == Vector<int>.Count ? target.LeafGroups.Count : target.LeafGroups.Count - 1;
                    for (int leafGroupIndex = 0; leafGroupIndex < max; ++leafGroupIndex)
                    {
                        for (int leafInGroupIndex = 0; leafInGroupIndex < Vector<int>.Count; ++leafInGroupIndex)
                        {
                            BoundingBoxWide leafWide;
                            BoundingBoxWide.GetBoundingBox(ref target.LeafGroups.Elements[leafGroupIndex].BoundingBoxes, leafInGroupIndex, out leafWide);
                            TestRecursive(target.LevelIndex, target.NodeIndex, ref leafWide, ref fallbackResults);
                            for (int resultIndex = 0; resultIndex < fallbackResults.Count; ++resultIndex)
                            {
                                var queryLeafIndex = target.LeafGroups.Elements[leafGroupIndex].Leaves[leafInGroupIndex];
                                if (queryLeafIndex < fallbackResults.Elements[resultIndex])
                                {
                                    results.Add(new Overlap { A = queryLeafIndex, B = fallbackResults.Elements[resultIndex] });
                                }
                            }
                            fallbackResults.Count = 0;
                        }
                    }
                    if (target.LastLeavesCount < Vector<int>.Count)
                    {
                        var leafGroupIndex = target.LeafGroups.Count - 1;
                        for (int leafInGroupIndex = 0; leafInGroupIndex < target.LastLeavesCount; ++leafInGroupIndex)
                        {
                            BoundingBoxWide leafWide;
                            BoundingBoxWide.GetBoundingBox(ref target.LeafGroups.Elements[leafGroupIndex].BoundingBoxes, leafInGroupIndex, out leafWide);
                            TestRecursive(target.LevelIndex, target.NodeIndex, ref leafWide, ref fallbackResults);
                            for (int resultIndex = 0; resultIndex < fallbackResults.Count; ++resultIndex)
                            {
                                var queryLeafIndex = target.LeafGroups.Elements[leafGroupIndex].Leaves[leafInGroupIndex];
                                if (queryLeafIndex < fallbackResults.Elements[resultIndex])
                                {
                                    results.Add(new Overlap { A = queryLeafIndex, B = fallbackResults.Elements[resultIndex] });
                                }
                            }
                            fallbackResults.Count = 0;
                        }
                    }
                }
                else
                {
                    var node = Levels[target.LevelIndex].Nodes[target.NodeIndex];



                    //Test each node child against all of the leaves for this node.
                    for (int nodeChildIndex = 0; nodeChildIndex < Vector<int>.Count; ++nodeChildIndex)
                    {
                        if (node.Children[nodeChildIndex] == -1)
                            continue;

                        BoundingBoxWide nodeChildWide;
                        BoundingBoxWide.GetBoundingBox(ref node.BoundingBoxes, nodeChildIndex, out nodeChildWide);

                        if (node.Children[nodeChildIndex] >= 0)
                        {
                            //Internal node. Can spawn more targets.
                            StreamingTarget newTarget = new StreamingTarget
                            {
                                LevelIndex = target.LevelIndex + 1,
                                NodeIndex = node.Children[nodeChildIndex],
                                LeafGroups = new QuickList<StreamingLeafGroup>(BufferPools<StreamingLeafGroup>.Locking, BufferPool<StreamingLeafGroup>.GetPoolIndex(target.LeafGroups.Count))
                            };
                            newTarget.LeafGroups.Add(new StreamingLeafGroup());


                            for (int leafGroupIndex = 0; leafGroupIndex < target.LeafGroups.Count; ++leafGroupIndex)
                            {

                                Vector<int> intersectionMask;
                                BoundingBoxWide.Intersects(ref nodeChildWide, ref target.LeafGroups.Elements[leafGroupIndex].BoundingBoxes, out intersectionMask);

                                int leafCountInGroup = leafGroupIndex == target.LeafGroups.Count - 1 ? target.LastLeavesCount : Vector<int>.Count;

                                for (int leafIndexInGroup = 0; leafIndexInGroup < leafCountInGroup; ++leafIndexInGroup)
                                {
                                    if (intersectionMask[leafIndexInGroup] < 0)
                                    {
                                        newTarget.Add(ref target, leafGroupIndex, leafIndexInGroup, singleMasks);
                                    }
                                }

                            }
                            targets.Enqueue(ref newTarget);
                        }
                        else
                        {
                            //Leaf node.

                            var nodeLeafIndex = Encode(node.Children[nodeChildIndex]);

                            for (int leafGroupIndex = 0; leafGroupIndex < target.LeafGroups.Count; ++leafGroupIndex)
                            {

                                Vector<int> intersectionMask;
                                BoundingBoxWide.Intersects(ref nodeChildWide, ref target.LeafGroups.Elements[leafGroupIndex].BoundingBoxes, out intersectionMask);

                                int leafCountInGroup = leafGroupIndex == target.LeafGroups.Count - 1 ? target.LastLeavesCount : Vector<int>.Count;

                                for (int leafIndexInGroup = 0; leafIndexInGroup < leafCountInGroup; ++leafIndexInGroup)
                                {
                                    if (intersectionMask[leafIndexInGroup] < 0)
                                    {
                                        var leafIndex = target.LeafGroups[leafGroupIndex].Leaves[leafIndexInGroup];
                                        if (leafIndex < nodeLeafIndex) //The other leaf will also find a collision!
                                        {
                                            results.Add(new Overlap { A = leafIndex, B = nodeLeafIndex });
                                        }
                                    }
                                }

                            }


                        }
                    }
                }
                target.LeafGroups.Count = 0; //Don't bother forcing a clear on these. TODO: buffer safety check disable
                target.LeafGroups.Dispose();
            }
            targets.Dispose();
            fallbackResults.Dispose();

            //Console.WriteLine("Streaming Query based results:");
            //for (int i = 0; i < results.Count; ++i)
            //{
            //    Console.WriteLine($"{results[i].A}, {results[i].B}");
            //}

        }
    }
}
