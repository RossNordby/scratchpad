////#define NODE8


//using BEPUutilities.DataStructures;
//using BEPUutilities.ResourceManagement;
//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Linq;
//using System.Numerics;
//using System.Runtime.CompilerServices;
//using System.Text;
//using System.Threading.Tasks;

//#if NODE32
//using Node = SIMDPrototyping.Trees.Baseline.Node32;
//#elif NODE16
//using Node = SIMDPrototyping.Trees.Baseline.Node16;
//#elif NODE8
//using Node = SIMDPrototyping.Trees.Baseline.Node8;
//#elif NODE4
//using Node = SIMDPrototyping.Trees.Baseline.Node4;
//#elif NODE2
//using Node = SIMDPrototyping.Trees.Baseline.Node2;
//#endif

//namespace SIMDPrototyping.Trees.SingleArray
//{
//    partial class Tree<T>
//    {
//        struct Path
//        {
//            public QuickList<int> ChildrenIndices;
//            public float CostIncrease;
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        void SwapPaths(ref Path candidate, ref Path best, float newCostIncrease, int childIndex)
//        {
//            //A new best! Swap.
//            //Note that we copy the new best's not-yet-modified state into the candidate.
//            //That way, the candidate contains the path used to reach this node, but not the final node.
//            var oldBest = best;
//            best = candidate;
//            candidate = oldBest;
//            candidate.ChildrenIndices.EnsureCapacity(best.ChildrenIndices.Count);
//            candidate.ChildrenIndices.Count = best.ChildrenIndices.Count;
//            Array.Copy(best.ChildrenIndices.Elements, candidate.ChildrenIndices.Elements, best.ChildrenIndices.Count);

//            best.ChildrenIndices.Add(childIndex);
//            best.CostIncrease = newCostIncrease;
//        }

//        unsafe void ComputeBestCostChange(int levelIndex, int nodeIndex, ref BoundingBox newLeafBounds, ref Path candidate, ref Path best)
//        {
//            var node = Levels[levelIndex].Nodes + nodeIndex;
//            var bounds = &node->A;
//            var children = &node->ChildA;
//            int nextLevel = levelIndex + 1;
//            if (node->ChildCount < ChildrenCapacity)
//            {
//                //May be able to insert in empty slot.
//                //Note that this does not increase the cost at all! This does not create a new internal node!
//                //Leaf costs are constant.

//                if (candidate.CostIncrease < best.CostIncrease)
//                {
//                    SwapPaths(ref candidate, ref best, candidate.CostIncrease, node->ChildCount);
//                }
//            }
//            for (int i = 0; i < node->ChildCount; ++i)
//            {
//                if (children[i] >= 0)
//                {
//                    //About to recurse into an internal node. Update the candidate.
//                    candidate.ChildrenIndices.Add(i);
//                    var previousCostIncrease = candidate.CostIncrease;
//                    var oldCost = ComputeBoundsHeuristic(ref bounds[i]);
//                    BoundingBox merged;
//                    BoundingBox.Merge(ref bounds[i], ref newLeafBounds, out merged);
//                    var newCost = ComputeBoundsHeuristic(ref merged);
//                    candidate.CostIncrease += newCost - oldCost;

//                    ComputeBestCostChange(nextLevel, children[i], ref newLeafBounds, ref candidate, ref best);

//                    //Undo this node's modification on the candidate in preparation for the next candidate in this node.
//                    //If any good option was found, it now resides in the 'best' path.
//                    //Note how the candidate is treated like a stack... because it is one.
//                    candidate.ChildrenIndices.FastRemoveAt(candidate.ChildrenIndices.Count - 1);
//                    candidate.CostIncrease = previousCostIncrease;
//                }
//                else
//                {
//                    //It's a leaf node. Merging with this leaf node to produce a new internal node could be a new optimal.
//                    //Note that we do not take into account the previous bounds cost. That's because that bound was a *leaf node*.
//                    //What is being measured is the increase in the INTERNAL NODE cost.
//                    //Given a constant set of leaves, the leaves' contribution to the total tree cost is constant, so they're ignored.
//                    //The merging process creates a whole new internal node, so it is included fully.

//                    BoundingBox merged;
//                    BoundingBox.Merge(ref bounds[i], ref newLeafBounds, out merged);
//                    var newCost = ComputeBoundsHeuristic(ref merged);

//                    var newPathCostIncrease = candidate.CostIncrease + newCost;
//                    if (newPathCostIncrease < best.CostIncrease)
//                    {
//                        SwapPaths(ref candidate, ref best, newPathCostIncrease, i);
//                    }


//                }

//            }

//            //Note: Since these nodes have a variable number of children, it's possible that the node cost varies.
//            //However, we should expect the cost difference between, say, a node with 2 children and 3 children
//            //is no where near as large as the difference between *no* internal node and *any* internal node.
//            //Going to a new internal node is a dependent cache miss. Testing children is cache-sequential and ALU cheap.
//            //So if you do want to make it more accurate, some empirical testing to see how well various tuning factors
//            //actually adhere to reality would be necessary.
//        }

//        /// <summary>
//        /// Tests every possible insertion path to find the one which minimizes the tree's heuristic cost.
//        /// </summary>
//        /// <param name="leaf">Leaf to insert.</param>
//        public unsafe void InsertGlobal(T leaf)
//        {
//            BoundingBox leafBox;
//            leaf.GetBoundingBox(out leafBox);

//            var poolIndex = BufferPool<int>.GetPoolIndex(maximumDepth);
//            var candidate = new Path { ChildrenIndices = new QuickList<int>(BufferPools<int>.Thread, poolIndex) };
//            var best = new Path { ChildrenIndices = new QuickList<int>(BufferPools<int>.Thread, poolIndex), CostIncrease = float.MaxValue };
//            ComputeBestCostChange(0, 0, ref leafBox, ref candidate, ref best);

//            int nodeIndex = 0;
//            for (int levelIndex = 0; levelIndex < best.ChildrenIndices.Count; ++levelIndex)
//            {
//                var parentNode = Levels[levelIndex].Nodes + nodeIndex;
//                var bounds = &parentNode->A;
//                var children = &parentNode->ChildA;
//                var leafCounts = &parentNode->LeafCountA;
//                //Merge the chosen child with the leaf.
//                var bestChildIndex = best.ChildrenIndices.Elements[levelIndex];
//                BoundingBox merged;
//                BoundingBox.Merge(ref leafBox, ref bounds[bestChildIndex], out merged);
//                if (children[bestChildIndex] == -1)
//                {
//                    Debug.Assert(levelIndex == best.ChildrenIndices.Count - 1);
//                    //Inserting the new leaf into an empty slot.
//                    InsertLeafIntoEmptySlot(leaf, ref leafBox, levelIndex, nodeIndex, bestChildIndex, parentNode);
//                }
//                else if (children[bestChildIndex] < -1)
//                {
//                    Debug.Assert(levelIndex == best.ChildrenIndices.Count - 1);
//                    //Merging the new leaf into an existing leaf node.
//                    MergeLeafNodes(leaf, ref leafBox, levelIndex, nodeIndex, bestChildIndex, ref children[bestChildIndex], ref bounds[bestChildIndex], ref leafCounts[bestChildIndex], ref merged);
//                }
//                else
//                {
//                    //Internal node.
//                    bounds[bestChildIndex] = merged;
//                    nodeIndex = children[bestChildIndex];
//                    ++leafCounts[bestChildIndex];
//                }


//            }

//            candidate.ChildrenIndices.Dispose();
//            best.ChildrenIndices.Dispose();
//        }
//    }
//}
