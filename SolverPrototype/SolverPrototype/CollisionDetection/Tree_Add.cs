﻿using BEPUutilities2;
using System.Numerics;
using System.Runtime.CompilerServices;
using System;

namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        /// <summary>
        /// Merges a new leaf node with an existing leaf node, producing a new internal node referencing both leaves, and then returns the index of the leaf node.
        /// </summary>
        /// <param name="newLeafBounds">Bounding box of the leaf being added.</param>
        /// <param name="parentIndex">Index of the parent node that the existing leaf belongs to.</param>
        /// <param name="indexInParent">Index of the child wtihin the parent node that the existing leaf belongs to.</param>
        /// <param name="merged">Bounding box holding both the new and existing leaves.</param>
        /// <returns>Index of the leaf </returns>
        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe int MergeLeafNodes(ref BoundingBox newLeafBounds, int parentIndex, int indexInParent, ref BoundingBox merged)
        {
            //It's a leaf node.
            //Create a new internal node with the new leaf and the old leaf as children.
            //this is the only place where a new level could potentially be created.

            var newNodeIndex = AllocateNode();
            var newNode = nodes + newNodeIndex;
            newNode->ChildCount = 2;
            newNode->Parent = parentIndex;
            newNode->IndexInParent = indexInParent;
            //The first child of the new node is the old leaf. Insert its bounding box.
            var parentNode = nodes + parentIndex;
            ref var childInParent = ref (&parentNode->A)[indexInParent];
            newNode->A = childInParent;

            //Insert the new leaf into the second child slot.
            ref var b = ref newNode->B;
            b.Min = newLeafBounds.Min;
            var leafIndex = AddLeaf(newNodeIndex, 1);
            b.Index = Encode(leafIndex);
            b.Max = newLeafBounds.Max;
            b.LeafCount = 1;
            
            //Update the old leaf node with the new index information.
            var oldLeafIndex = Encode(newNode->A.Index);
            leaves[oldLeafIndex] = new Leaf(newNodeIndex, 0);

            //Update the original node's child pointer and bounding box.
            childInParent.Index = newNodeIndex;
            childInParent.Min = merged.Min;
            childInParent.Max = merged.Max;
            ++childInParent.LeafCount;
            return leafIndex;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe int InsertLeafIntoEmptySlot(ref BoundingBox leafBox, int nodeIndex, int childIndex, Node* node)
        {
            ++node->ChildCount;
            var leafIndex = AddLeaf(nodeIndex, childIndex);
            ref var child = ref (&node->A)[childIndex];
            child.Min = leafBox.Min;
            child.Index = Encode(leafIndex);
            child.Max = leafBox.Max;
            child.LeafCount = 1;
            return leafIndex;
        }
        enum BestInsertionChoice
        {
            NewInternal,
            Traverse
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void CreateMerged(ref Vector3 minA, ref Vector3 maxA, ref Vector3 minB, ref Vector3 maxB, out BoundingBox merged)
        {
            merged.Min = Vector3.Min(minA, minB);
            merged.Max = Vector3.Max(maxA, maxB);
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        private static unsafe BestInsertionChoice ComputeBestInsertionChoice(ref BoundingBox box, ref NodeChild child, out BoundingBox mergedCandidate, out float costChange)
        {
            CreateMerged(ref child.Min, ref child.Max, ref box.Min, ref box.Max, out mergedCandidate);
            var newCost = ComputeBoundsMetric(ref mergedCandidate);
            if (child.Index >= 0)
            {
                costChange = newCost - ComputeBoundsMetric(ref child.Min, ref child.Max);
                return BestInsertionChoice.Traverse;
            }
            else
            {
                costChange = newCost;
                return BestInsertionChoice.NewInternal;
            }

        }

        /// <summary>
        /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
        /// </summary>
        /// <param name="box">Bounding box of the leaf to add.</param>
        /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public unsafe int Add(ref BoundingBox box)
        {
            //The rest of the function assumes we have sufficient room. We don't want to deal with invalidated pointers mid-add.
            if(Leaves.Length == leafCount)
            {
                //Note that, while we add 1, the underlying pool will request the next higher power of 2 in bytes that can hold it.
                //Since we're already at capacity, that will be ~double the size.
                Resize(leafCount + 1); 
            }

            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            int nodeIndex = 0;
            while (true)
            {
                //Which child should the leaf belong to?

                //Give the leaf to whichever node had the least cost change. 
                var node = nodes + nodeIndex;
                //This is a binary tree, so the only time a node can have less than full children is when it's the root node.
                //By convention, an empty tree still has a root node with no children, so we do have to handle this case.
                if (leafCount < 2)
                {
                    //The best slot will, at best, be tied with inserting it in a leaf node because the change in heuristic cost for filling an empty slot is zero.
                    return InsertLeafIntoEmptySlot(ref box, nodeIndex, leafCount, node);
                }
                else
                {
                    ref var a = ref node->A;
                    ref var b = ref node->B;
                    var choiceA = ComputeBestInsertionChoice(ref box, ref a, out var mergedA, out var costChangeA);
                    var choiceB = ComputeBestInsertionChoice(ref box, ref b, out var mergedB, out var costChangeB);
                    Console.WriteLine($"costchangeA: {costChangeA}, b: {costChangeB}");
                    if(costChangeA <= costChangeB)
                    {
                        if(choiceA == BestInsertionChoice.NewInternal)
                        {
                            return MergeLeafNodes(ref box, nodeIndex, 0, ref mergedA);
                        }
                        else //if (choiceA == BestInsertionChoice.Traverse)
                        {
                            a.Min = mergedA.Min;
                            a.Max = mergedA.Max;
                            nodeIndex = a.Index;
                            ++a.LeafCount;
                        }
                    }
                    else
                    {
                        if (choiceB == BestInsertionChoice.NewInternal)
                        {
                            return MergeLeafNodes(ref box, nodeIndex, 1, ref mergedB);
                        }
                        else //if (choiceB == BestInsertionChoice.Traverse)
                        {
                            b.Min = mergedB.Min;
                            b.Max = mergedB.Max;
                            nodeIndex = b.Index;
                            ++b.LeafCount;
                        }
                    }
                }


            }
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        internal static float ComputeBoundsMetric(ref BoundingBox bounds)
        {
            return ComputeBoundsMetric(ref bounds.Min, ref bounds.Max);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        internal static float ComputeBoundsMetric(ref Vector3 min, ref Vector3 max)
        {
            //Note that we just use the SAH. While we are primarily interested in volume queries for the purposes of collision detection, the topological difference
            //between a volume heuristic and surface area heuristic isn't huge. There is, however, one big annoying issue that volume heuristics run into:
            //all bounding boxes with one extent equal to zero have zero cost. Surface area approaches avoid this hole simply.
            var offset = max - min;
            //Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
            return offset.X * offset.Y + offset.Y * offset.Z + offset.X * offset.Z;
        }
    }
}
