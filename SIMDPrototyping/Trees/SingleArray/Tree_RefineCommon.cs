using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    public struct SubtreeHeapEntry
    {
        public int Index;
        public float Cost;
    }
    unsafe internal struct SubtreeBinaryHeap
    {
        public SubtreeHeapEntry* Entries;
        public int Count;

        public SubtreeBinaryHeap(SubtreeHeapEntry* entries)
        {
            Entries = entries;
            Count = 0;
        }


        public unsafe void Insert(Node* node, Node* nodes, ref QuickList<int> subtrees)
        {
            var children = &node->ChildA;
            var bounds = &node->A;
            for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
            {
                if (children[childIndex] >= 0)
                {
                    int index = Count;
                    var cost = Tree.ComputeBoundsMetric(ref bounds[childIndex]);// - node->PreviousMetric;
                    ++Count;

                    //Sift up.
                    while (index > 0)
                    {
                        var parentIndex = (index - 1) >> 1;
                        var parent = Entries + parentIndex;
                        if (parent->Cost < cost)
                        {
                            //Pull the parent down.
                            Entries[index] = *parent;
                            index = parentIndex;
                        }
                        else
                        {
                            //Found the insertion spot.
                            break;
                        }
                    }
                    var entry = Entries + index;
                    entry->Index = children[childIndex];
                    entry->Cost = cost;

                }
                else
                {
                    //Immediately add leaf nodes.
                    subtrees.Add(children[childIndex]);
                }
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Pop(out SubtreeHeapEntry entry)
        {
            entry = Entries[0];
            --Count;
            var cost = Entries[Count].Cost;

            //Pull the elements up to fill in the gap.
            int index = 0;
            while (true)
            {
                var childIndexA = (index << 1) + 1;
                var childIndexB = (index << 1) + 2;
                if (childIndexB < Count)
                {
                    //Both children are available.
                    //Try swapping with the largest one.
                    var childA = Entries + childIndexA;
                    var childB = Entries + childIndexB;
                    if (childA->Cost > childB->Cost)
                    {
                        if (cost > childA->Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexA];
                        index = childIndexA;
                    }
                    else
                    {
                        if (cost > childB->Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexB];
                        index = childIndexB;
                    }
                }
                else if (childIndexA < Count)
                {
                    //Only one child was available.
                    var childA = Entries + childIndexA;
                    if (cost > childA->Cost)
                    {
                        break;
                    }
                    Entries[index] = Entries[childIndexA];
                    index = childIndexA;
                }
                else
                {
                    //The children were beyond the heap.
                    break;
                }
            }
            //Move the last entry into position.
            Entries[index] = Entries[Count];

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPop(Node* nodes, ref int remainingSubtreeSpace, ref QuickList<int> subtrees, out int index, out float cost)
        {
            while (Count > 0)
            {
                //Repeatedly pop minimum until you find one that can fit.
                //Given the unique access nature, the fact that you're destroying the heap when there's not much space left doesn't matter.
                //In the event that you consume all the nodes, that just means there aren't any entries which would fit in the subtree set anymore.
                SubtreeHeapEntry entry;
                Pop(out entry);
                var node = nodes + entry.Index;
                var changeInChildCount = remainingSubtreeSpace - node->ChildCount == 0 ? node->ChildCount : node->ChildCount - 1;
                if (remainingSubtreeSpace >= changeInChildCount)// && node->RefineFlag == 0)
                {
                    //This node's children can be included successfully in the remaining space.
                    index = entry.Index;
                    cost = entry.Cost;
                    remainingSubtreeSpace -= changeInChildCount;
                    return true;
                }
                else
                {
                    //Either this node's children did not fit, or it was a refinement target. Refinement targets cannot be expanded.
                    //Since we won't be able to find this later, it needs to be added now.
                    //We popped the previous entry off the queue, so the remainingSubtreeSpace does not change by re-adding it.
                    //(remainingSubtreeSpace = maximumSubtreesCount - (priorityQueue.Count + subtrees.Count))
                    subtrees.Add(entry.Index);
                }
            }
            index = -1;
            cost = -1;
            return false;
        }
    }

    partial class Tree
    {

        public unsafe void CollectSubtrees(int nodeIndex, int maximumSubtrees, SubtreeHeapEntry* entries, ref QuickList<int> subtrees, ref QuickQueue<int> internalNodes, out float treeletCost)
        {

            //Collect subtrees iteratively by choosing the highest surface area subtree repeatedly.
            //This collects every child of a given node at once- the set of subtrees must not include only SOME of the children of a node.

            //(You could lift this restriction and only take some nodes, but it would complicate things. You could not simply remove
            //the parent and add its children to go deeper; it would require doing some post-fixup on the results of the construction
            //or perhaps constraining the generation process to leave room for the unaffected nodes.)


            var node = nodes + nodeIndex;
            Debug.Assert(maximumSubtrees >= node->ChildCount, "Can't only consider some of a node's children, but specified maximumSubtrees precludes the treelet root's children.");
            //All of treelet root's children are included immediately. (Follows from above requirement.)

            var priorityQueue = new SubtreeBinaryHeap(entries);

            priorityQueue.Insert(node, nodes, ref subtrees);

            //The root is inserted first; the dequeue process will find the root first, guaranteeing that the root does not move.
            internalNodes.Enqueue(nodeIndex);

            //Note that the treelet root's cost is excluded from the treeletCost.
            //That's because the treelet root cannot change.
            treeletCost = 0;
            int highestIndex;
            float highestCost;
            int remainingSubtreeSpace = maximumSubtrees - priorityQueue.Count;
            while (priorityQueue.TryPop(nodes, ref remainingSubtreeSpace, ref subtrees, out highestIndex, out highestCost))
            {
                treeletCost += highestCost;
                internalNodes.Enqueue(highestIndex);

                //Add all the children to the set of subtrees.
                //This is safe because we pre-validated the number of children in the node.
                var expandedNode = nodes + highestIndex;
                priorityQueue.Insert(expandedNode, nodes, ref subtrees);
            }

            for (int i = 0; i < priorityQueue.Count; ++i)
            {
                subtrees.Add(priorityQueue.Entries[i].Index);
            }

        }


        unsafe internal struct Subtrees
        {
            public BoundingBox* BoundingBoxes;
            public int* LeafCounts;
            public int* IndexMap;
            public float* CentroidsX;
            public float* CentroidsY;
            public float* CentroidsZ;
        }

        unsafe void ValidateStaging(Node* stagingNodes, ref QuickList<int> subtreeNodePointers, int treeletParent, int treeletIndexInParent)
        {
            int foundSubtrees, foundLeafCount;
            QuickList<int> collectedSubtreeReferences = new QuickList<int>(BufferPools<int>.Thread);
            QuickList<int> internalReferences = new QuickList<int>(BufferPools<int>.Thread);
            internalReferences.Add(0);
            ValidateStaging(stagingNodes, 0, ref subtreeNodePointers, ref collectedSubtreeReferences, ref internalReferences, out foundSubtrees, out foundLeafCount);
            if (treeletParent < -1 || treeletParent >= nodeCount)
                throw new Exception("Bad treelet parent.");
            if (treeletIndexInParent < -1 || (treeletParent >= 0 && treeletIndexInParent >= nodes[treeletParent].ChildCount))
                throw new Exception("Bad treelet index in parent.");
            if (treeletParent >= 0 && (&nodes[treeletParent].LeafCountA)[treeletIndexInParent] != foundLeafCount)
            {
                throw new Exception("Bad leaf count.");
            }
            if (subtreeNodePointers.Count != foundSubtrees)
            {
                throw new Exception("Bad subtree found count.");
            }
            for (int i = 0; i < collectedSubtreeReferences.Count; ++i)
            {
                if (!subtreeNodePointers.Contains(collectedSubtreeReferences[i]) || !collectedSubtreeReferences.Contains(subtreeNodePointers[i]))
                    throw new Exception("Bad subtree reference.");
            }
            collectedSubtreeReferences.Dispose();
            internalReferences.Dispose();
        }
        unsafe void ValidateStaging(Node* stagingNodes, int stagingNodeIndex, ref QuickList<int> subtreeNodePointers, ref QuickList<int> collectedSubtreeReferences, ref QuickList<int> internalReferences, out int foundSubtrees, out int foundLeafCount)
        {
            var stagingNode = stagingNodes + stagingNodeIndex;
            var children = &stagingNode->ChildA;
            var leafCounts = &stagingNode->LeafCountA;
            foundSubtrees = foundLeafCount = 0;
            for (int i = 0; i < stagingNode->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    int childFoundSubtrees, childFoundLeafCount;
                    if (internalReferences.Contains(children[i]))
                        throw new Exception("A child points to an internal node that was visited. Possible loop, or just general invalid.");
                    internalReferences.Add(children[i]);
                    ValidateStaging(stagingNodes, children[i], ref subtreeNodePointers, ref collectedSubtreeReferences, ref internalReferences, out childFoundSubtrees, out childFoundLeafCount);

                    if (childFoundLeafCount != leafCounts[i])
                        throw new Exception("Bad leaf count.");
                    foundSubtrees += childFoundSubtrees;
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    var subtreeNodePointerIndex = Encode(children[i]);
                    var subtreeNodePointer = subtreeNodePointers.Elements[subtreeNodePointerIndex];
                    //Rather than looking up the shuffled SweepSubtree for information, just go back to the source.
                    if (subtreeNodePointer >= 0)
                    {
                        var node = nodes + subtreeNodePointer;
                        var totalLeafCount = 0;
                        for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
                        {
                            totalLeafCount += (&node->LeafCountA)[childIndex];
                        }

                        if (leafCounts[i] != totalLeafCount)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += totalLeafCount;
                    }
                    else
                    {
                        var leafIndex = Encode(subtreeNodePointer);
                        if (leafCounts[i] != 1)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += 1;
                    }
                    ++foundSubtrees;
                    collectedSubtreeReferences.Add(subtreeNodePointer);
                }
            }

        }

    }
}
