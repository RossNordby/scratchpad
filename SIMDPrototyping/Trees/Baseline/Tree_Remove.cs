using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;


#if NODE32
using Node = SIMDPrototyping.Trees.Baseline.Node32;
#elif NODE16
using Node = SIMDPrototyping.Trees.Baseline.Node16;
#elif NODE8
using Node = SIMDPrototyping.Trees.Baseline.Node8;
#elif NODE4
using Node = SIMDPrototyping.Trees.Baseline.Node4;
#elif NODE2
using Node = SIMDPrototyping.Trees.Baseline.Node2;
#endif

namespace SIMDPrototyping.Trees.Baseline
{
    public struct LeafMove
    {
        public int OriginalIndex;
        public int NewIndex;
    }
    partial class Tree<T>
    {
        unsafe void PullUpChildren(int sourceLevelIndex, int sourceNodeIndex, int* childrenIndices, int childrenCount, int newParent)
        {
            bool oldNodeReplaced = false;
            //The source node is now dead. Did the moved node have any internal node children?
            int childrenLevel = sourceLevelIndex + 1;
            int firstInternalChildIndex = childrenCount;
            for (int i = 0; i < childrenCount; ++i)
            {
                if (childrenIndices[i] >= 0)
                {
                    //This child is internal. It should be pulled up into the source node itself.
                    //This reuses the node and avoids a pointless remove-add sequence.
                    ReplaceNodeUpOneLevel(childrenLevel, childrenIndices[i], sourceNodeIndex);

                    //Update the child pointer to point at its new home.
                    childrenIndices[i] = sourceNodeIndex;

                    firstInternalChildIndex = i;
                    oldNodeReplaced = true;
                    break;
                }
            }

            if (!oldNodeReplaced)
            {
                //The old node wasn't replaced, meaning there were no internal node children.
                //Just get rid of that old node then.
                Levels[sourceLevelIndex].RemoveAt(sourceNodeIndex);
            }
            else
            {
                //Walk through the remaining children to find any other internal nodes.
                for (int i = firstInternalChildIndex; i < childrenCount; ++i)
                {
                    if (childrenIndices[i] >= 0)
                    {
                        //There are no spare nodes available, so we'll just have to create another one as we move it up.
                        MoveNodeUpOneLevel(childrenLevel, childrenIndices[i], newParent, i);
                    }
                }
            }
        }
        unsafe void MoveNodeUpOneLevel(int sourceLevelIndex, int sourceNodeIndex, int newParentIndex, int indexInParent)
        {
            var sourceNode = Levels[sourceLevelIndex].Nodes + sourceNodeIndex;

            var newNode = *sourceNode;
            //The index in the parent should not change, since we're not changing parents. Just following the parent upwards.
            newNode.Parent = newParentIndex;
            Debug.Assert(indexInParent == newNode.IndexInParent);
            var newNodeIndex = Levels[sourceLevelIndex - 1].Add(ref newNode);

            //The parent's pointer to us, however, will change.
            //The parent is on level sourceLevelIndex - 2; it's moved up. We're following.
            (&Levels[sourceLevelIndex - 2].Nodes[newParentIndex].ChildA)[indexInParent] = newNodeIndex;

            PullUpChildren(sourceLevelIndex, sourceNodeIndex, &Levels[sourceLevelIndex - 1].Nodes[newNodeIndex].ChildA, newNode.ChildCount, newNodeIndex); 

            
        }

        /// <summary>
        /// Moves the bounds, leaf counts, and child count from the source node to the destination node.
        /// The destination parent pointers are left unmodified.
        /// All children are pulled up as necessary to retain the level-by-level access pattern.
        /// Child pointers to internal nodes are updated to match the new locations of the internal nodes.
        /// The source node is removed.
        /// </summary>
        unsafe void ReplaceNodeUpOneLevel(
            int sourceLevelIndex, int sourceNodeIndex, int destinationNodeIndex)
        {
            var sourceNode = Levels[sourceLevelIndex].Nodes + sourceNodeIndex;
            var sourceBounds = &sourceNode->A;
            var sourceChildren = &sourceNode->ChildA;
            var sourceLeafCounts = &sourceNode->LeafCountA;

            int destinationLevelIndex = sourceLevelIndex - 1;
            var destinationNode = Levels[destinationLevelIndex].Nodes + destinationNodeIndex;
            var destinationBounds = &destinationNode->A;
            var destinationChildren = &destinationNode->ChildA;
            var destinationLeafCounts = &destinationNode->LeafCountA;

            //Overwrite the destination node, except for the parent pointers.
            var parent = destinationNode->Parent;
            var indexInParent = destinationNode->IndexInParent;
            *destinationNode = *sourceNode;
            destinationNode->Parent = parent;
            destinationNode->IndexInParent = indexInParent;

            PullUpChildren(sourceLevelIndex, sourceNodeIndex, &destinationNode->ChildA, destinationNode->ChildCount, destinationNodeIndex);

        }

        unsafe void RefitForRemoval(Node* node, int levelIndex)
        {
            while (node->Parent >= 0)
            {
                //Compute the new bounding box for this node.
                var nodeBounds = &node->A;
                var nodeChildren = &node->ChildA;
                var merged = nodeBounds[0];
                for (int i = 1; i < node->ChildCount; ++i)
                {
                    BoundingBox.Merge(ref merged, ref nodeBounds[i], out merged);
                }
                //Push the changes to the parent.
                var parent = Levels[levelIndex - 1].Nodes + node->Parent;
                (&parent->A)[node->IndexInParent] = merged;
                --(&parent->LeafCountA)[node->IndexInParent];

                node = parent;
                levelIndex -= 1;
            }
        }

        public unsafe LeafMove RemoveAt(int leafIndex)
        {
            if (leafIndex < 0 || leafIndex >= leafCount)
                throw new ArgumentOutOfRangeException("Leaf index must be a valid index in the tree's leaf array.");
            var leaf = leaves[leafIndex];
            var node = Levels[leaf.LevelIndex].Nodes + leaf.NodeIndex;
            var nodeChildren = &node->ChildA;
            var nodeBounds = &node->A;
            var nodeLeafCounts = &node->LeafCountA;

            var previousLevel = leaf.LevelIndex - 1;
            //Remove the leaf from this node.
            //Note that the children must remain contiguous. Requires taking the last child of the node and moving into the slot
            //if the removed child was not the last child.
            //Further, if a child is moved and if it is a leaf, that leaf's ChildIndex must be updated.
            //If a child is moved and it is an internal node, all immediate children of that node must have their parent nodes updated.

            //Check to see if this node should collapse.
            if (node->ChildCount == 2 &&
                node->Parent >= 0) //The root cannot 'collapse'. The root is the only node that can end up with 1 child.
            {
                Debug.Assert(node->ChildCount != 1);
                //If there are only two children in the node, then the node containing the removed leaf will collapse.
                var otherIndex = leaf.ChildIndex == 0 ? 1 : 0;
                var otherChildIndex = nodeChildren[otherIndex];
                if (otherChildIndex >= 0)
                {
                    //The remaining child is an internal node. That's complicated: its children should be pulled upward.
                    ReplaceNodeUpOneLevel(leaf.LevelIndex + 1, otherChildIndex, leaf.NodeIndex);

                    //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                    RefitForRemoval(node, leaf.LevelIndex);

                }
                else
                {
                    //The remaining child is a leaf node. Whew!
                    //Move the leaf upward.
                    var parentNode = Levels[previousLevel].Nodes + node->Parent;
                    (&parentNode->A)[node->IndexInParent] = nodeBounds[otherIndex];
                    (&parentNode->ChildA)[node->IndexInParent] = otherChildIndex;
                    (&parentNode->LeafCountA)[node->IndexInParent] = 1;

                    //Update that leaf's reference in the leaves array.
                    var otherLeafIndex = Encode(otherChildIndex);
                    leaves[otherLeafIndex].LevelIndex = previousLevel;
                    leaves[otherLeafIndex].NodeIndex = node->Parent;
                    leaves[otherLeafIndex].ChildIndex = node->IndexInParent;

                    //Remove the now dead node.
                    Levels[leaf.LevelIndex].RemoveAt(leaf.NodeIndex);


                    //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                    //Note that this starts at the parent; we've already done the refit for the current level via collapse.
                    RefitForRemoval(parentNode, previousLevel);
                }

            }
            else
            {
                //The node has enough children that it should not collapse; just need to remove the leaf.

                var lastChildIndex = node->ChildCount - 1;
                if (leaf.ChildIndex < lastChildIndex)
                {
                    //The removed leaf is not the last child of the node it belongs to.
                    //Move the last node into the position of the removed node.
                    nodeBounds[leaf.ChildIndex] = nodeBounds[lastChildIndex];
                    nodeChildren[leaf.ChildIndex] = nodeChildren[lastChildIndex];
                    nodeLeafCounts[leaf.ChildIndex] = nodeLeafCounts[lastChildIndex];
                    if (nodeChildren[lastChildIndex] >= 0)
                    {
                        //The moved child is an internal node.
                        //Update the child's IndexInParent pointer.
                        var movedInternalNode = Levels[leaf.LevelIndex + 1].Nodes + nodeChildren[lastChildIndex];
                        movedInternalNode->IndexInParent = leaf.ChildIndex;
                    }
                    else
                    {
                        //The moved node is a leaf, so update the leaf array's reference.
                        leaves[Encode(nodeChildren[lastChildIndex])].ChildIndex = leaf.ChildIndex;
                    }
                }
                //Clear out the last slot.
                (&node->A)[lastChildIndex] = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                (&node->ChildA)[lastChildIndex] = -1;
                (&node->LeafCountA)[lastChildIndex] = 0;
                --node->ChildCount;

                //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                RefitForRemoval(node, leaf.LevelIndex);
            }


            //Delete the leaf from the leaves array.
            var lastIndex = LeafCount - 1;
            if (lastIndex != leafIndex)
            {
                //The removed leaf was in the middle of the leaves array. Take the last leaf and use it to fill the slot.
                //The node owner's index must be updated to point to the new location.
                var lastLeafOwner = Levels[leaves[lastIndex].LevelIndex].Nodes + leaves[lastIndex].NodeIndex;
                (&lastLeafOwner->ChildA)[leaves[lastIndex].ChildIndex] = Encode(leafIndex);
                leaves[leafIndex] = leaves[lastIndex];
            }
            leaves[lastIndex] = new Leaf();
            return new LeafMove { OriginalIndex = lastIndex, NewIndex = leafIndex };
        }
    }
}
