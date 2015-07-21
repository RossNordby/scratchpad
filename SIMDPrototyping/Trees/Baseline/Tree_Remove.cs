using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;



namespace SIMDPrototyping.Trees.Baseline
{
    public struct LeafMove
    {
        public int OriginalIndex;
        public int NewIndex;
    }
    partial class Tree<T>
    {
        unsafe void RemoveNodeAt(int levelIndex, int nodeIndex)
        {
            if (Levels[levelIndex].Count <= nodeIndex)
                Console.WriteLine("sup:)");
            ValidateLeaves();
            Debug.Assert(nodeIndex < Levels[levelIndex].Count && nodeIndex >= 0);
            //We make no guarantees here about maintaining the tree's coherency after a remove.
            //That's the responsibility of whoever called RemoveAt.
            if (nodeIndex == Levels[levelIndex].Count - 1)
            {
                //Last node; just remove directly.
                --Levels[levelIndex].Count;
                Console.WriteLine("Leaf");
            }
            else
            {
                ValidateLeaves();
                //Swap last node for removed node.
                --Levels[levelIndex].Count;
                var node = Levels[levelIndex].Nodes + nodeIndex;
                *node = Levels[levelIndex].Nodes[Levels[levelIndex].Count];

                //Update the moved node's pointers:
                //its parent's child pointer should change, and
                (&Levels[levelIndex - 1].Nodes[node->Parent].ChildA)[node->IndexInParent] = nodeIndex;
                //its children's parent pointers should change.
                var nodeChildren = &node->ChildA;
                var nextLevel = levelIndex + 1;
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    if (nodeChildren[i] >= 0)
                    {
                        Levels[nextLevel].Nodes[nodeChildren[i]].Parent = nodeIndex;
                    }
                    else
                    {
                        //It's a leaf node. It needs to have its pointers updated.
                        leaves[Encode(nodeChildren[i])].NodeIndex = nodeIndex;
                        if (leaves[Encode(nodeChildren[i])].LevelIndex > maximumDepth)
                            Console.WriteLine("sup");
                    }
                }
                ValidateLeaves();
                Console.WriteLine("Internal");

            }

            if (Levels[levelIndex].Count == 0)
            {
                Debug.Assert(levelIndex == maximumDepth, "Any level reduced to no nodes by removal should only be the final level in the tree, or else there's a gap.");
                --maximumDepth;
                for (int i = 0; i < LeafCount; ++i)
                {
                    if (leaves[i].LevelIndex > maximumDepth)
                    {
                        Console.WriteLine("Invalid leaf level!");
                    }
                }
            }
            ValidateLeaves();
        }

        unsafe void PullUpChildren(int sourceLevelIndex, int sourceNodeIndex, int* childrenIndices, int childrenCount, int newParent)
        {
            if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
                Console.WriteLine("sup:)");
            ValidateLeaves();
            bool oldNodeReplaced = false;
            //The source node is now dead. Did the moved node have any internal node children?
            int childrenLevel = sourceLevelIndex + 1;
            int firstInternalChildIndex = childrenCount;
            for (int i = 0; i < childrenCount; ++i)
            {
                if (childrenIndices[i] >= 0)
                {
                    Console.WriteLine("Previous children:");
                    for (int j = 0; j < childrenCount; ++j)
                    {
                        Console.WriteLine(childrenIndices[j]);
                        if (childrenIndices[j] >= Levels[childrenLevel].Count)
                        {
                            Console.WriteLine("OdD");
                        }
                    }
                    //This child is internal. It should be pulled up into the source node itself.
                    //This reuses the node and avoids a pointless remove-add sequence.
                    ReplaceNodeUpOneLevel(childrenLevel, childrenIndices[i], sourceNodeIndex);
                    Console.WriteLine("New children:");
                    for (int j = 0; j < childrenCount; ++j)
                    {
                        Console.WriteLine(childrenIndices[j]);
                        if (childrenIndices[j] >= Levels[childrenLevel].Count)
                        {
                            Console.WriteLine("OdD");
                        }
                    }
                    //Update the child pointer to point at its new home.
                    childrenIndices[i] = sourceNodeIndex;

                    firstInternalChildIndex = i;
                    oldNodeReplaced = true;
                    break;
                }
            }
            if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
            {
                Console.WriteLine("baD");
            }
            if (!oldNodeReplaced)
            {
                //The old node wasn't replaced, meaning there were no internal node children.
                //Just get rid of that old node then.
                RemoveNodeAt(sourceLevelIndex, sourceNodeIndex);
            }
            else
            {
                //Walk through the remaining children to find any other internal nodes.
                for (int i = firstInternalChildIndex; i < childrenCount; ++i)
                {
                    if (childrenIndices[i] >= 0)
                    {
                        if (childrenIndices[i] >= Levels[childrenLevel].Count)
                            Console.WriteLine("BAD):");
                        //There are no spare nodes available, so we'll just have to create another one as we move it up.
                        MoveNodeUpOneLevel(childrenLevel, childrenIndices[i], newParent, i);
                    }
                }
                //this check is only hear because removal causing it is expected.
                if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
                {
                    Console.WriteLine("baD");
                }
            }
            ValidateLeaves();
        }
        
        unsafe void MoveNodeUpOneLevel(int sourceLevelIndex, int sourceNodeIndex, int newParentIndex, int indexInParent)
        {
            if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
            {
                Console.WriteLine("baD");
            }
            ValidateLeaves();
            var sourceNode = Levels[sourceLevelIndex].Nodes + sourceNodeIndex;
            //Just copy the source node into the higher level directly.
            var previousLevelIndex = sourceLevelIndex - 1;
            var newNodeIndex = Levels[previousLevelIndex].Add(ref *sourceNode);
            var newNode = Levels[previousLevelIndex].Nodes + newNodeIndex;
            //The index in the parent should not change, since we're not changing parents. Just following the parent upwards.
            Debug.Assert(indexInParent == newNode->IndexInParent);
            //The parent does change.
            newNode->Parent = newParentIndex;

            //The parent's pointer to us also will change.
            //The parent is on level sourceLevelIndex - 2; it's moved up. We're following.
            (&Levels[sourceLevelIndex - 2].Nodes[newParentIndex].ChildA)[indexInParent] = newNodeIndex;

            //Update any leaves that were direct children of this node.
            var children = &newNode->ChildA;
            for (int i = 0; i < newNode->ChildCount; ++i)
            {
                if (children[i] < 0)
                {
                    var leafIndex = Encode(children[i]);
                    leaves[leafIndex].LevelIndex = previousLevelIndex;
                    leaves[leafIndex].NodeIndex = newNodeIndex;

                    if (leaves[leafIndex].LevelIndex >= maximumDepth)
                        Console.WriteLine("say whatnow");
                    if (leaves[leafIndex].NodeIndex >= Levels[leaves[leafIndex].LevelIndex].Count)
                        Console.WriteLine("bad");

                }
            }
            if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
            {
                Console.WriteLine("baD");
            }
            PullUpChildren(sourceLevelIndex, sourceNodeIndex, children, newNode->ChildCount, newNodeIndex);

            ValidateLeaves();

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
            ValidateLeaves();
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

            //Update any leaves that were direct children of this node.
            for (int i = 0; i < destinationNode->ChildCount; ++i)
            {
                if (destinationChildren[i] < 0)
                {
                    var leafIndex = Encode(destinationChildren[i]);
                    leaves[leafIndex].LevelIndex = destinationLevelIndex;
                    leaves[leafIndex].NodeIndex = destinationNodeIndex;

                    if (leaves[leafIndex].LevelIndex >= maximumDepth)
                        Console.WriteLine("say whatnow");
                    if (leaves[leafIndex].NodeIndex >= Levels[leaves[leafIndex].LevelIndex].Count)
                        Console.WriteLine("bad");
                }
            }
            if (Levels[sourceLevelIndex].Count <= sourceNodeIndex)
            {
                Console.WriteLine("baD");
            }
            for (int i = 0; i < sourceNode->ChildCount; ++i)
            {
                if (Levels[sourceLevelIndex + 1].Count <= sourceChildren[i])
                {
                    Console.WriteLine("bad index");
                }
            }
            for (int i = 0; i < destinationNode->ChildCount; ++i)
            {
                if (Levels[sourceLevelIndex + 1].Count <= destinationChildren[i])
                {
                    Console.WriteLine("bad index");
                }
            }
            PullUpChildren(sourceLevelIndex, sourceNodeIndex, destinationChildren, destinationNode->ChildCount, destinationNodeIndex);
            for (int i = 0; i < sourceNode->ChildCount; ++i)
            {
                if (Levels[sourceLevelIndex + 1].Count <= sourceChildren[i])
                {
                    Console.WriteLine("bad index");
                }
            }
            for (int i = 0; i < destinationNode->ChildCount; ++i)
            {
                if (Levels[sourceLevelIndex].Count <= destinationChildren[i])
                {
                    Console.WriteLine("bad index");
                }
            }
            ValidateLeaves();
        }

        unsafe void RefitForRemoval(Node* node, int levelIndex)
        {
            ValidateLeaves();
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
            ValidateLeaves();
        }

        public unsafe LeafMove RemoveAt(int leafIndex)
        {
            ValidateLeaves();
            if (leafIndex < 0 || leafIndex >= leafCount)
                throw new ArgumentOutOfRangeException("Leaf index must be a valid index in the tree's leaf array.");

            //Cache the leaf before overwriting.
            var leaf = leaves[leafIndex];

            //Delete the leaf from the leaves array.
            //This is done up front to make it easier for tests to catch bad behavior.
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
            leafCount = lastIndex;

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

                    if (leaves[otherLeafIndex].LevelIndex > maximumDepth)
                    {
                        Console.WriteLine("sup");
                    }

                    //Remove the now dead node.
                    RemoveNodeAt(leaf.LevelIndex, leaf.NodeIndex);
                    if (leaves[otherLeafIndex].LevelIndex > maximumDepth)
                    {
                        Console.WriteLine("sup");
                    }

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



            ValidateLeaves();

            return new LeafMove { OriginalIndex = lastIndex, NewIndex = leafIndex };
        }
    }
}
