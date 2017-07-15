﻿using BEPUutilities2;
using System;
using System.Diagnostics;
using System.Numerics;


namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        unsafe void RemoveNodeAt(int nodeIndex)
        {
            //Note that this function is a cache scrambling influence. That's okay- the cache optimization routines will take care of it later.
            Debug.Assert(nodeIndex < nodeCount && nodeIndex >= 0);
            //We make no guarantees here about maintaining the tree's coherency after a remove.
            //That's the responsibility of whoever called RemoveAt.
            --nodeCount;
            //If the node wasn't the last node in the list, it will be replaced by the last node.
            if (nodeIndex < nodeCount)
            {
                //Swap last node for removed node.
                var node = nodes + nodeIndex;
                *node = nodes[nodeCount];

                //Update the moved node's pointers:
                //its parent's child pointer should change, and...
                (&nodes[node->Parent].A)[node->IndexInParent].Index = nodeIndex;
                //its children's parent pointers should change.
                var nodeChildren = &node->A;
                for (int i = 0; i < 2; ++i)
                {
                    ref var child = ref nodeChildren[i];
                    if (child.Index >= 0)
                    {
                        nodes[child.Index].Parent = nodeIndex;
                    }
                    else
                    {
                        //It's a leaf node. It needs to have its pointers updated.
                        leaves[Encode(child.Index)] = new Leaf(nodeIndex, i);
                    }
                }

            }


        }


        unsafe void RefitForRemoval(Node* node)
        {
            //Note that no attempt is made to refit the root node. Note that the root node is the only node that can have a number of children less than 2.
            while (node->Parent >= 0)
            {
                //Compute the new bounding box for this node.
                var parent = nodes + node->Parent;
                ref var childInParent = ref (&parent->A)[node->IndexInParent];
                BoundingBox.CreateMerged(ref node->A.Min, ref node->A.Max, ref node->B.Min, ref node->B.Max, out childInParent.Min, out childInParent.Max);
                --childInParent.LeafCount;
                node = parent;
            }
        }

        /// <summary>
        /// Removes a leaf at an index. If the index is not at the end of the leaf list, the last leaf is swapped into the removed location.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf to remove.</param>
        /// <returns>Former index of the leaf that was moved into the removed leaf's slot, if any.
        /// If leafIndex pointed at the last slot in the list, then this returns -1 since no leaf was moved.</returns>
        public unsafe int RemoveAt(int leafIndex)
        {
            if (leafIndex < 0 || leafIndex >= leafCount)
                throw new ArgumentOutOfRangeException("Leaf index must be a valid index in the tree's leaf array.");

            //Cache the leaf being removed.
            var leaf = leaves[leafIndex];
            //Delete the leaf from the leaves array.
            --leafCount;
            if (leafIndex < leafCount)
            {
                //The removed leaf was not the last leaf, so we should move the last leaf into its slot.
                //This can result in a form of cache scrambling, but these leaves do not need to be referenced during high performance stages.
                //It does somewhat reduce the performance of AABB updating, but we shouldn't bother with any form of cache optimization for this unless it becomes a proven issue.
                ref var lastLeaf = ref leaves[leafCount];
                leaves[leafIndex] = lastLeaf;
                (&nodes[lastLeaf.NodeIndex].A)[lastLeaf.ChildIndex].Index = Encode(leafIndex);
            }

            var node = nodes + leaf.NodeIndex;
            var nodeChildren = &node->A;

            //Remove the leaf from this node.
            //Note that the children must remain contiguous. Requires taking the last child of the node and moving into the slot
            //if the removed child was not the last child.
            //Further, if a child is moved and if it is a leaf, that leaf's ChildIndex must be updated.
            //If a child is moved and it is an internal node, all immediate children of that node must have their parent nodes updated.

            var survivingChildIndexInNode = leaf.ChildIndex ^ 1;
            ref var survivingChild = ref nodeChildren[survivingChildIndexInNode];

            //Check to see if this node should collapse.
            if (node->Parent >= 0)
            {
                //This is a non-root internal node.
                //Since there are only two children in the node, then the node containing the removed leaf will collapse.
                Debug.Assert(node->ChildCount == 2);

                //Move the other node into the slot that used to point to the collapsing internal node.
                var parentNode = nodes + node->Parent;
                ref var childInParent = ref (&parentNode->A)[node->IndexInParent];
                childInParent.Min = survivingChild.Min;
                childInParent.Max = survivingChild.Max;
                childInParent.Index = survivingChild.Index;
                childInParent.LeafCount = survivingChild.LeafCount;

                if (survivingChild.Index < 0)
                {
                    //It's a leaf. Update the leaf's reference in the leaves array.
                    var otherLeafIndex = Encode(survivingChild.Index);
                    leaves[otherLeafIndex] = new Leaf(node->Parent, node->IndexInParent);
                }
                else
                {
                    //It's an internal node. Update its parent node.
                    nodes[survivingChild.Index].Parent = node->Parent;
                    nodes[survivingChild.Index].IndexInParent = node->IndexInParent;

                }


                //Remove the now dead node.
                RemoveNodeAt(leaf.NodeIndex);

                //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                //Note that this starts at the parent; we've already done the refit for the current level via collapse.
                RefitForRemoval(parentNode);

            }
            else
            {
                //This is the root. It cannot collapse, but if the other child is an internal node, then it will overwrite the root node.
                //This maintains the guarantee that any tree with at least 2 leaf nodes has every single child slot filled with a node or leaf.
                Debug.Assert(nodes == node, "Only the root should have a negative parent, so only the root should show up here.");
                if (leafCount > 0)
                {
                    //The post-removal leafCount is still positive, so there must be at least one child in the root node.
                    //If it is an internal node, then it will be promoted into the root node's slot.
                    if (survivingChild.Index >= 0)
                    {
                        //The surviving child is an internal node and it should replace the root node.
                        var pulledNodeIndex = survivingChild.Index;
                        //TODO: This node movement logic could be unified with other instances of node moving. Nothing too special about the fact that it's the root.
                        *nodes = nodes[pulledNodeIndex]; //Note that this overwrites the memory pointed to by the otherChild reference.
                        nodes->Parent = -1;
                        nodes->IndexInParent = -1;
                        //Update the parent pointers of the children of the moved internal node.
                        for (int i = 0; i < 2; ++i)
                        {
                            ref var child = ref (&nodes->A)[i];
                            if (child.Index >= 0)
                            {
                                //Child is an internal node. Note that the index in child doesn't change; we copied the children directly.
                                nodes[child.Index].Parent = 0;
                            }
                            else
                            {
                                //Child is a leaf node.
                                leaves[Encode(child.Index)] = new Leaf(0, i);
                            }
                        }
                        RemoveNodeAt(pulledNodeIndex);
                    }
                    else
                    {
                        //The surviving child is a leaf node.
                        if (survivingChildIndexInNode > 0)
                        {
                            //It needs to be moved to keep the lowest slot filled.
                            nodes->A = nodes->B;
                            //Update the leaf pointer to reflect the change.
                            leaves[Encode(survivingChild.Index)] = new Leaf(0, 0);
                        }
                        nodes->ChildCount = 1;
                    }
                }
                else
                {
                    nodes->ChildCount = 0;
                }

                //No need to perform a RefitForRemoval here; it's the root. There is no higher bounding box.
            }
            return leafIndex < leafCount ? leafCount : -1;
        }
    }
}
