using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;


namespace SIMDPrototyping.Trees.SingleArray
{
    public struct LeafMove
    {
        public int OriginalIndex;
        public int NewIndex;
    }
    partial class Tree
    {
        unsafe void RemoveNodeAt(int nodeIndex)
        {
            if (NodeCount <= nodeIndex)
                Console.WriteLine("sup:)");
            ValidateLeaves();
            Debug.Assert(nodeIndex < NodeCount && nodeIndex >= 0);
            //We make no guarantees here about maintaining the tree's coherency after a remove.
            //That's the responsibility of whoever called RemoveAt.
            if (nodeIndex == NodeCount - 1)
            {
                //Last node; just remove directly.
                --NodeCount;
                Console.WriteLine("Leaf");
            }
            else
            {
                ValidateLeaves();
                //Swap last node for removed node.
                --NodeCount;
                var node = Nodes + nodeIndex;
                *node = Nodes[NodeCount];

                //Update the moved node's pointers:
                //its parent's child pointer should change, and
                (&Nodes[node->Parent].ChildA)[node->IndexInParent] = nodeIndex;
                //its children's parent pointers should change.
                var nodeChildren = &node->ChildA;
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    if (nodeChildren[i] >= 0)
                    {
                        Nodes[nodeChildren[i]].Parent = nodeIndex;
                    }
                    else
                    {
                        //It's a leaf node. It needs to have its pointers updated.
                        Leaves[Encode(nodeChildren[i])].NodeIndex = nodeIndex;
                    }
                }
                ValidateLeaves();
                Console.WriteLine("Internal");

            }


            ValidateLeaves();
        }


       
        unsafe void RefitForRemoval(Node* node)
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
                var parent = Nodes + node->Parent;
                (&parent->A)[node->IndexInParent] = merged;
                --(&parent->LeafCountA)[node->IndexInParent];

                node = parent;
            }
            ValidateLeaves();
        }

        public unsafe LeafMove RemoveAt(int leafIndex)
        {
            ValidateLeaves();
            if (leafIndex < 0 || leafIndex >= leafCount)
                throw new ArgumentOutOfRangeException("Leaf index must be a valid index in the tree's leaf array.");

            //Cache the leaf before overwriting.
            var leaf = Leaves[leafIndex];

            //Delete the leaf from the leaves array.
            //This is done up front to make it easier for tests to catch bad behavior.
            var lastIndex = LeafCount - 1;
            if (lastIndex != leafIndex)
            {
                //The removed leaf was in the middle of the leaves array. Take the last leaf and use it to fill the slot.
                //The node owner's index must be updated to point to the new location.
                var lastLeafOwner = Nodes + Leaves[lastIndex].NodeIndex;
                (&lastLeafOwner->ChildA)[Leaves[lastIndex].ChildIndex] = Encode(leafIndex);
                Leaves[leafIndex] = Leaves[lastIndex];
            }
            Leaves[lastIndex] = new Leaf();
            leafCount = lastIndex;

            var node = Nodes + leaf.NodeIndex;
            var nodeChildren = &node->ChildA;
            var nodeBounds = &node->A;
            var nodeLeafCounts = &node->LeafCountA;

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

                //Move the other node into the slot that used to point to the collapsing internal node.
                var parentNode = Nodes + node->Parent;
                (&parentNode->A)[node->IndexInParent] = nodeBounds[otherIndex];
                (&parentNode->ChildA)[node->IndexInParent] = otherChildIndex;
                (&parentNode->LeafCountA)[node->IndexInParent] = 1;

                if (otherChildIndex < 0)
                {
                    //It's a leaf. Update the leaf's reference in the leaves array.
                    var otherLeafIndex = Encode(otherChildIndex);
                    Leaves[otherLeafIndex].NodeIndex = node->Parent;
                    Leaves[otherLeafIndex].ChildIndex = node->IndexInParent;
                }


                //Remove the now dead node.
                RemoveNodeAt(leaf.NodeIndex);

                //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                //Note that this starts at the parent; we've already done the refit for the current level via collapse.
                RefitForRemoval(parentNode);

            }
            else
            {
                //The node has enough children that it should not collapse or it's the root; just need to remove the leaf.

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
                        var movedInternalNode = Nodes + nodeChildren[lastChildIndex];
                        movedInternalNode->IndexInParent = leaf.ChildIndex;
                    }
                    else
                    {
                        //The moved node is a leaf, so update the leaf array's reference.
                        Leaves[Encode(nodeChildren[lastChildIndex])].ChildIndex = leaf.ChildIndex;
                    }
                }
                //Clear out the last slot.
                (&node->A)[lastChildIndex] = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                (&node->ChildA)[lastChildIndex] = -1;
                (&node->LeafCountA)[lastChildIndex] = 0;
                --node->ChildCount;

                //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                RefitForRemoval(node);
            }



            ValidateLeaves();

            return new LeafMove { OriginalIndex = lastIndex, NewIndex = leafIndex };
        }
    }
}
