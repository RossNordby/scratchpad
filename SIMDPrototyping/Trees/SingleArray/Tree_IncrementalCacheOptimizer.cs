﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        unsafe void SwapNodes(int indexA, int indexB)
        {




            //var a = nodes + indexA;
            //var b = nodes + indexB;


            ////Notify every child of B that it has moved to A.
            //var children = &b->ChildA;
            //for (int i = 0; i < b->ChildCount; ++i)
            //{
            //    if (children[i] >= 0)
            //    {
            //        nodes[children[i]].Parent = indexA;
            //    }
            //    else
            //    {
            //        var leafIndex = Encode(children[i]);
            //        leaves[leafIndex].NodeIndex = indexA;
            //    }
            //}
            ////Notify every child of A that it has moved to B.
            //children = &a->ChildA;
            //for (int i = 0; i < a->ChildCount; ++i)
            //{
            //    if (children[i] >= 0)
            //    {
            //        nodes[children[i]].Parent = indexB;
            //    }
            //    else
            //    {
            //        var leafIndex = Encode(children[i]);
            //        leaves[leafIndex].NodeIndex = indexB;
            //    }
            //}
            ////In the event that B is the parent of A or vice versa, the above will update A or B's parent pointer.
            ////That invalidates the parent pointer until this entire function is complete.
            ////So, if e.g. B is a parent of A, updating the child pointer of the parent cannot make use of the parent index anymore.

            ////Update the child pointer in the parent. 
            ////A will mvoe to indexB.
            //if (a->Parent == indexA) //If B was the parent of A, this could happen and the parent pointer is invalidated.
            //    (&b->ChildA)[a->IndexInParent] = indexB;
            //else
            //    (&nodes[a->Parent].ChildA)[a->IndexInParent] = indexB;

            ////B will move to indexA.
            //if (b->Parent == indexB) //If A was the parent of B, this could happen and the parent pointer is invalidated.
            //    (&a->ChildA)[b->IndexInParent] = indexA;
            //else
            //    (&nodes[b->Parent].ChildB)[b->IndexInParent] = indexA;


            //var temp = *a;
            //*a = *b;
            //*b = temp;

            var a = nodes + indexA;
            var b = nodes + indexB;

            var temp = *a;
            *a = *b;
            *b = temp;

            //Update the children pointers in the parents.
            //if (a->Parent == -1 || a->Parent >= nodeCount || b->Parent == -1 || b->Parent >= nodeCount)
            //    throw new Exception("baD");
            //if (a->Parent == indexA || a->Parent == indexB || b->Parent == indexA || b->Parent == indexB)
            //    Console.WriteLine("Unresolvable cycle");

            if (a->Parent == indexA)
            {
                //The original B's parent was A.
                //That parent has moved.
                a->Parent = indexB;
            }
            else if (b->Parent == indexB)
            {
                //The original A's parent was B.
                //that parent has moved.
                b->Parent = indexA;
            }
            (&nodes[a->Parent].ChildA)[a->IndexInParent] = indexA;
            (&nodes[b->Parent].ChildA)[b->IndexInParent] = indexB;


            //if ((&nodes[a->Parent].ChildA)[a->IndexInParent] != indexA)
            //{
            //    Console.WriteLine("HUH?");
            //}
            //if ((&nodes[b->Parent].ChildA)[b->IndexInParent] != indexB)
            //{
            //    Console.WriteLine("HUH?");
            //}
            //Update the parent pointers of the children.
            var children = &a->ChildA;
            for (int i = 0; i < a->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    //if (children[i] > nodeCount)
                    //    Console.WriteLine("bad");
                    nodes[children[i]].Parent = indexA;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    //if (leafIndex > leafCount)
                    //    Console.WriteLine("bad");
                    leaves[leafIndex].NodeIndex = indexA;
                }
            }
            children = &b->ChildA;
            for (int i = 0; i < b->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    //if (children[i] > nodeCount)
                    //    Console.WriteLine("bad");
                    nodes[children[i]].Parent = indexB;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    //if (leafIndex > leafCount)
                    //    Console.WriteLine("bad");
                    leaves[leafIndex].NodeIndex = indexB;
                }
            }

            //if ((&nodes[a->Parent].ChildA)[a->IndexInParent] != indexA)
            //{
            //    Console.WriteLine("HUH?");
            //}
            //if ((&nodes[b->Parent].ChildA)[b->IndexInParent] != indexB)
            //{
            //    Console.WriteLine("HUH?");
            //}

        }

        public unsafe void IncrementalCacheOptimize(int nodeIndex)
        {
            Debug.Assert(ChildrenCapacity == 2, "the multi-swap of children only works on 2-ary trees due to child count guarantees. Is it even necessary?");
            //This node can only be moved during the execution of this function if:
            //1) a descendant of this node's target position is nodeIndex: can't happen because all target positions are relative to, and higher than, this node index.
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            var targetIndex = nodeIndex + 1;


            //Note that we pull all children up to their final positions relative to the current node index.
            //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
            //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
            //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
            //to stil choose something which converged.
            //TODO: consider swapping children around so that the first child is the largest child. That maximizes the chance that the in-cache node is chosen,
            //because the probability of volume query traversal is proportional to volume. (Or surface area for rays...)
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (targetIndex >= nodeCount)
                {
                    //This attempted swap would reach beyond the allocated nodes.
                    //That means the current node is quite a bit a lower than it should be.
                    //Later refinement attempts should fix this, but for now, do nothing.
                    //Other options:
                    //We could aggressively swap this node upward. More complicated.
                    break;
                }
                if (children[i] >= 0)
                {
                    if (children[i] != targetIndex)
                    {
                        //Validate();
                        SwapNodes(children[i], targetIndex);
                        //Validate();
                    }
                    targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
                }
            }
            //for (int i = 0; i < node->ChildCount; ++i)
            //{
            //    if (children[i] == nodeIndex)
            //    {
            //        Console.WriteLine("This happened!");
            //    }
            //    //Note: while swapping into the final positions, as computed using leaf counts, guarantees
            //    //that the children will never need to move again, there is no hard requirement that they jump *here*.
            //    //So making this work for n-ary trees would look something like 'ignore the positioning of children that aren't the first one'.
            //    //Would be interesting to see the cache behavior of that.
            //    if (children[i] >= 0)
            //    {
            //        if (children[i] != targetIndex)
            //        {
            //            Validate();
            //            {
            //                var child = nodes + children[i];
            //                for (int j = 0; j < child->ChildCount; ++j)
            //                {
            //                    if ((&child->ChildA)[j] == targetIndex)
            //                        Console.WriteLine("asdf");
            //                }
            //            }
            //            SwapNodes(children[i], targetIndex);
            //            Validate();
            //        }
            //        targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
            //    }
            //}
            //var originalChildren = stackalloc int[node->ChildCount];
            //var originalLeafCounts = stackalloc int[node->ChildCount];
            //for (int i = 0; i < node->ChildCount; ++i)
            //{
            //    originalChildren[i] = children[i];
            //    originalLeafCounts[i] = leafCounts[i];
            //}
            //var originalCount = node->ChildCount;
            //for (int i = 0; i < node->ChildCount; ++i)
            //{
            //    if (children[i] >= 0)
            //    {
            //        IncrementalCacheOptimize(children[i]);
            //    }
            //    if (originalCount != node->ChildCount)
            //    {
            //        Console.WriteLine("expectation badly violated; current node was corrupted. probably moved elsewhere.");
            //    }
            //    for (int j = 0; j < node->ChildCount; ++j)
            //    {
            //        if (leafCounts[j] != originalLeafCounts[j])
            //        {
            //            Console.WriteLine("Expectation badly violated; current node was corrupted. probably moved elsewhere.");
            //        }
            //    }
            //    for (int j = 0; j < node->ChildCount; ++j)
            //    {
            //        if (children[j] != originalChildren[j])
            //        {
            //            Console.WriteLine("Expectation violated, child pointers were moved despite being in ostensibly final positions.");
            //        }
            //    }
            //}

        }
    }
}
