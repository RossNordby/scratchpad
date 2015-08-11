using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        unsafe void SwapNodes(int indexA, int indexB)
        {
            var a = nodes + indexA;
            var b = nodes + indexB;

            var temp = *a;
            *a = *b;
            *b = temp;

            if (a->Parent == indexA)
            {
                //The original B's parent was A.
                //That parent has moved.
                a->Parent = indexB;
            }
            else if (b->Parent == indexB)
            {
                //The original A's parent was B.
                //That parent has moved.
                b->Parent = indexA;
            }
            (&nodes[a->Parent].ChildA)[a->IndexInParent] = indexA;
            (&nodes[b->Parent].ChildA)[b->IndexInParent] = indexB;


            //Update the parent pointers of the children.
            var children = &a->ChildA;
            for (int i = 0; i < a->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    nodes[children[i]].Parent = indexA;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    leaves[leafIndex].NodeIndex = indexA;
                }
            }
            children = &b->ChildA;
            for (int i = 0; i < b->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    nodes[children[i]].Parent = indexB;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    leaves[leafIndex].NodeIndex = indexB;
                }
            }

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

            int largestIndex = 0;
            float largestMetric = ComputeBoundsMetric(ref node->A);
            var bounds = &node->A;
            for (int i = 1; i < node->ChildCount; ++i)
            {
                var metric = ComputeBoundsMetric(ref bounds[i]);
                if (metric > largestMetric)
                {
                    largestIndex = i;
                    largestMetric = metric;
                }
            }
            if (largestIndex != 0)
            {
                //The largest index should be in the first slot, because the first slot is stored contiguously.
                //(There are other ways to guarantee this- like during construction, or even just choosing different target indices above-
                //but this just makes things simple.)

            }


        }
    }
}
