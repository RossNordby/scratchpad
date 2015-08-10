using System;
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
            var a = nodes + indexA;
            var b = nodes + indexB;

            var temp = *a;
            *a = *b;
            *b = temp;

            //Update the children pointers in the parents.
            if (a->Parent == -1 || a->Parent >= nodeCount || b->Parent == -1 || b->Parent >= nodeCount)
                throw new Exception("baD");
            (&nodes[a->Parent].ChildA)[a->IndexInParent] = indexA;
            (&nodes[b->Parent].ChildA)[b->IndexInParent] = indexB;
            //Update the parent pointers of the children.
            var children = &a->ChildA;
            for (int i = 0; i < a->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    if (children[i] > nodeCount)
                        Console.WriteLine("bad");
                    nodes[children[i]].Parent = indexA;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    if (leafIndex > leafCount)
                        Console.WriteLine("bad");
                    leaves[leafIndex].NodeIndex = indexA;
                }
            }
            children = &b->ChildA;
            for (int i = 0; i < b->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    if (children[i] > nodeCount)
                        Console.WriteLine("bad");
                    nodes[children[i]].Parent = indexB;
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    if (leafIndex > leafCount)
                        Console.WriteLine("bad");
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
            for (int i = 0; i < node->ChildCount; ++i)
            {
                //Note: while swapping into the final positions, as computed using leaf counts, guarantees
                //that the children will never need to move again, there is no hard requirement that they jump *here*.
                //So making this work for n-ary trees would look something like 'ignore the positioning of children that aren't the first one'.
                //Would be interesting to see the cache behavior of that.
                if (children[i] >= 0)
                {
                    if (children[i] != targetIndex)
                    {
                        SwapNodes(children[i], targetIndex);
                    }
                    targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
                }
            }
            var originalChildren = stackalloc int[node->ChildCount];
            var originalLeafCounts = stackalloc int[node->ChildCount];
            for (int i = 0; i < node->ChildCount; ++i)
            {
                originalChildren[i] = children[i];
                originalLeafCounts[i] = leafCounts[i];
            }
            var originalCount = node->ChildCount;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    IncrementalCacheOptimize(children[i]);
                }
                if (originalCount != node->ChildCount)
                {
                    Console.WriteLine("expectation badly violated; current node was corrupted. probably moved elsewhere.");
                }
                for (int j = 0; j < node->ChildCount; ++j)
                {
                    if (leafCounts[j] != originalLeafCounts[j])
                    {
                        Console.WriteLine("Expectation badly violated; current node was corrupted. probably moved elsewhere.");
                    }
                }
                for (int j = 0; j < node->ChildCount; ++j)
                {
                    if (children[j] != originalChildren[j])
                    {
                        Console.WriteLine("Expectation violated, child pointers were moved despite being in ostensibly final positions.");
                    }
                }
            }

        }
    }
}
