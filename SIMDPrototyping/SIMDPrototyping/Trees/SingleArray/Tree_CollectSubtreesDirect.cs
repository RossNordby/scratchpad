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


    partial class Tree
    {
        unsafe void CollectSubtreesForNodeDirect(int nodeIndex, int remainingDepth, ref QuickList<int> subtrees, ref QuickQueue<int> internalNodes, out float treeletCost)
        {
            internalNodes.Enqueue(nodeIndex);

            treeletCost = 0;
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var bounds = &node->A;

            --remainingDepth;
            if (remainingDepth >= 0)
            {
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    if (children[i] >= 0)
                    {
                        treeletCost += ComputeBoundsMetric(ref bounds[i]);
                        float childCost;
                        CollectSubtreesForNodeDirect(children[i], remainingDepth, ref subtrees, ref internalNodes, out childCost);
                        treeletCost += childCost;
                    }
                    else
                    {
                        //It's a leaf, immediately add it to subtrees.
                        subtrees.Add(children[i]);
                    }
                }
            }
            else
            {
                //Recursion has bottomed out. Add every child.
                //Once again, note that the treelet costs of these nodes are not considered, even if they are internal.
                //That's because the subtree internal nodes cannot change size due to the refinement.
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    subtrees.Add(children[i]);
                }
            }
        }

        public unsafe void CollectSubtreesDirect(int nodeIndex, int maximumSubtrees, ref QuickList<int> subtrees, ref QuickQueue<int> internalNodes, out float treeletCost)
        {
            //TODO: doesn't work for non-node2.
            Debug.Assert(ChildrenCapacity == 2, "If you're going to use >2ary trees, you need to correct the maximum depth. Probably better to use explicit recursion depth as input...");
            var maximumDepth = BufferPool<int>.GetPoolIndex(maximumSubtrees) - 1;
            Debug.Assert(maximumDepth > 0);
            //Cost excludes the treelet root, since refinement can't change the treelet root's size. So don't bother including it in treeletCost.

            CollectSubtreesForNodeDirect(nodeIndex, maximumDepth, ref subtrees, ref internalNodes, out treeletCost);


        }




    }
}
