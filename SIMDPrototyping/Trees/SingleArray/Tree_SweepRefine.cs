using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        struct SweepSubtree
        {
            public BoundingBox BoundingBox;
            public int Index;
            public int LeafCount;
        }

        unsafe void CreateStagingNode(int parentIndex, int indexInParent, SweepSubtree* subtrees, int subtreeStart, int subtreeCount, Node* stagingNodes, ref int stagingNodeCount, out float treeletCost)
        {

        }

        public unsafe void SweepRefine(int nodeIndex, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {
            const int maximumSubtrees = 256;
            var subtrees = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int internalNodeStartIndex = internalNodes.Count;
            float originalTreeletCost;
            CollectSubtrees(0, maximumSubtrees, ref subtrees, ref internalNodes, out originalTreeletCost);

            //Gather necessary information from nodes. (TODO: This could be more efficiently gathered up front... collectsubtrees already touched most of this data!)
            var sweepSubtrees = stackalloc SweepSubtree[subtrees.Count];
            for (int i = 0; i < subtrees.Count; ++i)
            {
                var subtree = sweepSubtrees + i;
                subtree->Index = subtrees.Elements[i];
                if (subtree->Index >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtree->Index;
                    var parentNode = nodes + subtreeNode->Parent;
                    subtree->BoundingBox = (&parentNode->A)[subtreeNode->IndexInParent];
                    subtree->LeafCount = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    subtree->LeafCount = 1;
                    var leaf = leaves + Encode(subtree->Index);
                    subtree->BoundingBox = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                }
            }

            var node = nodes + nodeIndex;
            int parent = node->Parent;
            int indexInParent = node->IndexInParent;

            //Now perform a top-down sweep build.
            //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
            //If you end up making others, keep this in mind.
            int stagingNodeCount = 0;
            int stagingNodeCapacity = subtrees.Count - 1;
            var stagingNodes = stackalloc Node[stagingNodeCapacity];
            float newTreeletCost;
            CreateStagingNode(parent, indexInParent, sweepSubtrees, 0, subtrees.Count, stagingNodes, ref stagingNodeCount, out newTreeletCost);

            if (newTreeletCost < originalTreeletCost)
            {
                //Reify the nodes.
                ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtrees, ref internalNodes, out nodesInvalidated);
            }
            else
            {
                //The internal nodes collected by the most recent iteration of CollectSubtrees weren't replaced! Get them out of the pool.
                //TODO: Would be nice to do this in a slightly less gross way.
                for (int i = internalNodes.Count - 1; i >= internalNodeStartIndex; --i)
                {
                    internalNodes.FastRemoveAt(i);
                }
                nodesInvalidated = false;
            }

            subtrees.Dispose();
        }
    }
}
