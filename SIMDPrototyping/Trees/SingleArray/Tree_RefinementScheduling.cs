using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
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

        unsafe float RefitRefine2(int nodeIndex, ref BoundingBox boundingBox)
        {
            var node = nodes + nodeIndex;
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            var premetric = ComputeBoundsMetric(ref boundingBox);
            if (node->ChildA >= 0)
            {
                RefitRefine2(node->ChildA, ref node->A);
            }
            if (node->ChildB >= 0)
            {
                RefitRefine2(node->ChildB, ref node->B);
            }
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            var postmetric = ComputeBoundsMetric(ref boundingBox);

            return postmetric - premetric;

        }
        unsafe void RefitRefine2()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootChildren = &nodes->ChildA;
            var rootBounds = &nodes->A;
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
                if (rootChildren[i] >= 0)
                {
#if NODE2
                    Refit2(rootChildren[i], out rootBounds[i]);
#else
                    Refit(rootChildren[i], out rootBounds[i]);
#endif
                }
            }
        }


        //public unsafe void RefitRefine(int maximumSubtrees, float threshold)
        //{
        //    var pool = BufferPools<int>.Thread;
        //    var spareNodes = new QuickList<int>(pool, 8);
        //    var subtreeReferences = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
        //    var treeletInternalNodes = new QuickQueue<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
        //    int[] buffer;
        //    MemoryRegion region;
        //    BinnedResources resources;
        //    CreateBinnedResources(pool, maximumSubtrees, out buffer, out region, out resources);

        //    //This is a little complicated.
        //    //The process goes like this:
        //    //1) Refit, and while refitting, compare the previous volume to the current volume for each node. Also sum up the total divergence as you go.
        //    //2) Use the total divergence to compute the aggressiveness of the periodic refinement, and mark nodes for periodic refinement.
        //    //3) Visit all nodes depth first. Mark any node that is sufficiently distant from a node that was marked for refinement by the periodic phase,
        //    //   and which has a high divergence score.


        //    BoundingBox boundingBox;
        //    bool nodesInvalidated;
        //    RefitRefine(0, maximumSubtrees, threshold, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, ref resources, out boundingBox, out nodesInvalidated);


        //    RemoveUnusedInternalNodes(ref spareNodes);
        //    region.Dispose();
        //    pool.GiveBack(buffer);
        //    spareNodes.Dispose();
        //}



        //unsafe void MarkPeriodic(int maximumSubtrees, float aggressiveness, ref QuickList<int> markedNodes)
        //{
        //    int minimumLeafCount = (int)Math.Min(leafCount, maximumSubtrees * 0.75f);
        //    //TODO: the vast majority if indices tested by this loop will be rejected by the leaf count minimum.
        //    //Probably a lot faster to do a top down. Would also simplify the leaf count management.
        //    var startIndex = (int)((t * 79151L) % skip);
        //    const int skip = 8;
        //    for (int i = startIndex; i < tree.NodeCount; i += skip)
        //    {
        //        //Avoid refitting any node which doesn't have enough children to warrant a full refine.
        //        var node = nodes + i;
        //        //TODO: Check if asking the parent is faster for this.
        //        var leafCounts = &node->LeafCountA;
        //        int leafCount = 0;
        //        for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
        //        {
        //            leafCount += leafCounts[childIndex];
        //        }

        //        if (leafCount >= minimumLeafCount)
        //        {
        //            markedNodes.Add(i);
                   
        //        }
        //    }
        //}


        //unsafe void RefitAndScore(int nodeIndex, int maximumSubtrees, float threshold,
        //    ref QuickList<int> subtreeReferences, ref QuickQueue<int> treeletInternalNodes, ref QuickList<int> spareNodes, ref BinnedResources resources, 
        //    float[] divergence, out BoundingBox boundingBox, out bool nodesInvalidated)
        //{
        //    var node = nodes + nodeIndex;
        //    //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
        //    Debug.Assert(node->ChildCount >= 2);
        //    nodesInvalidated = false;



        //    BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
        //    var premetric = ComputeBoundsMetric(ref boundingBox);
        //    if (node->ChildA >= 0)
        //    {
        //        bool invalidated;
        //        RefitRefine(node->ChildA, maximumSubtrees, threshold, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, ref resources, out node->A, out invalidated);
        //        if (invalidated)
        //        {
        //            node = nodes + nodeIndex;
        //            nodesInvalidated = true;
        //        }
        //    }
        //    if (node->ChildB >= 0)
        //    {
        //        bool invalidated;
        //        RefitRefine(node->ChildB, maximumSubtrees, threshold, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, ref resources, out node->B, out invalidated);
        //        if (invalidated)
        //        {
        //            node = nodes + nodeIndex;
        //            nodesInvalidated = true;
        //        }
        //    }
        //    BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);

        //    //TODO: doing this ahead of time would save a lot of time. Consider what happens when a leaf node gets teleported- you can get a chain of refines all the way to the root without a lot of benefit.
        //    //Youw ill be using out of date information to refine on, though. uncf.

        //    //BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
        //    var metric = ComputeBoundsMetric(ref boundingBox);
        //    if (metric > premetric * threshold)
        //    {
        //        bool invalidated;
        //        BinnedRefine(nodeIndex, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out invalidated);
        //        BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
        //        subtreeReferences.Count = 0;
        //        if (invalidated)
        //            nodesInvalidated = true;
        //    }


        //}

    }
}
