using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        //public unsafe void RefitRefine(int maximumSubtrees, float threshold)
        //{


        //    BoundingBox boundingBox;
        //    RefitRefine(0, maximumSubtrees, threshold, out boundingBox, out nodesInvalidated);

        //}


        //unsafe void RefitRefine(int nodeIndex, int maximumSubtrees, float threshold,
        //    ref QuickList<int> subtreeReferences, ref QuickList<int> spareNodes, ref BinnedResources resources, out BoundingBox boundingBox, out bool nodesInvalidated)
        //{
        //    var node = nodes + nodeIndex;
        //    //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
        //    Debug.Assert(node->ChildCount >= 2);
        //    nodesInvalidated = false;



        //    if (node->ChildA >= 0)
        //    {
        //        bool invalidated;
        //        RefitRefine(node->ChildA, maximumSubtrees, threshold, ref subtreeReferences, ref spareNodes, ref resources, out node->A, out invalidated);
        //        if (invalidated)
        //        {
        //            node = nodes + nodeIndex;
        //            nodesInvalidated = true;
        //        }
        //    }
        //    if (node->ChildB >= 0)
        //    {
        //        bool invalidated;
        //        RefitRefine(node->ChildB, maximumSubtrees, threshold, ref subtreeReferences, ref spareNodes, ref resources, out node->B, out invalidated);
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
        //    if (metric > node->PreviousMetric * threshold)
        //    {
        //        bool invalidated;
        //        BinnedRefine(nodeIndex, ref subtreeReferences, maximumSubtrees, ref spareNodes, ref resources, out invalidated);
        //        BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
        //        node->PreviousMetric = ComputeBoundsMetric(ref boundingBox);
        //        subtreeReferences.Count = 0;
        //        if (invalidated)
        //            nodesInvalidated = true;
        //    }


        //}
    }
}
