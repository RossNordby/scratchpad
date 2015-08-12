using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
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

        public unsafe void IncrementalCacheOptimizeMultithreaded(int nodeIndex)
        {
            //Multithreaded cache optimization attempts to acquire a lock on every involved node.
            //If any lock fails, it just abandons the entire attempt.
            //That's acceptable- the incremental optimization only cares about eventual success.
            var node = nodes + nodeIndex;
            bool nodeLock = 0 == Interlocked.CompareExchange(ref node->RefineFlag, 1, 0);
            if (nodeLock)
            {
                var children = &node->ChildA;
                //Note that the child pointers are not guaranteed to be unchanging just because we locked the node.
                //That requires locking all the children.
                //bool allChildrenLocked = true;
                //for (int i = 0; i < node->ChildCount; ++i)
                //{
                //    //It is very possible that this child pointer could swap between now and the compare exchange read. 
                //    //However, a child pointer will not turn from an internal node (positive) to a leaf node (negative), and that's all that matters.
                //    if (children[i] >= 0)
                //    {
                //        var lockTaken = 0 == Interlocked.CompareExchange(ref nodes[children[i]].RefineFlag, 1, 0);
                //        if (!lockTaken)
                //        {
                //            allChildrenLocked = false;
                //            //Release any child locks. Note that these children pointers cannot have changed due to the lock we already took.
                //            for (int j = 0; j < i; ++j)
                //            {
                //                if (children[i] >= 0)
                //                {
                //                    nodes[children[i]].RefineFlag = 0;
                //                }
                //            }
                //            break;
                //        }
                //    }
                //}
                //if (allChildrenLocked)
                {
                    var leafCounts = &node->LeafCountA;
                    var targetIndex = nodeIndex + 1;


                    //Note that we pull all children up to their final positions relative to the current node index.
                    //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
                    //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
                    //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
                    //to stil choose something which converged.

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
                        //It is very possible that this child pointer could swap between now and the compare exchange read. 
                        //However, a child pointer will not turn from an internal node (positive) to a leaf node (negative), and that's all that matters.
                        if (children[i] >= 0)
                        {
                            //Lock before comparing the children to stop the children from changing.
                            if (0 == Interlocked.CompareExchange(ref nodes[children[i]].RefineFlag, 1, 0))
                            {
                                if (children[i] != targetIndex)
                                {
                                    var originalChildIndex = children[i];
                                    if (0 == Interlocked.CompareExchange(ref nodes[targetIndex].RefineFlag, 1, 0))
                                    {
                                        SwapNodes(originalChildIndex, targetIndex);
                                        //Unlock.
                                        //The refined node was swapped into the child's old position. Clear its flag.
                                        nodes[originalChildIndex].RefineFlag = 0;
                                        //break;
                                    }
                                }
                                //Unlock. children[i] is either the targetIndex, if a swap went through, or it's the original child index if it didn't.
                                //Those are the proper targets.
                                nodes[children[i]].RefineFlag = 0;
                            }
                            //Leafcounts cannot change due to other threads.
                            targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
                        }
                    }


                    ////Unlock children.
                    //for (int j = 0; j < node->ChildCount; ++j)
                    //{
                    //    if (children[j] >= 0)
                    //    {
                    //        nodes[children[j]].RefineFlag = 0;
                    //    }
                    //}
                }
                //Unlock the node.
                node->RefineFlag = 0;
            }
            //for (int i = 0; i < nodeCount; ++i)
            //{
            //    if (nodes[i].RefineFlag != 0)
            //    {
            //        Console.WriteLine("bad");
            //    }
            //}
        }

        public unsafe void IncrementalCacheOptimize(int nodeIndex)
        {
            Debug.Assert(ChildrenCapacity == 2, "the multi-swap of children only works on 2-ary trees due to child count guarantees. Is it even necessary?");
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;


            //{

            //    int largestIndex = -1;
            //    float largestMetric = 0;
            //    var bounds = &node->A;
            //    for (int i = 0; i < node->ChildCount; ++i)
            //    {
            //        if (children[i] >= 0) //Only swap internal nodes forward, because leaf nodes are irrelevant to cache behavior.
            //        {
            //            var metric = ComputeBoundsMetric(ref bounds[i]);
            //            if (metric > largestMetric)
            //            {
            //                largestIndex = i;
            //                largestMetric = metric;
            //            }
            //        }
            //    }
            //    if (largestIndex > 0)
            //    {
            //        //The largest index should be in the first slot, because the first slot is stored contiguously.
            //        //(There are other ways to guarantee this- like during construction, or even just choosing different target indices above-
            //        //but this just makes things simple.)
            //        var tempBounds = bounds[0];
            //        bounds[0] = bounds[largestIndex];
            //        bounds[largestIndex] = tempBounds;
            //        var tempChild = children[0];
            //        children[0] = children[largestIndex];
            //        children[largestIndex] = tempChild;
            //        var tempLeafCount = leafCounts[0];
            //        leafCounts[0] = leafCounts[largestIndex];
            //        leafCounts[largestIndex] = tempLeafCount;

            //        if (children[0] >= 0)
            //        {
            //            nodes[children[0]].IndexInParent = 0;
            //        }
            //        else
            //        {
            //            var leafIndex = Encode(children[0]);
            //            leaves[leafIndex].ChildIndex = 0;
            //        }
            //        if (children[largestIndex] >= 0)
            //        {
            //            nodes[children[largestIndex]].IndexInParent = largestIndex;
            //        }
            //        else
            //        {
            //            var leafIndex = Encode(children[largestIndex]);
            //            leaves[leafIndex].ChildIndex = largestIndex;
            //        }
            //    }
            //}




            var targetIndex = nodeIndex + 1;


            //Note that we pull all children up to their final positions relative to the current node index.
            //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
            //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
            //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
            //to stil choose something which converged.

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
                    //break;
                    targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
                }
            }



        }


    }

}
