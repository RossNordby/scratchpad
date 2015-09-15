﻿using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
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

        unsafe void Attempt(int nodeIndex, ref int targetIndex, int min, int max)
        {
            //TWO KNOWNS:
            //1) The parent node is in its global optimum position.
            //2) All simultaneously executing cache optimizations target global optimums, so the parent node cannot move.

            //It is possible that the children will move
            var node = nodes + nodeIndex;
            var children = &node->ChildA;

            //Acquire a lock on all children that are outside of the range.
            //DANGER: DEADLOCKS. Need to be able to prove that it can't happen, but current organization makes it very likely.
            for (int i = 0; i < node->ChildCount; ++i)
            {
                //If the child is already within the subtree's dedicated memory range, we know that no other thread will interfere with it.
                // Consider:
                //  There is only one thread operating per logical subtree.
                //  Other threads 
                //
                //No lock is necessary.
                if (children[i] < min || children[i] >= max)
                {
                    //Lock(children[i]);
                }
            }

            //All children are now immobile.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe bool TryLock(ref int nodeIndex)
        {
            //Node index may change during the execution of this function.
            int lockedIndex;
            while (true)
            {
                lockedIndex = nodeIndex;
                if (0 != Interlocked.CompareExchange(ref nodes[lockedIndex].RefineFlag, 1, 0))
                {
                    //Abort.
                    return false;
                }
                if (lockedIndex != nodeIndex) //Compare exchange inserts memory barrier.
                {
                    //Locked the wrong node, let go.
                    nodes[lockedIndex].RefineFlag = 0;
                }
                else
                {
                    // If lockedIndex == nodeIndex and we have the lock on lockedIndex, nodeIndex can't move and we're done.
                    return true;
                }
            }
        }

        /// <summary>
        /// Attempts to swap two nodes. Aborts without changing memory if the swap is contested by another thread.
        /// </summary>
        /// <remarks>Uses Node.RefineFlag as a lock-keeping mechanism. All refine flags should be cleared to 0 before a multithreaded processing stage that performs swaps.</remarks>
        /// <param name="aIndex">First node of the swap pair.</param>
        /// <param name="bIndex">Second node of the swap pair.</param>
        /// <returns>True if the nodes were swapped, false if the swap was contested.</returns>
        public unsafe bool TrySwapNodesThreadSafe(ref int aIndex, ref int bIndex)
        {
            Debug.Assert(aIndex != bIndex, "Can't swap a node with itself.");

            //We must lock:
            //a
            //b
            //a->Parent
            //b->Parent
            //a->{Children}
            //b->{Children}
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            bool success = false;
            if (TryLock(ref aIndex))
            {
                var a = nodes + aIndex;
                if (TryLock(ref bIndex))
                {
                    //Now, we know that aIndex and bIndex will not change.
                    var b = nodes + bIndex;

                    var aParentAvoidedLock = a->Parent == bIndex;
                    if (aParentAvoidedLock || TryLock(ref a->Parent))
                    {
                        var bParentAvoidedLock = b->Parent == aIndex;
                        if (bParentAvoidedLock || TryLock(ref b->Parent))
                        {

                            int aChildrenLockedCount = a->ChildCount;
                            var aChildren = &a->ChildA;
                            for (int i = 0; i < a->ChildCount; ++i)
                            {
                                if (aChildren[i] != bIndex && aChildren[i] != b->Parent && !TryLock(ref aChildren[i]))
                                {
                                    //Failed to acquire lock on all children.
                                    aChildrenLockedCount = i;
                                    break;
                                }
                            }

                            if (aChildrenLockedCount == a->ChildCount)
                            {
                                int bChildrenLockedCount = b->ChildCount;
                                var bChildren = &b->ChildA;
                                for (int i = 0; i < b->ChildCount; ++i)
                                {
                                    if (bChildren[i] != aIndex && bChildren[i] != a->Parent && !TryLock(ref bChildren[i]))
                                    {
                                        //Failed to acquire lock on all children.
                                        bChildrenLockedCount = i;
                                        break;
                                    }
                                }

                                if (bChildrenLockedCount == b->ChildCount)
                                {
                                    //ALL nodes locked successfully.
                                    SwapNodes(aIndex, bIndex);
                                    success = true;
                                }

                                for (int i = bChildrenLockedCount - 1; i >= 0; --i)
                                {
                                    if (bChildren[i] != aIndex && bChildren[i] != a->Parent) //Do not yet unlock a or its parent.
                                        nodes[bChildren[i]].RefineFlag = 0;
                                }
                            }
                            for (int i = aChildrenLockedCount - 1; i >= 0; --i)
                            {
                                if (aChildren[i] != bIndex && aChildren[i] != b->Parent) //Do not yet unlock b or its parent.
                                    nodes[aChildren[i]].RefineFlag = 0;
                            }
                            if (!bParentAvoidedLock)
                                nodes[b->Parent].RefineFlag = 0;
                        }
                        if (!aParentAvoidedLock)
                            nodes[a->Parent].RefineFlag = 0;
                    }
                    b->RefineFlag = 0;
                }
                a->RefineFlag = 0;
            }
            return success;
        }


        public unsafe void IncrementalCacheOptimizeThreadSafe(int nodeIndex)
        {
            //Multithreaded cache optimization attempts to acquire a lock on every involved node.
            //If any lock fails, it just abandons the entire attempt.
            //That's acceptable- the incremental optimization only cares about eventual success.
            var node = nodes + nodeIndex;
            bool nodeLock = 0 == Interlocked.CompareExchange(ref node->RefineFlag, 1, 0);
            if (nodeLock)
            {
                var children = &node->ChildA;

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


                //Unlock the node.
                node->RefineFlag = 0;
            }
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


        /// <summary>
        /// Begins a cache optimization at the given node and proceeds all the way to the bottom of the tree.
        /// Requires that the targeted node is already at the global optimum position.
        /// </summary>
        /// <param name="nodeIndex">Node to begin the optimization process at.</param>
        public unsafe void CacheOptimize(int nodeIndex)
        {
            var targetIndex = nodeIndex + 1;

            CacheOptimize(nodeIndex, ref targetIndex);
        }


        unsafe void CacheOptimize(int nodeIndex, ref int nextIndex)
        {
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    Debug.Assert(nextIndex >= 0 && nextIndex < nodeCount, "Swap target should be within the node set. If it's not, the initial node was probably not in global optimum position.");
                    //if (nextIndex < 0 || nextIndex >= nodeCount)
                    //    continue;
                    if (children[i] != nextIndex)
                        SwapNodes(children[i], nextIndex);
                    if (children[i] != nextIndex)
                        Console.WriteLine("That's impossible");
                    ++nextIndex;
                    CacheOptimize(children[i], ref nextIndex);
                }
            }
        }
    }

}
