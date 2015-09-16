using BEPUutilities.DataStructures;
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

        unsafe bool TrySwapNodeWithTargetThreadSafe(int swapperIndex, int swapperParentIndex, int swapTargetIndex)
        {
            Debug.Assert(nodes[swapperIndex].RefineFlag == 1, "The swapper should be locked.");
            Debug.Assert(nodes[swapperParentIndex].RefineFlag == 1, "The swapper parent should be locked.");
            Debug.Assert(swapTargetIndex != swapperIndex, "If the swapper is already at the swap target, this should not be called."); //safe to compare since if equal, it's locked.
            //We must make sure that the node, its parent, and its children are locked.
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //The possible cases are as follows:
            //0) The swap target is the swapper's grandparent. Don't lock the swap target's child that == swapper's parent.
            //1) The swap target is the swapper's parent. Don't lock the swap target, AND don't lock the swap target's child == swapper.
            //2) The swap target is the swapper's position (do nothing, you're done).
            //3) The swap target is one of the swapper's children. Don't lock the swap target, AND don't lock swap target's parent.
            //4) The swap target is one of the swapper's grandchildren. Don't lock the swap target's parent.
            //5) The swap target is unrelated to the swapper.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            //Don't lock swapTarget if:
            //1) swapTargetIndex == swapperParentIndex, because swapperParentIndex is already locked
            //2) nodes[swapTargetIndex].Parent == swapperIndex, because swapper's children are already locked 

            //Note that the above comparison between a potentially unlocked nodes[swapTargetIndex].Parent and swapperIndex is safe because
            //if it evaluates true, then it was actually locked. In the event that it evaluates to false, they aren't the same node- which random node it might be doesn't matter.
            //Similar logic applies to the similar lock elisions below.

            bool success = false;
            var needSwapTargetLock = swapTargetIndex != swapperParentIndex && nodes[swapTargetIndex].Parent != swapperIndex;
            if (!needSwapTargetLock || TryLock(ref swapTargetIndex))
            {
                var swapTarget = nodes + swapTargetIndex;

                //Don't lock swapTarget->Parent if:
                //1) swapTarget->Parent == swapperIndex, because swapper is already locked.
                //2) nodes[swapTarget->Parent].Parent == swapperIndex, because swapper's children are already locked.

                var needSwapTargetParentLock = swapTarget->Parent != swapperIndex && nodes[swapTarget->Parent].Parent != swapperIndex;
                if (!needSwapTargetParentLock || TryLock(ref swapTarget->Parent))
                {

                    int childrenLockedCount = swapTarget->ChildCount;
                    var children = &swapTarget->ChildA;
                    for (int i = 0; i < swapTarget->ChildCount; ++i)
                    {
                        //Don't lock children[i] if:
                        //1) children[i] == swapperIndex, because the swapper is already locked 
                        //2) children[i] == swapperParentIndex, because the swapperParent is already locked
                        if (children[i] >= 0 && children[i] != swapperIndex && children[i] != swapperParentIndex && !TryLock(ref children[i]))
                        {
                            //Failed to acquire lock on all children.
                            childrenLockedCount = i;
                            break;
                        }
                    }

                    if (childrenLockedCount == swapTarget->ChildCount)
                    {
                        //Nodes locked successfully.
                        SwapNodes(swapperIndex, swapTargetIndex);
                        success = true;

                        //Unlock children of the original swap target, *which now lives in the swapperIndex*.
                        children = &nodes[swapperIndex].ChildA;
                        for (int i = childrenLockedCount - 1; i >= 0; --i)
                        {
                            //Again, note use of swapTargetIndex instead of swapperIndex.
                            if (children[i] >= 0 && children[i] != swapTargetIndex && children[i] != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                                nodes[children[i]].RefineFlag = 0;
                        }
                    }
                    else
                    {
                        //No swap occurred. Can still use the swapTarget->ChildA pointer.
                        for (int i = childrenLockedCount - 1; i >= 0; --i)
                        {
                            if (children[i] >= 0 && children[i] != swapperIndex && children[i] != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                                nodes[children[i]].RefineFlag = 0;
                        }
                    }


                    if (needSwapTargetParentLock)
                    {
                        if (success)
                        {
                            //Note that swapTarget pointer is no longer used, since the node was swapped.
                            //The old swap target is now in the swapper index slot!
                            nodes[nodes[swapperIndex].Parent].RefineFlag = 0;
                        }
                        else
                        {
                            //No swap occurred, 
                            nodes[swapTarget->Parent].RefineFlag = 0;
                        }
                    }
                }

                if (needSwapTargetLock)
                {
                    if (success)
                    {
                        //Once again, the original swapTarget now lives in swapperIndex.
                        nodes[swapperIndex].RefineFlag = 0;
                    }
                    else
                    {
                        swapTarget->RefineFlag = 0;
                    }
                }
            }
            return success;
        }

        public unsafe bool TryLockSwapTargetThreadSafe(ref int swapTargetIndex, int swapperIndex, int swapperParentIndex)
        {
            Debug.Assert(nodes[swapperIndex].RefineFlag == 1, "The swapper should be locked.");
            Debug.Assert(nodes[swapperParentIndex].RefineFlag == 1, "The swapper parent should be locked.");
            Debug.Assert(swapTargetIndex != swapperIndex, "If the swapper is already at the swap target, this should not be called."); //safe to compare since if equal, it's locked.
            //We must make sure that the node, its parent, and its children are locked.
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //The possible cases are as follows:
            //0) The swap target is the swapper's grandparent. Don't lock the swap target's child that == swapper's parent.
            //1) The swap target is the swapper's parent. Don't lock the swap target, AND don't lock the swap target's child == swapper.
            //2) The swap target is the swapper's position (do nothing, you're done).
            //3) The swap target is one of the swapper's children. Don't lock the swap target, AND don't lock swap target's parent.
            //4) The swap target is one of the swapper's grandchildren. Don't lock the swap target's parent.
            //5) The swap target is unrelated to the swapper.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            //Don't lock swapTarget if:
            //1) swapTargetIndex == swapperParentIndex, because swapperParentIndex is already locked
            //2) nodes[swapTargetIndex].Parent == swapperIndex, because swapper's children are already locked 

            //Note that the above comparison between a potentially unlocked nodes[swapTargetIndex].Parent and swapperIndex is safe because
            //if it evaluates true, then it was actually locked. In the event that it evaluates to false, they aren't the same node- which random node it might be doesn't matter.
            //Similar logic applies to the similar lock elisions below.

            bool success = false;
            var needSwapTargetLock = swapTargetIndex != swapperParentIndex && nodes[swapTargetIndex].Parent != swapperIndex;
            if (!needSwapTargetLock || TryLock(ref swapTargetIndex))
            {
                var swapTarget = nodes + swapTargetIndex;

                //Don't lock swapTarget->Parent if:
                //1) swapTarget->Parent == swapperIndex, because swapper is already locked.
                //2) nodes[swapTarget->Parent].Parent == swapperIndex, because swapper's children are already locked.

                var needSwapTargetParentLock = swapTarget->Parent != swapperIndex && nodes[swapTarget->Parent].Parent != swapperIndex;
                if (!needSwapTargetParentLock || TryLock(ref swapTarget->Parent))
                {

                    int childrenLockedCount = swapTarget->ChildCount;
                    var children = &swapTarget->ChildA;
                    for (int i = 0; i < swapTarget->ChildCount; ++i)
                    {
                        //Don't lock children[i] if:
                        //1) children[i] == swapperIndex, because the swapper is already locked 
                        //2) children[i] == swapperParentIndex, because the swapperParent is already locked
                        if (children[i] != swapperIndex && children[i] != swapperParentIndex && !TryLock(ref children[i]))
                        {
                            //Failed to acquire lock on all children.
                            childrenLockedCount = i;
                            break;
                        }
                    }

                    if (childrenLockedCount == swapTarget->ChildCount)
                    {
                        //Nodes locked successfully.
                        success = true;
                    }
                    //TODO: should not unlock here because this is a LOCK function!
                    for (int i = childrenLockedCount - 1; i >= 0; --i)
                    {
                        if (children[i] != swapperIndex && children[i] != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                            nodes[children[i]].RefineFlag = 0;
                    }

                    if (needSwapTargetParentLock)
                        nodes[swapTarget->Parent].RefineFlag = 0;
                }
                if (needSwapTargetLock)
                    swapTarget->RefineFlag = 0;
            }
            return success;
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


        /// <summary>
        /// Moves the children if the specified node into the correct relative position in memory.
        /// Takes care to avoid contested moves in multithreaded contexts. May not successfully
        /// complete all desired moves if contested.
        /// </summary>
        /// <param name="nodeIndex">Node whose children should be optimized.</param>
        /// <returns>True if no other threads contested the optimization, otherwise false.
        /// Will return true even if not all nodes are optimized if the reason was a target index outside of the node list bounds.</returns>
        public unsafe bool IncrementalCacheOptimizeThreadSafe(int nodeIndex)
        {
            //Multithreaded cache optimization attempts to acquire a lock on every involved node.
            //If any lock fails, it just abandons the entire attempt.
            //That's acceptable- the incremental optimization only cares about eventual success.

            //TODO: could attempt to compare child pointers without locks. Unsafe, but acceptable as an optimization prepass. Would avoid some interlocks.
            //It's a PERFORMANCE question, though, so make sure you measure it.
            var node = nodes + nodeIndex;
            bool success = true;
            if (0 == Interlocked.CompareExchange(ref node->RefineFlag, 1, 0))
            {
                var children = &node->ChildA;

                var leafCounts = &node->LeafCountA;
                var targetIndex = nodeIndex + 1;

                Debug.Assert(node->RefineFlag == 1);


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
                        if (TryLock(ref children[i]))
                        {
                            var originalChildIndex = children[i];
                            //While we checked if children[i] != targetIndex earlier as an early-out, it must be done post-lock for correctness because children[i] could have changed.
                            //Attempting a swap between an index and itself is invalid.
                            if (originalChildIndex != targetIndex)
                            {
                                //Now lock all of this child's children.
                                var child = nodes + originalChildIndex;
                                var grandchildren = &child->ChildA;
                                int lockedChildrenCount = child->ChildCount;
                                for (int grandchildIndex = 0; grandchildIndex < child->ChildCount; ++grandchildIndex)
                                {
                                    //It is very possible that this grandchild pointer could swap between now and the compare exchange read. 
                                    //However, a child pointer will not turn from an internal node (positive) to a leaf node (negative), and that's all that matters.
                                    if (grandchildren[grandchildIndex] >= 0 && !TryLock(ref grandchildren[grandchildIndex]))
                                    {
                                        lockedChildrenCount = grandchildIndex;
                                        break;
                                    }
                                }
                                if (lockedChildrenCount == child->ChildCount)
                                {
                                    Debug.Assert(node->RefineFlag == 1);
                                    var preaabb = node->A;
                                    if (!TrySwapNodeWithTargetThreadSafe(originalChildIndex, nodeIndex, targetIndex))
                                    {
                                        //Failed target lock.
                                        success = false;
                                    }
                                    var postaabbb = node->A;
                                    if (preaabb.Min != postaabbb.Min || preaabb.Max != postaabbb.Max)
                                    {
                                        Console.WriteLine("asdF");
                                    }
                                    Debug.Assert(node->RefineFlag == 1);

                                }
                                else
                                {
                                    //Failed grandchild lock.
                                    success = false;
                                }

                                //Unlock all grandchildren.
                                //Note that we can't use the old grandchildren pointer. If the swap went through, it's pointing to the *target's* children.
                                //So update the pointer.
                                grandchildren = &nodes[children[i]].ChildA;
                                for (int grandchildIndex = lockedChildrenCount - 1; grandchildIndex >= 0; --grandchildIndex)
                                {
                                    if (grandchildren[grandchildIndex] >= 0)
                                        nodes[grandchildren[grandchildIndex]].RefineFlag = 0;
                                }

                            }
                            //Unlock. children[i] is either the targetIndex, if a swap went through, or it's the original child index if it didn't.
                            //Those are the proper targets.
                            nodes[children[i]].RefineFlag = 0;
                        }
                        else
                        {
                            //Failed child lock.
                            success = false;
                        }
                        //Leafcounts cannot change due to other threads.
                        targetIndex += leafCounts[i] - 1; //Only works on 2-ary trees.
                    }
                }
                //Unlock the parent.
                node->RefineFlag = 0;
            }
            else
            {
                //Failed parent lock.
                success = false;
            }
            {
                //TEMP DEBUG. broken in multithread.
                Debug.Assert(node->RefineFlag == 0);
                var children = &node->ChildA;

                for (int i = 0; i < node->ChildCount; ++i)
                {
                    if (children[i] >= 0)
                    {
                        Debug.Assert(nodes[children[i]].RefineFlag == 0);
                        var child = nodes + children[i];
                        var grandchildren = &child->ChildA;
                        for (int j = 0; j < child->ChildCount; ++j)
                        {
                            if (grandchildren[j] >= 0)
                            {
                                Debug.Assert(nodes[grandchildren[j]].RefineFlag == 0);
                            }
                        }
                    }
                }
            }
            return success;
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
