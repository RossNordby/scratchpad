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

        /// <summary>
        /// Reorganizes the memory layout of this node and all of its descendants.
        /// </summary>
        /// <param name="nodeIndex">Node at which to start the optimization.</param>
        public unsafe void RecursiveIncrementalCacheOptimizeThreadSafe(int nodeIndex)
        {
            IncrementalCacheOptimizeThreadSafe(nodeIndex);
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                    RecursiveIncrementalCacheOptimizeThreadSafe(children[i]);
            }
        }

        /// <summary>
        /// Reorganizes the memory layout of this node and all of its descendants down to a maximum tree depth from this node.
        /// </summary>
        /// <param name="nodeIndex">Node at which to start the optimization.</param>
        /// <param name="maximumDepth">Maximum depth from the specified node down into the tree.</param>
        public unsafe void RecursiveIncrementalCacheOptimizeThreadSafe(int nodeIndex, int maximumDepth)
        {
            IncrementalCacheOptimizeThreadSafe(nodeIndex);
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var depthRemaining = maximumDepth - 1;
            if (depthRemaining > 0)
            {
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    if (children[i] >= 0)
                        RecursiveIncrementalCacheOptimizeThreadSafe(children[i], depthRemaining);
                }
            }
        }


        public unsafe void SwapNodesThreadSafe(int currentIndex, int targetIndex, int targetMin, int targetMax)
        {
            Debug.Assert(targetIndex >= targetMin && targetIndex < targetMax);
            if (currentIndex < targetMin || currentIndex >= targetMax)
            {
                SwapNodesThreadSafe(currentIndex, targetIndex);
            }
        }

        void Lock(int index)
        {
        }

        void Unlock(int index)
        {
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
                    Lock(children[i]);
                }
            }

            //All children are now immobile.
        }

        public unsafe void SwapNodesThreadSafe(int currentIndex, int targetIndex)
        {
            Debug.Assert(currentIndex != targetIndex, "Can't swap a node with itself.");
            var current = nodes + currentIndex;
            var target = nodes + targetIndex;
            //We must lock:
            //a
            //b
            //a->Parent
            //b->Parent
            //a->{Children}
            //b->{Children}
            //But watch out, a may be the parent of b or vice versa. Given that these locks are non-reentrant, you must avoid attempting to lock them twice.

            //Note that we don't have to worry about ordering because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note that there is a small chance that the 'current' pointer does not refer to the same node here as it did when the function was called, because
            //another swap may have moved things around.

            //Maybe that's okay?
            if (0 == Interlocked.CompareExchange(ref current->RefineFlag, 1, 0))
            {
                if (0 == Interlocked.CompareExchange(ref target->RefineFlag, 1, 0))
                {
                    var currentParent = nodes + current->Parent;
                    if (0 == Interlocked.CompareExchange(ref currentParent->RefineFlag, 1, 0))
                    {
                        var targetParent = nodes + current->Parent;
                        if (0 == Interlocked.CompareExchange(ref targetParent->RefineFlag, 1, 0))
                        {
                            int currentChildrenLockedCount = current->ChildCount;
                            var currentChildren = &current->ChildA;
                            for (int i = 0; i < current->ChildCount; ++i)
                            {
                                if (0 != Interlocked.CompareExchange(ref nodes[currentChildren[i]].RefineFlag, 1, 0))
                                {
                                    //Failed to acquire lock on all children.
                                    currentChildrenLockedCount = i;
                                    break;
                                }
                            }

                            if (currentChildrenLockedCount == current->ChildCount)
                            {
                                int targetChildrenLockedCount = target->ChildCount;
                                var targetChildren = &target->ChildA;
                                for (int i = 0; i < target->ChildCount; ++i)
                                {
                                    if (0 != Interlocked.CompareExchange(ref nodes[targetChildren[i]].RefineFlag, 1, 0))
                                    {
                                        //Failed to acquire lock on all children.
                                        targetChildrenLockedCount = i;
                                        break;
                                    }
                                }

                                if (targetChildrenLockedCount == target->ChildCount)
                                {
                                    //ALL nodes locked successfully.
                                    SwapNodes(currentIndex, targetIndex);
                                }

                                for (int i = targetChildrenLockedCount - 1; i >= 0; --i)
                                {
                                    nodes[targetChildren[i]].RefineFlag = 0;
                                }
                            }
                            for (int i = currentChildrenLockedCount - 1; i >= 0; --i)
                            {
                                nodes[currentChildren[i]].RefineFlag = 0;
                            }

                            targetParent->RefineFlag = 0;
                        }
                        currentParent->RefineFlag = 0;
                    }
                    target->RefineFlag = 0;
                }
                current->RefineFlag = 0;
            }
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
            ++nextIndex;

            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    Debug.Assert(nextIndex >= 0 && nextIndex < nodeCount, "Swap target should be within the node set. If it's not, the initial node was probably not in global optimum position.");

                    if (children[i] != nextIndex)
                        SwapNodes(nodeIndex, nextIndex);
                    ++nextIndex;
                    CacheOptimize(children[i], ref nextIndex);
                }
            }
        }
    }

}
