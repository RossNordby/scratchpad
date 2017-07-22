using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{
    public unsafe class BroadPhase : IDisposable
    {
        //For now, we only have an active tree. Later on, we'll need two trees- the second one represents all objects that do not move.
        //That one would cover static or inactive collidables.
        Buffer<CollidableReference> activeLeaves;
        public Tree ActiveTree;
        Tree.MultithreadedSelfTest<OverlapHandler> selfTestContext;
        Tree.RefitAndRefineMultithreadedContext refineContext;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096)
        {
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            pool.SpecializeFor<CollidableReference>().Take(initialActiveLeafCapacity, out activeLeaves);

            selfTestContext = new Tree.MultithreadedSelfTest<OverlapHandler>();
            refineContext = new Tree.RefitAndRefineMultithreadedContext();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Add(CollidableReference collidable, ref BoundingBox bounds)
        {
            //TODO: when there is a static tree, we'll need to distinguish between the two trees in the index that we return to the user.
            var leafIndex = ActiveTree.Add(ref bounds);
            activeLeaves[leafIndex] = collidable;
            return leafIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveAt(int index)
        {
            Debug.Assert(index >= 0);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBoundsPointers(int broadPhaseIndex, out float* minPointer, out float* maxPointer)
        {
            var leaf = ActiveTree.Leaves[broadPhaseIndex];
            var nodeChild = (&ActiveTree.nodes[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min.X;
            maxPointer = &nodeChild->Max.X;
        }
        //Note that some systems (like the demos renderer bounding box line extractor) iterate over the leaves. However, they're not contiguously stored.
        //So, the user needs a way to know if the leaf index exists. Hence, a 'try' variant. If there happen to be more use cases for checking existence, a
        //dedicated 'leafexists' method would probably be a better idea.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetBoundsPointers(int broadPhaseIndex, out float* minPointer, out float* maxPointer)
        {
            GetBoundsPointers(broadPhaseIndex, out minPointer, out maxPointer);
            return true;
        }

        int frameIndex;
        OverlapHandler[] threadHandlers;
        public void Update(IThreadDispatcher threadDispatcher, NarrowPhase narrowPhase)
        {
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            if (threadDispatcher != null)
            {
                if (threadHandlers == null || threadHandlers.Length < threadDispatcher.ThreadCount)
                {
                    //This initialization/resize should occur extremely rarely.
                    threadHandlers = new OverlapHandler[threadDispatcher.ThreadCount];
                    for (int i = 0; i < threadHandlers.Length; ++i)
                    {
                        threadHandlers[i] = new OverlapHandler { narrowPhase = narrowPhase };
                    }
                }
                refineContext.RefitAndRefine(ActiveTree, threadDispatcher, frameIndex);
                selfTestContext.SelfTest(ActiveTree, threadHandlers, threadDispatcher);
            }
            else
            {
                ActiveTree.RefitAndRefine(frameIndex);
                var handler = new OverlapHandler { narrowPhase = narrowPhase };
                ActiveTree.GetSelfOverlaps(ref handler);
            }

        }

        struct OverlapHandler : IOverlapHandler
        {
            public NarrowPhase narrowPhase;
            public void Handle(int indexA, int indexB)
            {
                throw new NotImplementedException();
            }
        }

        //TODO: EnsureCapacity and so on. Seems like we should just expose the underlying tree structures- otherwise, we'll have two passthrough functions for every tree version.
        //Plus, it's potentially useful to only raycast active objects or something- no reason to hide the trees.

        public void Dispose()
        {
            ActiveTree.Dispose();
        }
    }
}
