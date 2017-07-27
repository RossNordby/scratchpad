using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    public unsafe class BroadPhase : IDisposable
    {
        //For now, we only have an active tree. Later on, we'll need two trees- the second one represents all objects that do not move.
        //That one would cover static or inactive collidables.
        internal Buffer<CollidableReference> activeLeaves;
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
            if (leafIndex >= activeLeaves.Length)
            {
                ActiveTree.Pool.SpecializeFor<CollidableReference>().Resize(ref activeLeaves, ActiveTree.LeafCount + 1, activeLeaves.Length);
            }
            activeLeaves[leafIndex] = collidable;
            return leafIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveAt(int index, out CollidableReference movedLeaf)
        {
            Debug.Assert(index >= 0);
            var movedLeafIndex = ActiveTree.RemoveAt(index);
            if (movedLeafIndex >= 0)
            {
                movedLeaf = activeLeaves[movedLeafIndex];
                activeLeaves[index] = movedLeaf;
                return true;
            }
            movedLeaf = new CollidableReference();
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBoundsPointers(int broadPhaseIndex, out float* minPointer, out float* maxPointer)
        {
            var leaf = ActiveTree.Leaves[broadPhaseIndex];
            var nodeChild = (&ActiveTree.nodes[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min.X;
            maxPointer = &nodeChild->Max.X;
        }

        internal void UpdateForCollidableMemoryMove(int broadPhaseIndex, int newCollidableIndex)
        {
            activeLeaves[broadPhaseIndex].CollidableIndex = newCollidableIndex;
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
                        threadHandlers[i] = new OverlapHandler(activeLeaves, narrowPhase);
                    }
                }
                refineContext.RefitAndRefine(ActiveTree, threadDispatcher, frameIndex);
                selfTestContext.SelfTest(ActiveTree, threadHandlers, threadDispatcher);
            }
            else
            {
                ActiveTree.RefitAndRefine(frameIndex);
                var handler = new OverlapHandler(activeLeaves, narrowPhase);
                ActiveTree.GetSelfOverlaps(ref handler);
            }

        }

        struct OverlapHandler : IOverlapHandler
        {
            //TODO: Once we have a second tree, we'll need to have a second handler type which can pull from two different leaf sources.
            //No reason to try to make one type do both- that would just result in a bunch of last second branches that could be avoided by using the proper
            //handler upon dispatch. 
            public NarrowPhase NarrowPhase;
            public Buffer<CollidableReference> Leaves;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public OverlapHandler(Buffer<CollidableReference> leaves, NarrowPhase narrowPhase)
            {
                Leaves = leaves;
                NarrowPhase = narrowPhase;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                NarrowPhase.HandleOverlap(Leaves[indexA], Leaves[indexB]);
            }
        }

        //TODO: EnsureCapacity and so on. Need them for the broadphase's own leaves sets. We COULD expose the underlying trees and let their sizes be managed separately,
        //or we can handle them at the level of the broadphase too. 

        public void Dispose()
        {
            ActiveTree.Dispose();
        }
    }
}
