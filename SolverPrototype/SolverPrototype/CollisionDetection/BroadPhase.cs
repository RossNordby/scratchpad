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
        Tree.RefitAndRefineMultithreadedContext refineContext;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096)
        {
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            pool.SpecializeFor<CollidableReference>().Take(initialActiveLeafCapacity, out activeLeaves);
            
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
        public void Update(IThreadDispatcher threadDispatcher = null)
        {
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            if (threadDispatcher != null)
            {
                refineContext.RefitAndRefine(ActiveTree, threadDispatcher, frameIndex);
            }
            else
            {
                ActiveTree.RefitAndRefine(frameIndex);
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
