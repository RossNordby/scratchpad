using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace SolverPrototype.Collidables
{

    /// <summary>
    /// Collidable representing a body.
    /// </summary>
    /// <remarks>Body collidables are distinguished from non-body collidables by the need to gather pose and velocity from a body to update the bounding box.
    /// Isolated collidables can exist and use the same shapes as body collidables, but since their bounding boxes don't need to be updated every frame, they aren't included 
    /// in the body collidable batches (whose main purpose is to update bounding boxes efficiently).</remarks>
    public struct BodyCollidable
    {
        /// <summary>
        /// Index of the shape used by the body. The type is implicit; the body collidable batch holding this body collidable knows which type to use.
        /// </summary>
        public int ShapeIndex;
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters.
        /// </summary>
        public int BroadPhaseIndex;
        /// <summary>
        /// Index of the body that owns this collidable.
        /// </summary>
        /// <remarks>This is needed for the narrow phase to know what bodies are involved in newly generated contact constraints. It will be updated whenever a body moves in memory.</remarks> 
        public int BodyIndex;

        //Note that the AABB update phase needs only the shape index and broad phase index, while the narrow phase only needs the shape index and body index.
        //It's possible to split this struct into two parallel arrays with redundant body index storage, but the value is pretty questionable.
        //The AABB update is followed directly by the narrow phase. Given that the AABB update deals with a pretty limited set of data (pose integrator + AABB update 
        //will pull about 4MB for 32768 bodies), it's very likely that all of it will still be in L3 cache. That isn't perfect, but the potential benefit is really small
        //and the complexity isn't.
    }



    public abstract class BodyCollidableBatch
    {
        protected QuickList<BodyCollidable, Buffer<BodyCollidable>> collidables;

        public ref BodyCollidable this[int collidableIndex] { get { return ref collidables[collidableIndex]; } }

        protected BodyCollidableBatch(BufferPool pool, int initialCollidableCount)
        {
            QuickList<BodyCollidable, Buffer<BodyCollidable>>.Create(pool.SpecializeFor<BodyCollidable>(), initialCollidableCount, out collidables);

        }

        public int Allocate(BufferPool pool)
        {
            //Note that we do not actually fill the slot here. That's because of a dependency issue- we have to add the collidable's index to the broad phase to get a target index,
            //but we need a target index to build the data for the body collidable...
            int index = collidables.Count;
            var newCount = collidables.Count + 1;
            collidables.EnsureCapacity(newCount, pool.SpecializeFor<BodyCollidable>());
            collidables.Count = newCount;
            //The bodies set will store an index that points back here, so we need to report it.
            return index;
        }

        /// <summary>
        /// Removes a body collidable from the batch and which body owns the collidable that moved to fill its slot, if any. 
        /// </summary>
        /// <param name="index">Index to remove from the batch.</param>
        /// <param name="bodyHandleOfMovedCollidable">If the index was not the last slot in the collidables (and so this function returned true),
        /// this is the handle of the body owning the body collidable that was moved to fill its slot.
        /// If no collidable was moved (and so this function returned false), the value is undefined.</param>
        /// <returns>True if a body collidable was moved and reported, false otherwise.</returns>
        public bool RemoveAt(int index, out int bodyHandleOfMovedCollidable)
        {
            collidables.FastRemoveAt(index);
            if (index < collidables.Count)
            {
                bodyHandleOfMovedCollidable = collidables[index].BodyIndex;
                return true;
            }
            bodyHandleOfMovedCollidable = -1;
            return false;
        }


        public abstract void FlushUpdates(Bodies bodies, ref QuickList<int, Buffer<int>> collidablesToUpdate);

        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    public class BodyCollidableBatch<TShape> : BodyCollidableBatch where TShape : struct, IShape
    {
        ShapeBatch<TShape> shapes;

        public BodyCollidableBatch(ShapeBatch shapeBatch, BufferPool pool, int initialCollidableCount)
            : base(pool, initialCollidableCount)
        {
            shapes = (ShapeBatch<TShape>)shapeBatch;
        }

        public override void FlushUpdates(Bodies bodies, ref QuickList<int, Buffer<int>> collidablesToUpdate)
        {
        }
    }

    public interface IBroadPhase
    {
        int AllocateForCollidable(CollidableReference collidableReference);
    }

    public struct BroadPhaseWrapper<TBroadPhase> : IBroadPhase where TBroadPhase : IBroadPhase
    {
        public int AllocateForCollidable(CollidableReference collidableReference)
        {
            throw new NotImplementedException();
        }
    }


    public class BodyCollidables//<TBroadPhase> where TBroadPhase : struct, IBroadPhase
    {
        BodyCollidableBatch[] batches;
        public Shapes Shapes;
        IBroadPhase broadPhase;

        public BodyCollidableBatch this[int typeIndex] => batches[typeIndex];

        public int InitialCapacityPerCollidableBatch { get; set; }

        public BodyCollidables(Shapes shapes, int initialCapacityPerCollidableBatch)
        {
            //This set pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            batches = new BodyCollidableBatch[16];
        }

        public void Add(int bodyIndex, TypedIndex shapeIndex, BufferPool pool)
        {
            var typeIndex = shapeIndex.Type;
            var index = shapeIndex.Index;
            Debug.Assert(typeIndex >= 0 && typeIndex < Shapes.RegisteredTypeCount);
            if (batches.Length < Shapes.RegisteredTypeCount)
            {
                Array.Resize(ref batches, Shapes.RegisteredTypeCount);
            }
            ref var batch = ref batches[typeIndex];
            if (batch == null)
            {
                batch = Shapes[typeIndex].CreateBodyCollidableBatchForType(InitialCapacityPerCollidableBatch);
            }
            var collidableIndex = batch.Allocate(pool);
            ref var collidable = ref batch[collidableIndex];
            collidable.BodyIndex = bodyIndex;
            collidable.ShapeIndex = index;
            collidable.BroadPhaseIndex = broadPhase.AllocateForCollidable(new CollidableReference(true, typeIndex, collidableIndex));
        }

    }
}
