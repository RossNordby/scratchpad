using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

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
        /// Index of the body that owns this collidable.
        /// </summary>
        /// <remarks>This is needed for the narrow phase to know what bodies are involved in newly generated contact constraints. It will be updated whenever a body moves in memory.</remarks> 
        public int BodyIndex;
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters.
        /// </summary>
        public int BroadPhaseIndex;
        /// <summary>
        /// Size of the margin around the surface of the shape in which contacts can be generated. These contacts will have negative depth and only contribute if the frame's velocity
        /// would push the shape into overlap. This should be positive to avoid jittering. It can also be used as a form of continuous collision detection, but excessively high values
        /// combined with fast motion may result in visible 'ghost collision' artifacts. 
        /// <para>For continuous collision detection with less chance of ghost collisions, use the dedicated continuous collision detection modes.</para>
        /// </summary>
        public float SpeculativeMargin;

        /// <summary>
        /// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
        /// </summary>
        public ContinuousDetectionSettings Continuity;
        //These CCD settings are bundled together away from the rest of the collidable data for a few reasons:
        //1) They do a little packing to avoid pointless memory overhead,
        //2) It's possible that we'll want to split them out later if data access patterns suggest that it's a good idea,
        //3) Don't really want to pollute this structure's members with CCD-conditional tuning variables.
    }


    public interface IBundleBounder
    {
        void Initialize(ShapeBatch batch);
        //Note that this requires the bundle bounder to have access to the type-specific shape data; a typed shape collection is not passed in.
        //This helps limit type exposure and generics explosion.
        void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max);
    }

    public interface IBoundsScatterer
    {
        void Scatter(ref Vector3Wide min, ref Vector3Wide max, int index);
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
        /// <param name="bodyIndexOfMovedCollidable">If the index was not the last slot in the collidables (and so this function returned true),
        /// this is the handle of the body owning the body collidable that was moved to fill its slot.
        /// If no collidable was moved (and so this function returned false), the value is undefined.</param>
        /// <returns>True if a body collidable was moved and reported, false otherwise.</returns>
        public bool RemoveAt(int index, out int bodyIndexOfMovedCollidable)
        {
            collidables.FastRemoveAt(index);
            if (index < collidables.Count)
            {
                bodyIndexOfMovedCollidable = collidables[index].BodyIndex;
                return true;
            }
            bodyIndexOfMovedCollidable = -1;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherCollidableData(ref QuickList<int, Buffer<int>> bundledCollidables, int startIndex,
            out Vector<int> shapeIndices, out Vector<int> bodyIndices, out Vector<float> maximumExpansion)
        {
            ref var firstCollidable = ref bundledCollidables[startIndex];
            Debug.Assert(bundledCollidables.Count > startIndex + Vector<float>.Count);

            ref var firstShapeIndex = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var firstBodyIndex = ref Unsafe.As<Vector<int>, int>(ref bodyIndices);
            ref var firstExpansion = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                ref var collidable = ref collidables[Unsafe.Add(ref firstCollidable, i)];
                Unsafe.Add(ref firstShapeIndex, i) = collidable.ShapeIndex;
                Unsafe.Add(ref firstBodyIndex, i) = collidable.BodyIndex;
                Unsafe.Add(ref firstExpansion, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }

        /// <summary>
        /// Defines a type that acts as a source of data needed for bounding box calculations.
        /// </summary>
        /// <remarks>
        /// Collidables may be pulled from objects directly in the world or from compound children. Compound children have to pull and compute information from the parent compound,
        /// and the result of the calculation has to be pushed back to the compound parent for further processing. In contrast, body collidables that live natively in the space simply
        /// gather data directly from the bodies set and scatter bounds directly into the broad phase.
        /// </remarks>
        public interface ICollidableBundleSource
        {
            /// <summary>
            /// Gets the number of collidables in this set of bundles.
            /// </summary>
            int Count { get; }
            /// <summary>
            /// Gathers collidable data required to calculate the bounding boxes for a bundle.
            /// </summary>
            /// <param name="collidablesStartIndex">Start index of the bundle in the collidables set to gather bounding box relevant data for.</param>
            void GatherCollidableBundle(int collidablesStartIndex, out Vector<int> shapeIndices, out Vector<float> maximumExpansion,
                out BodyPoses poses, out BodyVelocities velocities);
            /// <summary>
            /// Scatters the calculated bounds into the target memory locations.
            /// </summary>
            void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int collidablesStartIndex);
        }
        
        public static void UpdatePrimitiveBoundingBoxes<TBundleBounder, TBundleSource>(
            ref TBundleSource bundleSource, ref TBundleBounder bundleBounder, Bodies bodies, float dt)
            where TBundleBounder : IBundleBounder where TBundleSource : ICollidableBundleSource
        {
            for (int i = 0; i < bundleSource.Count; i += Vector<float>.Count)
            {
                bundleSource.GatherCollidableBundle(i, out var shapeIndices, out var maximumExpansion, out var poses, out var velocities);

                //The bundle bounder is responsible for gathering shapes from type specific sources. Since it has type knowledge, it is able to both
                //gather the necessary shape information and call the appropriate bounding box calculator.
                bundleBounder.GetBounds(ref shapeIndices, ref poses, out var maximumRadius, out var min, out var max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumExpansion);

                //The bounding boxes are now fully expanded. Scatter them to the target location. For raw primitives in the broad phase, this means sticking them
                //in the broad phase's acceleration structure. For compound children, we stick them in the waiting compound temporary slots.
                //To avoid branching or virtual indirections, we once again abuse generics to supply the scattering logic.
                bundleSource.ScatterBounds(ref min, ref max, i);

            }
        }

        public abstract void FlushUpdates(Bodies bodies, float dt, ref QuickList<int, Buffer<int>> collidablesToUpdate);

        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    struct SphereBundleBounder : IBundleBounder
    {
        ShapeBatch<Sphere> shapes;

        //Using an initialize function gets around generic issues with constructors.
        public void Initialize(ShapeBatch shapes)
        {
            this.shapes = (ShapeBatch<Sphere>)shapes;
        }

        public void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max)
        {
        }
    }


    public class PrimitiveCollidableBatch<TBundleBounder> : BodyCollidableBatch where TBundleBounder : struct, IBundleBounder
    {
        TBundleBounder bundleBounder;

        public PrimitiveCollidableBatch(ShapeBatch shapeBatch, BufferPool pool, int initialCollidableCount)
            : base(pool, initialCollidableCount)
        {
            bundleBounder = default(TBundleBounder);
            bundleBounder.Initialize(shapeBatch);
        }

        public sealed override void FlushUpdates(Bodies bodies, float dt, ref QuickList<int, Buffer<int>> collidablesToUpdate)
        {
            //UpdatePrimitiveBoundingBoxes(this, ref bundleBounder, bodies, dt, ref collidablesToUpdate);
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

        public void Add(Bodies bodies, int bodyIndex, ref CollidableDescription collidableDescription, BufferPool pool)
        {
            var typeIndex = collidableDescription.ShapeIndex.Type;
            var index = collidableDescription.ShapeIndex.Index;
            Debug.Assert(typeIndex >= 0 && typeIndex < Shapes.RegisteredTypeSpan);
            if (batches.Length < Shapes.RegisteredTypeSpan)
            {
                Array.Resize(ref batches, Shapes.RegisteredTypeSpan);
            }
            ref var batch = ref batches[typeIndex];
            if (batch == null)
            {
                batch = Shapes[typeIndex].CreateBodyCollidableBatchForType(InitialCapacityPerCollidableBatch);
            }
            var collidableIndex = batch.Allocate(pool);
            //Note that this is responsible for setting the body's collidable reference back to the collidable. A little less gross in practice than forcing the simulation to handle
            //that mapping, and it's consistent with the way the Remove function works.
            bodies.Collidables[bodyIndex] = new TypedIndex(typeIndex, collidableIndex);
            ref var collidable = ref batch[collidableIndex];
            collidable.BodyIndex = bodyIndex;
            collidable.ShapeIndex = index;
            collidable.BroadPhaseIndex = broadPhase.AllocateForCollidable(new CollidableReference(true, typeIndex, collidableIndex));
            collidable.SpeculativeMargin = collidableDescription.SpeculativeMargin;
            collidable.Continuity = collidableDescription.Continuity;
        }

        public void Remove(Bodies bodies, TypedIndex collidableIndex)
        {
            if (batches[collidableIndex.Type].RemoveAt(collidableIndex.Index, out var bodyIndexOfMovedCollidable))
            {
                //When another collidable is moved into the removed slot, we must notify the body of the moved collidable of the new position.
                bodies.Collidables[bodyIndexOfMovedCollidable] = collidableIndex;
            }
        }
    }
}
