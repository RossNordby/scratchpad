﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;
using SolverPrototype.CollisionDetection;
using BEPUutilities2;

namespace SolverPrototype.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape
    {

        //Note that the shape gathering required for get bounds is also useful for narrow phase calculations.
        //However, exposing it in a type-safe way isn't trivial. So instead, we just choose at the API level to bundle the gather and AABB calculation together.
        //Analogously to the bounds calculation, narrow phase pairs will have the type information to directly call the underlying type's gather function.
        void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
           out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
           where TShape : struct, IShape;
        //One-off bounds calculations are useful sometimes, even within the engine. Adding individual bodies to the simulation, for example.
        //(Of course, if you wanted to add a lot of bodies, you'd want to batch everything up and use either cached values or the above bundle bounds calculator, but 
        //for most use cases, body-adding isn't the bottleneck.)
        //Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
        //I don't predict ever needing it, but such an implementation could be added...
        void GetBounds(ref BEPUutilities2.Quaternion orientation, out Vector3 min, out Vector3 max);

        //These functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
        //By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
        //deals with positions directly.

        int TypeId { get; }
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
        void GatherCollidableBundle(int collidablesStartIndex, int count, out Vector<int> shapeIndices, out Vector<float> maximumExpansion,
            out RigidPoses poses, out BodyVelocities velocities);
        /// <summary>
        /// Scatters the calculated bounds into the target memory locations.
        /// </summary>
        void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int startIndex, int count);
    }

    public struct BodyBundleSource : ICollidableBundleSource
    {
        public Bodies Bodies;
        public BroadPhase BroadPhase;
        public QuickList<int, Buffer<int>> BodyIndices;
        public int Count
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return BodyIndices.Count; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherCollidableBundle(int collidablesStartIndex, int count, out Vector<int> shapeIndices, out Vector<float> maximumExpansion, out RigidPoses poses, out BodyVelocities velocities)
        {
            Bodies.GatherDataForBounds(ref BodyIndices[collidablesStartIndex], count, out poses, out velocities, out shapeIndices, out maximumExpansion);
        }

        //TODO: There's a compiler bug if this is inlined (wrong values are written; presumably caused by the same compiler bug affecting the Bodies.GatherInertiaAndPose. 
        //Revisit once newer compiler versions become available.
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int startIndex, int count)
        {
            ref var minBase = ref Unsafe.As<Vector<float>, float>(ref min.X);
            ref var maxBase = ref Unsafe.As<Vector<float>, float>(ref max.X);
            for (int i = 0; i < count; ++i)
            {
                //Note that we're hardcoding a relationship between the broadphase implementation and AABB calculation. This increases coupling and makes it harder to swap
                //broadphases, but in practice, I don't think a single person besides me ever created a broad phase for v1, and there was only ever a single broad phase implementation 
                //that was worth using at any given time. Abstraction for the sake of abstraction at the cost of virtual calls everywhere isn't worth it.
                BroadPhase.GetActiveBoundsPointers(Bodies.Collidables[BodyIndices[startIndex + i]].BroadPhaseIndex, out var minPointer, out var maxPointer);
                //TODO: Check codegen.
                *minPointer = Unsafe.Add(ref minBase, i);
                *(minPointer + 1) = Unsafe.Add(ref minBase, i + Vector<float>.Count);
                *(minPointer + 2) = Unsafe.Add(ref minBase, i + Vector<float>.Count * 2);
                *maxPointer = Unsafe.Add(ref maxBase, i);
                *(maxPointer + 1) = Unsafe.Add(ref maxBase, i + Vector<float>.Count);
                *(maxPointer + 2) = Unsafe.Add(ref maxBase, i + Vector<float>.Count * 2);
            }
        }
    }

    public abstract class ShapeBatch
    {
        protected RawBuffer shapesData;
        protected int shapeDataSize;
        protected BufferPool pool;
        public abstract void ComputeBounds<TBundleSource>(ref TBundleSource source, float dt) where TBundleSource : ICollidableBundleSource;
        public abstract void RemoveAt(int index);

        public abstract void ComputeBounds(int shapeIndex, ref RigidPose pose, out Vector3 min, out Vector3 max);
        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose

        /// <summary>
        /// Gets a raw untyped pointer to a shape's data.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to look up.</param>
        /// <param name="shapePointer">Pointer to the indexed shape data.</param>
        /// <param name="shapeSize">Size of the shape data in bytes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetShapeData(int shapeIndex, out void* shapePointer, out int shapeSize)
        {
            shapePointer = shapesData.Memory + shapeDataSize * shapeIndex;
            shapeSize = shapeDataSize;
        }
    }

    public class ShapeBatch<TShape> : ShapeBatch where TShape : struct, IShape//TODO: When blittable is supported, shapes should be made blittable. We store them in buffers.
    {

        internal Buffer<TShape> shapes;
        protected IdPool<Buffer<int>> idPool;

        void Resize(int shapeCount, int oldCopyLength)
        {
            shapeDataSize = Unsafe.SizeOf<TShape>();
            var requiredSizeInBytes = shapeCount * Unsafe.SizeOf<TShape>();
            pool.Take(requiredSizeInBytes, out var newShapesData);
            var newShapes = newShapesData.As<TShape>();
#if DEBUG
            //In debug mode, unused slots are kept at the default value. This helps catch misuse.
            newShapes.Clear(shapes.Length, newShapes.Length - shapes.Length);
#endif
            if (shapesData.Allocated)
            {
                shapes.CopyTo(0, ref newShapes, 0, oldCopyLength);
                pool.Return(ref shapesData);
            }
            else
            {
                Debug.Assert(oldCopyLength == 0);
            }
            shapes = newShapes;
            shapesData = newShapesData;
        }

        public ShapeBatch(BufferPool pool, int initialShapeCount)
        {
            this.pool = pool;
            Resize(initialShapeCount, 0);
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialShapeCount, out idPool);
        }

        /// <summary>
        /// Gets a reference to the shape associated with an index.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape reference to retrieve.</param>
        /// <returns>Reference to the shape at the given index.</returns>
        public ref TShape this[int shapeIndex] { get { return ref shapes[shapeIndex]; } }





        //Note that shapes cannot be moved; there is no reference to the collidables using them, so we can't correct their indices.
        //But that's fine- we never directly iterate over the shapes set anyway.
        //(This doesn't mean that it's impossible to compact the shape set- it just requires doing so by iterating over collidables.)
        public int Add(ref TShape shape)
        {
            var shapeIndex = idPool.Take();
            if (shapes.Length <= shapeIndex)
            {
                Resize(shapeIndex + 1, shapes.Length);
            }
            Debug.Assert(SpanHelper.IsZeroed(ref shapes[shapeIndex]), "In debug mode, the slot a shape is stuck into should be cleared. If it's not, it is already in use.");
            shapes[shapeIndex] = shape;
            return shapeIndex;
        }

        public sealed override void RemoveAt(int index)
        {
#if DEBUG
            Debug.Assert(!SpanHelper.IsZeroed(ref shapes[index]),
                "Either a shape was default constructed (which is almost certainly invalid), or this is attempting to remove a shape that was already removed.");
            //Don't have to actually clear out the shape set since everything is blittable. For debug purposes, we do, just to catch invalid usages.
            shapes[index] = default(TShape);
#endif
            idPool.Return(index, pool.SpecializeFor<int>());
        }

        public override void ComputeBounds<TBundleSource>(ref TBundleSource source, float dt)
        {
            for (int i = 0; i < source.Count; i += Vector<float>.Count)
            {
                int count = source.Count - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;
                source.GatherCollidableBundle(i, count, out var shapeIndices, out var maximumExpansions, out var poses, out var velocities);

                //Note that this outputs a bundle, which we turn around and immediately use. Considering only this function in isolation, they could be combined.
                //However, in the narrow phase, it's useful to be able to gather shapes, and you don't want to do bounds computation at the same time.
                default(TShape).GetBounds(ref shapes, ref shapeIndices, count, ref poses.Orientation, out var maximumRadius, out var maximumAngularExpansion, out var min, out var max);
                Vector3Wide.Add(ref min, ref poses.Position, out min);
                Vector3Wide.Add(ref max, ref poses.Position, out max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumAngularExpansion, ref maximumExpansions);
                source.ScatterBounds(ref min, ref max, i, count);
            }
        }

        public override void ComputeBounds(int shapeIndex, ref RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].GetBounds(ref pose.Orientation, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }

        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }
    public class Shapes
    {
        QuickList<ShapeBatch, Array<ShapeBatch>> batches;

        //Note that not every index within the batches list is guaranteed to be filled. For example, if only a cylinder has been added, and a cylinder's type id is 7,
        //then the batches.Count and RegisteredTypeSpan will be 8- but indices 0 through 6 will be null.
        //We don't tend to do any performance sensitive iteration over shape type batches, so this lack of contiguity is fine.
        public int RegisteredTypeSpan => batches.Count;

        public int InitialCapacityPerTypeBatch { get; set; }
        public ShapeBatch this[int typeIndex] => batches[typeIndex];
        BufferPool pool;

        public Shapes(BufferPool pool, int initialCapacityPerTypeBatch)
        {
            InitialCapacityPerTypeBatch = initialCapacityPerTypeBatch;
            //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            QuickList<ShapeBatch, Array<ShapeBatch>>.Create(new PassthroughArrayPool<ShapeBatch>(), 16, out batches);
            this.pool = pool;
        }

        /// <summary>
        /// Computes a bounding box for a single shape.
        /// </summary>
        /// <param name="pose">Pose to calculate the bounding box of.</param>
        /// <param name="shapeIndex">Index of the shape.</param>
        /// <param name="bounds">Bounding box of the specified shape with the specified pose.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateBounds(ref RigidPose pose, ref TypedIndex shapeIndex, out BoundingBox bounds)
        {
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            batches[shapeIndex.Type].ComputeBounds(shapeIndex.Index, ref pose, out bounds.Min, out bounds.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShape GetShape<TShape>(int shapeIndex) where TShape : struct, IShape
        {
            var typeId = default(TShape).TypeId;
            return ref Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId])[shapeIndex];
        }

        public TypedIndex Add<TShape>(ref TShape shape) where TShape : struct, IShape
        {
            var typeId = default(TShape).TypeId;
            if (RegisteredTypeSpan <= typeId)
            {
                if (batches.Span.Length <= typeId)
                {
                    batches.Resize(typeId, new PassthroughArrayPool<ShapeBatch>());
                }
                batches.Count = typeId + 1;
                batches[typeId] = new ShapeBatch<TShape>(pool, InitialCapacityPerTypeBatch);
            }
            Debug.Assert(batches[typeId] is ShapeBatch<TShape>);
            var batch = Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId]);
            var index = batch.Add(ref shape);
            return new TypedIndex(typeId, index);
        }

        public void Remove(TypedIndex shapeIndex)
        {
            Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
            batches[shapeIndex.Type].RemoveAt(shapeIndex.Index);
        }


        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }
}
