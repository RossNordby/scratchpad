using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;

namespace SolverPrototype.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape<TShape, TShapeBundle> where TShape : IShape<TShape, TShapeBundle> where TShapeBundle : IShapeBundle
    {
        float MaximumRadius { get; }

        void Gather(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, out TShapeBundle shapeBundle);
    }
    public interface IShapeBundle//<TShapeBundle, TShape> where TShapeBundle : IShapeBundle<TShapeBundle, TShape> where TShape : IShape<TShapeBundle, TShape> 
    {
        void GetBounds(ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max);
    }

    public abstract class ShapeBatch
    {
        protected BufferPool pool;
        public abstract void ComputeBounds<TBundleSource>(ref TBundleSource source) where TBundleSource : ICollidableBundleSource;
        public abstract void RemoveAt(int index);
        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    public class ShapeBatch<TShape, TShapeBundle> : ShapeBatch where TShape : struct, IShape<TShape, TShapeBundle> where TShapeBundle : IShapeBundle //TODO: When blittable is supported, shapes should be made blittable. We store them in buffers.
    {

        Buffer<TShape> shapes;
        protected IdPool<Buffer<int>, BufferPool<int>> idPool;

        public ShapeBatch(BufferPool pool, int initialShapeCount)
        {
            this.pool = pool;
            pool.SpecializeFor<TShape>().Take(initialShapeCount, out shapes);
#if DEBUG
            //In debug mode, unused slots are kept at the default value. This helps catch misuse.
            shapes.Clear(0, shapes.Length);
#endif
            idPool = new IdPool<Buffer<int>, BufferPool<int>>(pool.SpecializeFor<int>(), initialShapeCount);
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
                pool.SpecializeFor<TShape>().Take(shapeIndex + 1, out var newSpan);
                shapes.CopyTo(0, ref newSpan, 0, shapes.Length);
#if DEBUG
                //In debug mode, unused slots are kept at the default value. This helps catch misuse.
                newSpan.Clear(shapes.Length, newSpan.Length - shapes.Length);
#endif
                pool.SpecializeFor<TShape>().Return(ref shapes);
                shapes = newSpan;
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
            idPool.Return(index);
        }

        public override void ComputeBounds<TBundleSource>(ref TBundleSource source, float dt)
        {
            for (int i = 0; i < source.Count; i += Vector<float>.Count)
            {
                source.GatherCollidableBundle(i, out var shapeIndices, out var maximumExpansions, out var poses, out var velocities);
                //TODO: Confirm zero overhead.
                //Note that this outputs a bundle, which we turn around and immediately use. Considering only this function in isolation, they could be combined.
                //However, in the narrow phase, it's useful to be able to gather shapes, and you don't want to do bounds computation at the same time.
                default(TShape).Gather(ref shapes, ref shapeIndices, out var shapeBundle);
                shapeBundle.GetBounds(ref poses, out var maximumRadius, out var min, out var max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumExpansions);
                source.ScatterBounds(ref min, ref max, i);
            }
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShape GetShape<TShape>(int shapeIndex) where TShape : struct, IShape<TShape, TShapeBundle>
        {
            var typeIndex = TypeIds<>.GetId<TShape>();
            return ref Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeIndex])[shapeIndex];
        }

        public TypedIndex Add<TShape>(ref TShape shape) where TShape : struct, IShape
        {
            var typeId = TypeIds<IShape>.GetId<TShape>();
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
