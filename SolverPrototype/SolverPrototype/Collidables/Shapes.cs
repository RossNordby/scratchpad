using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Collidables
{
    //honestly these generics arent necessary i just wanted to see it 
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape//<TShapeBundle, TShape> where TShapeBundle : IShapeBundle<TShapeBundle, TShape> where TShape : IShape<TShapeBundle, TShape>
    {
    }
    public interface IShapeBundle//<TShapeBundle, TShape> where TShapeBundle : IShapeBundle<TShapeBundle, TShape> where TShape : IShape<TShapeBundle, TShape> 
    {
        void ComputeBoundingBoxes(ref BodyPoses poses, out Vector3Wide min, out Vector3Wide max);
    }

    public abstract class ShapeBatch
    {
        protected BufferPool pool;
        public abstract BodyCollidableBatch CreateBodyCollidableBatchForType(int initialCapacityPerCollidableBatch);

        


        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    public class ShapeBatch<TShape> : ShapeBatch where TShape : struct, IShape //TODO: When blittable is supported, shapes should be made blittable. We store them in buffers.
    {

        QuickList<TShape, Buffer<TShape>> shapes;

        public ShapeBatch(BufferPool pool, int initialShapeCount)
        {
            this.pool = pool;
            QuickList<TShape, Buffer<TShape>>.Create(pool.SpecializeFor<TShape>(), initialShapeCount, out shapes);
        }

        /// <summary>
        /// Gets a reference to the shape associated with an index.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape reference to retrieve.</param>
        /// <returns>Reference to the shape at the given index.</returns>
        public ref TShape this[int shapeIndex] { get { return ref shapes[shapeIndex]; } }


        public override BodyCollidableBatch CreateBodyCollidableBatchForType(int initialCapacity)
        {
            return new BodyCollidableBatch<TShape>(this, pool, initialCapacity);
        }


        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }
    public class Shapes
    {
        QuickList<ShapeBatch, Array<ShapeBatch>> batches;

        public int RegisteredTypeCount => batches.Count;

        public ShapeBatch this[int typeIndex] => batches[typeIndex];

        public Shapes()
        {
            //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            QuickList<ShapeBatch, Array<ShapeBatch>>.Create(new PassthroughArrayPool<ShapeBatch>(), 16, out batches);
        }

    }
}
