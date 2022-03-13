using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

public abstract class ShapeBatch
{
    protected Buffer<byte> shapesData;
    protected int shapeDataSize;
    /// <summary>
    /// Gets the number of shapes that the batch can currently hold without resizing.
    /// </summary>
    public int Capacity { get { return shapesData.Length / shapeDataSize; } }
    protected BufferPool pool;
    protected IdPool idPool;
    /// <summary>
    /// Gets the type id of the shape type in this batch.
    /// </summary>
    public int TypeId { get; protected set; }
    /// <summary>
    /// Gets whether this shape batch's contained type potentially contains children that require other shape batches.
    /// </summary>
    public bool Compound { get; protected set; }
    /// <summary>
    /// Gets the size of the shape type stored in this batch in bytes.
    /// </summary>
    public int ShapeDataSize { get { return shapeDataSize; } }

    protected abstract void Dispose(int index, BufferPool pool);

    public abstract void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

    /// <summary>
    /// Gets a raw untyped pointer to a shape's data.
    /// </summary>
    /// <param name="shapeIndex">Index of the shape to look up.</param>
    /// <param name="shapePointer">Pointer to the indexed shape data.</param>
    /// <param name="shapeSize">Size of the shape data in bytes.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe void GetShapeData(int shapeIndex, out void* shapePointer, out int shapeSize)
    {
        Debug.Assert(shapeIndex >= 0 && shapeIndex < Capacity);
        shapePointer = shapesData.Memory + shapeDataSize * shapeIndex;
        shapeSize = shapeDataSize;
    }


    /// <summary>
    /// Frees all shape slots without returning any resources to the pool.
    /// </summary>
    public abstract void Clear();
    /// <summary>
    /// Increases the size of the type batch if necessary to hold the target capacity.
    /// </summary>
    /// <param name="shapeCapacity">Target capacity.</param>
    public abstract void EnsureCapacity(int shapeCapacity);
    /// <summary>
    /// Changes the size of the type batch if the target capacity is different than the current capacity. Note that shrinking allocations is conservative; resizing will
    /// never allow an existing shape to point to unallocated memory.
    /// </summary>
    /// <param name="shapeCapacity">Target capacity.</param>
    public abstract void Resize(int shapeCapacity);
    /// <summary>
    /// Returns all backing resources to the pool, leaving the batch in an unusable state.
    /// </summary>
    public abstract void Dispose();

    /// <summary>
    /// Shrinks or expands the allocation of the batch's id pool. Note that shrinking allocations is conservative; resizing will never allow any pending ids to be lost.
    /// </summary>
    /// <param name="targetIdCapacity">Number of slots to allocate space for in the id pool.</param>
    public void ResizeIdPool(int targetIdCapacity)
    {
        idPool.Resize(targetIdCapacity, pool);
    }

}

public abstract class ShapeBatch<TShape> : ShapeBatch where TShape : unmanaged, IShape
{
    internal Buffer<TShape> shapes;

    /// <summary>
    /// Gets a reference to the shape associated with an index.
    /// </summary>
    /// <param name="shapeIndex">Index of the shape reference to retrieve.</param>
    /// <returns>Reference to the shape at the given index.</returns>
    public ref TShape this[int shapeIndex] { get { return ref shapes[shapeIndex]; } }

    protected ShapeBatch(BufferPool pool, int initialShapeCount)
    {
        this.pool = pool;
        TypeId = default(TShape).TypeId;
        InternalResize(initialShapeCount, 0);
        idPool = new IdPool(initialShapeCount, pool);
    }

    //Note that shapes cannot be moved; there is no reference to the collidables using them, so we can't correct their indices.
    //But that's fine- we never directly iterate over the shapes set anyway.
    //(This doesn't mean that it's impossible to compact the shape set- it just requires doing so by iterating over collidables.)
    public int Add(in TShape shape)
    {
        var shapeIndex = idPool.Take();
        if (shapes.Length <= shapeIndex)
        {
            InternalResize(shapeIndex + 1, shapes.Length);
        }
        shapes[shapeIndex] = shape;
        return shapeIndex;
    }


    void InternalResize(int shapeCount, int oldCopyLength)
    {
        shapeDataSize = Unsafe.SizeOf<TShape>();
        var requiredSizeInBytes = shapeCount * Unsafe.SizeOf<TShape>();
        pool.TakeAtLeast<byte>(requiredSizeInBytes, out var newShapesData);
        var newShapes = newShapesData.As<TShape>();
#if DEBUG
            //In debug mode, unused slots are kept at the default value. This helps catch misuse.
            if (newShapes.Length > shapes.Length)
                newShapes.Clear(shapes.Length, newShapes.Length - shapes.Length);
#endif
        if (shapesData.Allocated)
        {
            shapes.CopyTo(0, newShapes, 0, oldCopyLength);
            pool.Return(ref shapesData);
        }
        else
        {
            Debug.Assert(oldCopyLength == 0);
        }
        shapes = newShapes;
        shapesData = newShapesData;
    }

    public override void Clear()
    {
#if DEBUG
            shapes.Clear(0, idPool.HighestPossiblyClaimedId + 1);
#endif
        idPool.Clear();
    }
    public override void EnsureCapacity(int shapeCapacity)
    {
        if (shapes.Length < shapeCapacity)
        {
            InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
        }
    }

    public override void Resize(int shapeCapacity)
    {
        shapeCapacity = BufferPool.GetCapacityForCount<TShape>(Math.Max(idPool.HighestPossiblyClaimedId + 1, shapeCapacity));
        if (shapeCapacity != shapes.Length)
        {
            InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
        }
    }
    public override void Dispose()
    {
        Debug.Assert(shapesData.Id == shapes.Id, "If the buffer ids don't match, there was some form of failed resize.");
        pool.Return(ref shapesData);
        idPool.Dispose(pool);
    }
}

public class HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> : ShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
    where TChildShape : IConvexShape
    where TChildShapeWide : IShapeWide<TChildShape>
{
    public HomogeneousCompoundShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
    {
        Compound = true;
    }

    protected override void Dispose(int index, BufferPool pool)
    {
        shapes[index].Dispose(pool);
    }

    public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler)
    {
        shapes[shapeIndex].RayTest(pose, ray, ref maximumT, ref hitHandler);
    }

}

public class CompoundShapeBatch<TShape> : ShapeBatch<TShape> where TShape : unmanaged, ICompoundShape
{
    Shapes shapeBatches;

    public CompoundShapeBatch(BufferPool pool, int initialShapeCount, Shapes shapeBatches) : base(pool, initialShapeCount)
    {
        this.shapeBatches = shapeBatches;
        Compound = true;
    }

    protected override void Dispose(int index, BufferPool pool)
    {
        shapes[index].Dispose(pool);
    }


    public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler)
    {
        shapes[shapeIndex].RayTest(pose, ray, ref maximumT, shapeBatches, ref hitHandler);
    }

}
