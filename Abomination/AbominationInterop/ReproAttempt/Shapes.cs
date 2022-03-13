
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

public class Shapes
{
    ShapeBatch[] batches;
    int registeredTypeSpan;

    //Note that not every index within the batches list is guaranteed to be filled. For example, if only a cylinder has been added, and a cylinder's type id is 7,
    //then the batches.Count and RegisteredTypeSpan will be 8- but indices 0 through 6 will be null.
    //We don't tend to do any performance sensitive iteration over shape type batches, so this lack of contiguity is fine.
    public int RegisteredTypeSpan => registeredTypeSpan;

    public int InitialCapacityPerTypeBatch { get; set; }
    public ShapeBatch this[int typeIndex] => batches[typeIndex];
    BufferPool pool;


    public Shapes(BufferPool pool, int initialCapacityPerTypeBatch)
    {
        InitialCapacityPerTypeBatch = initialCapacityPerTypeBatch;
        //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
        batches = new ShapeBatch[16];
        this.pool = pool;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ref TShape GetShape<TShape>(int shapeIndex) where TShape : unmanaged, IShape
    {
        var typeId = default(TShape).TypeId;
        return ref Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId])[shapeIndex];
    }


    public TypedIndex Add<TShape>(in TShape shape) where TShape : unmanaged, IShape
    {
        var typeId = default(TShape).TypeId;
        if (RegisteredTypeSpan <= typeId)
        {
            registeredTypeSpan = typeId + 1;
            if (batches.Length <= typeId)
            {
                Array.Resize(ref batches, typeId + 1);
            }
        }
        if (batches[typeId] == null)
        {
            batches[typeId] = default(TShape).CreateShapeBatch(pool, InitialCapacityPerTypeBatch, this);
        }

        Debug.Assert(batches[typeId] is ShapeBatch<TShape>);
        var batch = Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId]);
        var index = batch.Add(shape);
        return new TypedIndex(typeId, index);
    }


    /// <summary>
    /// Clears all shapes from existing batches. Does not release any memory.
    /// </summary>
    public void Clear()
    {
        for (int i = 0; i < registeredTypeSpan; ++i)
        {
            if (batches[i] != null)
                batches[i].Clear();
        }
    }

    //Technically we're missing some degrees of freedom here, but these are primarily convenience functions. The underlying batches have the remaining (much more rarely used) functionality.
    //You may also note that we don't have any form of per-type minimum capacities like we do in the solver. The solver benefits from tighter 'dynamic' control over allocations
    //because type batches are expected to be created and destroyed pretty frequently- sometimes multiple times a frame. Contact constraints come and go regardless of user input.
    //Shapes, on the other hand, only get added or removed by the user.
    /// <summary>
    /// Ensures a minimum capacity for all existing shape batches.
    /// </summary>
    /// <param name="shapeCapacity">Capacity to ensure for all existing shape batches.</param>
    public void EnsureBatchCapacities(int shapeCapacity)
    {
        for (int i = 0; i < registeredTypeSpan; ++i)
        {
            if (batches[i] != null)
                batches[i].EnsureCapacity(shapeCapacity);
        }
    }

    /// <summary>
    /// Resizes all existing batches for a target capacity. Note that this is conservative; it will never orphan an existing shape.
    /// </summary>
    /// <param name="shapeCapacity">Capacity to target for all existing shape batches.</param>
    public void ResizeBatches(int shapeCapacity)
    {
        for (int i = 0; i < registeredTypeSpan; ++i)
        {
            if (batches[i] != null)
                batches[i].Resize(shapeCapacity);
        }
    }

    /// <summary>
    /// Releases all memory from existing batches. Leaves shapes set in an unusable state.
    /// </summary>
    public void Dispose()
    {
        for (int i = 0; i < registeredTypeSpan; ++i)
        {
            if (batches[i] != null)
                batches[i].Dispose();
        }
    }
}