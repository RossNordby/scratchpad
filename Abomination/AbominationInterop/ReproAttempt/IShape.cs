using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;


/// <summary>
/// Defines a type usable as a shape by collidables.
/// </summary>
public interface IShape
{
    //TODO: Note that these should really be *static* as they do not need any information about an instance, but static abstract interface methods are not yet out of preview.
    /// <summary>
    /// Unique type id for this shape type.
    /// </summary>
    int TypeId { get; }

    /// <summary>
    /// Creates a shape batch for this type of shape.
    /// </summary>
    /// <param name="pool">Buffer pool used to create the batch.</param>
    /// <param name="initialCapacity">Initial capacity to allocate within the batch.</param>
    /// <param name="shapeBatches">The set of shapes to contain this batch.</param>
    /// <returns>Shape batch for the shape type.</returns>
    /// <remarks>This is typically used internally to initialize new shape collections in response to shapes being added. It is not likely to be useful outside of the engine.</remarks>
    ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches);
}

//Note that the following bounds functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
//By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
//deals with positions directly.

//Note that we also support one-off bounds calculations. They are used even in the engine sometimes. Adding individual bodies to the simulation, for example.
//Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
//I don't predict ever needing it, but such an implementation could be added...

/// <summary>
/// Defines functions available on all convex shapes. Convex shapes have no hollowed out regions; any line passing through a convex shape will never enter and exit more than once.
/// </summary>
public interface IConvexShape : IShape
{
    bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal);
}

/// <summary>
/// Defines a compound shape type that has children of potentially different types.
/// </summary>
public interface ICompoundShape : IShape
{
    void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, Shapes shapeBatches, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
    int ChildCount { get; }
    void Dispose(BufferPool pool);
}

/// <summary>
/// Defines a compound shape type that has children of only one type.
/// </summary>
/// <typeparam name="TChildShape">Type of the child shapes.</typeparam>
/// <typeparam name="TChildShapeWide">Type of the child shapes, formatted in AOSOA layout.</typeparam>
public interface IHomogeneousCompoundShape<TChildShape, TChildShapeWide> : IShape
    where TChildShape : IConvexShape
    where TChildShapeWide : IShapeWide<TChildShape>
{
    void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max);

    void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

    int ChildCount { get; }
    void GetLocalChild(int childIndex, out TChildShape childData);
    void GetPosedLocalChild(int childIndex, out TChildShape childData, out RigidPose childPose);
    void GetLocalChild(int childIndex, ref TChildShapeWide childData);
    void Dispose(BufferPool pool);
}

public interface IShapeWide<TShape> where TShape : IShape
{

    /// <summary>
    /// Gets whether this type supports accessing its memory by lane offsets. If false, WriteSlot must be used instead of WriteFirst.
    /// </summary>
    bool AllowOffsetMemoryAccess { get; }
    /// <summary>
    /// Gets the number of bytes required for allocations within the wide shape.
    /// </summary>
    int InternalAllocationSize { get; }
    /// <summary>
    /// For types with a nonzero internal allocation size, provides memory to the shape for internal allocations.
    /// Memory should be assumed to be stack allocated.
    /// </summary>
    /// <param name="memory">Memory to use for internal allocations in the wide shape.</param>
    void Initialize(in Buffer<byte> memory);

    /// <summary>
    /// Places the specified AOS-formatted shape into the first lane of the wide 'this' reference.
    /// </summary>
    /// <remarks>Note that we are effectively using the TShapeWide as a stride.
    /// The base address is offset by the user of this function, so the implementation only ever considers the first slot.</remarks>
    /// <param name="source">AOS-formatted shape to gather from.</param>
    void WriteFirst(in TShape source);
    /// <summary>
    /// Places the specified AOS-formatted shape into the selected slot of the wide 'this' reference.
    /// </summary>
    /// <param name="index">Index of the slot to put the data into.</param>
    /// <param name="source">Source of the data to insert.</param>
    void WriteSlot(int index, in TShape source);
    void Broadcast(in TShape shape);

    void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max);
    /// <summary>
    /// Gets the lower bound on the number of rays to execute in a wide fashion. Ray bundles with fewer rays will fall back to the single ray code path.
    /// </summary>
    int MinimumWideRayCount { get; }
}
