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
    void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
}

public interface IShapeWide<TShape> where TShape : IShape
{


}
