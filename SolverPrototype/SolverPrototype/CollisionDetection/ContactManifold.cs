using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype.CollisionDetection
{
    //TODO: These memory layouts are questionable; should probably see if they can be improved once we have something going.
    /// <summary>
    /// Information about a single contact in a nonconvex collidable pair.
    /// Nonconvex pairs can have different surface bases at each contact point, since the contact surface is not guaranteed to be a plane.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 36)]
    public struct NonconvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Surface basis of the contact. If transformed into a rotation matrix, X and Z represent tangent directions and Y represents the contact normal.
        /// </summary>
        [FieldOffset(16)]
        public BEPUutilities2.Quaternion SurfaceBasis;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        [FieldOffset(32)]
        public int FeatureId;
    }
    /// <summary>
    /// Information about a single contact in a convex collidable pair. Convex collidable pairs share one surface basis across the manifold, since the contact surface is guaranteed to be a plane.
    /// </summary>    
    [StructLayout(LayoutKind.Explicit, Size = 20)]
    public struct ConvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        [FieldOffset(16)]
        public int FeatureId;
    }


    //TODO: We could use specialized storage types for things like continuations if L2 can't actually hold it all. Seems unlikely, but it's not that hard if required.

    /// <summary>
    /// Contains the data associated with a convex or nonconvex contact manifold.
    /// </summary>
    /// <remarks>The idea here is to unify producers and consumers of convex and nonconvex manifolds. In many cases, this type is far larger than is required- we could store far less
    /// when we know there's only going to be one contact. However, having a single representation keeps things a little simpler.</remarks>
    [StructLayout(LayoutKind.Explicit, Size = 160)]
    public unsafe struct ContactManifold
    {
        /// <summary>
        /// Offset from collidable A to collidable B.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 OffsetB;
        [FieldOffset(12)]
        public int PackedConvexityAndContactCount;
        /// <summary>
        /// Gets or sets whether the manifold should be considered convex. Setting this requires properly configuring each used NonconvexContact in the manifold.
        /// </summary>
        public bool Convex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return (PackedConvexityAndContactCount & (1 << 4)) > 0;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                PackedConvexityAndContactCount = (PackedConvexityAndContactCount & (~(1 << 4))) | ((*(int*)&value) << 4); //hack^__^ 
            }
        }
        /// <summary>
        /// Gets or sets the number of contacts in the manifold.
        /// </summary>
        public int ContactCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return PackedConvexityAndContactCount & 7;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                PackedConvexityAndContactCount = value | (PackedConvexityAndContactCount & 8);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetConvexityAndCount(int count, bool convex)
        {
            PackedConvexityAndContactCount = count | ((*(int*)&convex) << 4);
        }

        [FieldOffset(16)]
        public NonconvexContact NonconvexContact0;
        [FieldOffset(52)]
        public NonconvexContact NonconvexContact1;
        [FieldOffset(88)]
        public NonconvexContact NonconvexContact2;
        [FieldOffset(124)]
        public NonconvexContact NonconvexContact3;

        /// <summary>
        /// Surface basis of of the whole manifold when the manifold type is convex.
        /// </summary>
        [FieldOffset(16)]
        public BEPUutilities2.Quaternion ConvexSurfaceBasis;
        [FieldOffset(32)]
        public ConvexContact ConvexContact0;
        [FieldOffset(52)]
        public ConvexContact ConvexContact1;
        [FieldOffset(72)]
        public ConvexContact ConvexContact2;
        [FieldOffset(92)]
        public ConvexContact ConvexContact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactManifold(int contactCount, bool convex) : this()
        {
            SetConvexityAndCount(contactCount, convex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref NonconvexContact GetNonconvexContact(ref ContactManifold manifold, int index)
        {
            return ref Unsafe.Add(ref manifold.NonconvexContact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref NonconvexContact GetConvexContact(ref ContactManifold manifold, int index)
        {
            return ref Unsafe.Add(ref manifold.NonconvexContact0, index);
        }

    }

}