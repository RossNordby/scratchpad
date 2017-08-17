using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    internal interface IContactManifold
    {
        int TypeId { get; }

    }

    /// <summary>
    /// Information about a single contact in a nonconvex collidable pair.
    /// Nonconvex pairs can have different surface bases at each contact point, since the contact surface is not guaranteed to be a plane.
    /// </summary>
    public struct NonconvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        public float Depth;
        /// <summary>
        /// Surface basis of the contact. If transformed into a rotation matrix, X and Z represent tangent directions and Y represents the contact normal.
        /// </summary>
        public BEPUutilities2.Quaternion SurfaceBasis;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        public int FeatureId;
    }
    /// <summary>
    /// Information about a single contact in a convex collidable pair. Convex collidable pairs share one surface basis across the manifold, since the contact surface is guaranteed to be a plane.
    /// </summary>
    public struct ConvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        public float Depth;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        public int FeatureId;
    }


    //This set of contact manifolds acts as the storage representation for contact manifolds for any system that needs a compressed representation.
    //For example, we use this in substeps to store only the minimum amount of information.

    public struct ConvexContactManifold1 : IContactManifold
    {
        public int TypeId => 0;

        public ConvexContact Contact;
        public Vector3 OffsetB;
        public BEPUutilities2.Quaternion SurfaceBasis;
    }
    public struct ConvexContactManifold2 : IContactManifold
    {
        public int TypeId => 1;

        public ConvexContact Contact0;
        public ConvexContact Contact1;
        public Vector3 OffsetB;
        public BEPUutilities2.Quaternion SurfaceBasis;
    }
    public struct ConvexContactManifold3 : IContactManifold
    {
        public int TypeId => 2;

        public ConvexContact Contact0;
        public ConvexContact Contact1;
        public ConvexContact Contact2;
        public Vector3 OffsetB;
        public BEPUutilities2.Quaternion SurfaceBasis;
    }
    public struct ConvexContactManifold4 : IContactManifold
    {
        public int TypeId => 3;

        public ConvexContact Contact0;
        public ConvexContact Contact1;
        public ConvexContact Contact2;
        public ConvexContact Contact3;
        public Vector3 OffsetB;
        public BEPUutilities2.Quaternion SurfaceBasis;
    }
    public struct NonconvexContactManifold1 : IContactManifold
    {
        public int TypeId => 4 + 0;

        public NonconvexContact Contact;
        public Vector3 OffsetB;
    }
    public struct NonconvexContactManifold2 : IContactManifold
    {
        public int TypeId => 4 + 1;

        public NonconvexContact Contact0;
        public NonconvexContact Contact1;
        public Vector3 OffsetB;
    }
    public struct NonconvexContactManifold3 : IContactManifold
    {
        public int TypeId => 4 + 2;

        public NonconvexContact Contact0;
        public NonconvexContact Contact1;
        public NonconvexContact Contact2;
        public Vector3 OffsetB;
    }
    public struct NonconvexContactManifold4 : IContactManifold
    {
        public int TypeId => 4 + 3;

        public NonconvexContact Contact0;
        public NonconvexContact Contact1;
        public NonconvexContact Contact2;
        public NonconvexContact Contact3;
        public Vector3 OffsetB;
    }


    public unsafe struct NonconvexContactManifold
    {
        public NonconvexContactManifold4 Data;
        public int ContactCount;
    }
    public unsafe struct ConvexContactManifold
    {
        public ConvexContactManifold4 Data;
        public int ContactCount;
    }

    /// <summary>
    /// Wraps the data associated with a convex or nonconvex contact manifold.
    /// </summary>
    /// <typeparam name="TManifold">Type of the underlying manifold.</typeparam>
    /// <remarks>
    /// <para>The idea here is to avoid the need for the user to specify two implementations of contact manifold configurers in the narrow phase callback.
    /// The assembly generated will be essentially equivalent to two separate functions, but if the user doesn't care about convexity, this wrapper simplifies things a little.
    /// </para>
    /// <para>This structure should never be stored. The pointer should be considered invalid as soon as the function to which it was passed returns.</para></remarks>
    public unsafe struct ManifoldData<TManifold>
    {
        /// <summary>
        /// Direct pointer to the manifold data.
        /// </summary>
        public byte* Manifold;

        /// <summary>
        /// Gets a reference to the manifold's contact count.
        /// </summary>
        /// <remarks>The user is trusted to not set this to an invalid value. The configuration callback can only set the count to a value between 1 and 4. If no contact is desired,
        /// then it should instead return false to stop the creation of a contact constraint.</remarks>
        public ref int ContactCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (typeof(TManifold) == typeof(ConvexContactManifold))
                    return ref Unsafe.As<byte, ConvexContactManifold>(ref *Manifold).ContactCount;
                Debug.Assert(typeof(TManifold) == typeof(NonconvexContactManifold), "Only the nonconvex and convex types should be used with this wrapper struct.");
                return ref Unsafe.As<byte, NonconvexContactManifold>(ref *Manifold).ContactCount;
            }
        }

        /// <summary>
        /// Gets whether the manifold is convex.
        /// </summary>
        public bool Convex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return typeof(TManifold) == typeof(ConvexContactManifold);
            }
        }

        /// <summary>
        /// Gets a reference to a contact at the given index. Must only be used with a convex manifold.
        /// </summary>
        /// <param name="index">Index of the contact to retrieve.</param>
        /// <returns>Reference to the contact at the given index.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContact GetConvexContact(int index)
        {
            Debug.Assert(index >= 0 && index < ContactCount);
            Debug.Assert(ContactCount > 0 && ContactCount <= 4);
            Debug.Assert(Convex, "To grab convex data, the manifold must be convex.");
            return ref ((ConvexContact*)Manifold)[index];
        }
        /// <summary>
        /// Gets a reference to a contact at the given index. Must only be used with a nonconvex manifold.
        /// </summary>
        /// <param name="index">Index of the contact to retrieve.</param>
        /// <returns>Reference to the contact at the given index.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContact GetNonconvexContact(int index)
        {
            Debug.Assert(index >= 0 && index < ContactCount);
            Debug.Assert(ContactCount > 0 && ContactCount <= 4);
            Debug.Assert(!Convex, "To grab nonconvex data, the manifold must be convex.");
            return ref ((NonconvexContact*)Manifold)[index];
        }

        /// <summary>
        /// Gets the reference to the manifold's surface basis if the underlying type is convex.
        /// </summary>
        public ref BEPUutilities2.Quaternion ConvexSurfaceBasis
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(Convex, "To grab convex data, the manifold must be convex.");
                return ref Unsafe.As<byte, ConvexContactManifold>(ref *Manifold).Data.SurfaceBasis;
            }
        }
        /// <summary>
        /// Gets the reference to the manifold's offset from collidable A to collidable B.
        /// </summary>
        public ref Vector3 OffsetB
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                if (typeof(TManifold) == typeof(ConvexContactManifold))
                    return ref Unsafe.As<byte, ConvexContactManifold>(ref *Manifold).Data.OffsetB;
                Debug.Assert(typeof(TManifold) == typeof(NonconvexContactManifold), "Only the nonconvex and convex types should be used with this wrapper struct.");
                return ref Unsafe.As<byte, NonconvexContactManifold>(ref *Manifold).Data.OffsetB;
            }
        }
    }

}