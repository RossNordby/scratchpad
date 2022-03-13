using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

//TODO: It's a little odd that this exists alongside the BepuUtilities.RigidTransform. The original reasoning was that rigid poses may end up having a non-FP32 representation.
//We haven't taken advantage of that, so right now it's pretty much a pure duplicate.
//When/if we take advantage of larger sizes, we'll have to closely analyze every use case of RigidPose to see if we need the higher precision or not.
/// <summary>
/// Represents a rigid transformation.
/// </summary>
[StructLayout(LayoutKind.Sequential, Size = 32, Pack = 1)]
public struct RigidPose
{
    //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
    //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
    //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
    //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
    /// <summary>
    /// Orientation of the pose.
    /// </summary>
    public Quaternion Orientation;
    /// <summary>
    /// Position of the pose.
    /// </summary>
    public Vector3 Position;

    /// <summary>
    /// Returns a pose with a position at (0,0,0) and identity orientation.
    /// </summary>
    public static RigidPose Identity => new RigidPose(default);

    /// <summary>
    /// Creates a rigid pose with the given position and orientation.
    /// </summary>
    /// <param name="position">Position of the pose.</param>
    /// <param name="orientation">Orientation of the pose.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RigidPose(Vector3 position, Quaternion orientation)
    {
        Position = position;
        Orientation = orientation;
    }

    /// <summary>
    /// Creates a rigid pose with the given position and identity orientation.
    /// </summary>
    /// <param name="position">Position of the pose.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RigidPose(Vector3 position)
    {
        Position = position;
        Orientation = Quaternion.Identity;
    }

    /// <summary>
    /// Creates a pose by treating a <see cref="Vector3"/> as a position. Orientation is set to identity.
    /// </summary>
    /// <param name="position">Position to use in the pose.</param>
    public static implicit operator RigidPose(Vector3 position)
    {
        return new RigidPose(position);
    }

    /// <summary>
    /// Creates a pose by treating a <see cref="Quaternion"/> as an orientation in the pose. Position is set to zero.
    /// </summary>
    /// <param name="orientation">Orientation to use in the pose.</param>
    public static implicit operator RigidPose(Quaternion orientation)
    {
        return new RigidPose(default, orientation);
    }

    /// <summary>
    /// Creates a pose from a tuple of a position and orientation.
    /// </summary>
    /// <param name="poseComponents">Position and orientation to use in the pose.</param>
    public static implicit operator RigidPose((Vector3 position, Quaternion orientation) poseComponents)
    {
        return new RigidPose(poseComponents.position, poseComponents.orientation);
    }

    /// <summary>
    /// Transforms a vector by the rigid pose: v * pose.Orientation + pose.Position.
    /// </summary>
    /// <param name="v">Vector to transform.</param>
    /// <param name="pose">Pose to transform the vector with.</param>
    /// <param name="result">Transformed vector.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Transform(in Vector3 v, in RigidPose pose, out Vector3 result)
    {
        QuaternionEx.TransformWithoutOverlap(v, pose.Orientation, out var rotated);
        result = rotated + pose.Position;
    }
    /// <summary>
    /// Transforms a vector by the inverse of a rigid pose: (v - pose.Position) * pose.Orientation^-1.
    /// </summary>
    /// <param name="v">Vector to transform.</param>
    /// <param name="pose">Pose to invert and transform the vector with.</param>
    /// <param name="result">Transformed vector.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void TransformByInverse(in Vector3 v, in RigidPose pose, out Vector3 result)
    {
        var translated = v - pose.Position;
        QuaternionEx.Conjugate(pose.Orientation, out var conjugate);
        QuaternionEx.TransformWithoutOverlap(translated, conjugate, out result);
    }
    /// <summary>
    /// Inverts the rigid transformation of the pose.
    /// </summary>
    /// <param name="pose">Pose to invert.</param>
    /// <param name="inverse">Inverse of the pose.</param>
    public static void Invert(in RigidPose pose, out RigidPose inverse)
    {
        QuaternionEx.Conjugate(pose.Orientation, out inverse.Orientation);
        QuaternionEx.Transform(-pose.Position, inverse.Orientation, out inverse.Position);
    }

    /// <summary>
    /// Concatenates one rigid transform with another. The resulting transform is equivalent to performing transform a followed by transform b.
    /// </summary>
    /// <param name="a">First transform to concatenate.</param>
    /// <param name="b">Second transform to concatenate.</param>
    /// <param name="result">Result of the concatenation.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void MultiplyWithoutOverlap(in RigidPose a, in RigidPose b, out RigidPose result)
    {
        QuaternionEx.ConcatenateWithoutOverlap(a.Orientation, b.Orientation, out result.Orientation);
        QuaternionEx.Transform(a.Position, b.Orientation, out var rotatedTranslationA);
        result.Position = rotatedTranslationA + b.Position;
    }
}



