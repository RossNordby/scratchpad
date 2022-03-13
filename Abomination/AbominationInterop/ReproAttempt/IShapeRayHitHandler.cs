
using System.Numerics;

public interface IShapeRayHitHandler
{
    /// <summary>
    /// Checks whether the child of a collidable should be tested against a ray. Only called by shape types that can have more than one child.
    /// </summary>
    /// <param name="childIndex">Index of the candidate in the parent collidable.</param>
    /// <returns>True if the child should be tested by the ray, false otherwise.</returns>
    bool AllowTest(int childIndex);
    /// <summary>
    /// Called when a ray impact has been found.
    /// </summary>
    /// <param name="ray">Information about the ray associated with this hit.</param>
    /// <param name="maximumT">Maximum distance along the ray that the traversal is allowed to go in units of ray direction length. Can be set to limit future tests.</param>
    /// <param name="t">Distance along the ray to the impact in units of ray direction length. In other words, hitLocation = ray.Origin + ray.Direction * t.</param>
    /// <param name="normal">Surface normal at the hit location.</param>
    /// <param name="childIndex">Index of the hit child. For convex shapes or other types that don't have multiple children, this is always zero.</param>
    void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, int childIndex);
}