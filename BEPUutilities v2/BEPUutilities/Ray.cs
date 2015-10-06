using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2
{
    /// <summary>
    /// Provides XNA-like ray functionality.
    /// </summary>
    public struct Ray
    {
        /// <summary>
        /// Starting position of the ray.
        /// </summary>
        public Vector3 Position;
        /// <summary>
        /// Direction in which the ray points.
        /// </summary>
        public Vector3 Direction;


        /// <summary>
        /// Constructs a new ray.
        /// </summary>
        /// <param name="position">Starting position of the ray.</param>
        /// <param name="direction">Direction in which the ray points.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Ray(Vector3 position, Vector3 direction)
        {
            this.Position = position;
            this.Direction = direction;
        }



        /// <summary>
        /// Determines if and when a ray intersects the bounding box.
        /// </summary>
        /// <param name="ray">Ray to test against the box.</param>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(ref Ray ray, ref BoundingBox boundingBox, out float t)
        {
            //Make sure the ray is pointing toward the box.
            //This could be done in a more SIMD-friendly way.
            //var absDirection = Vector3.Abs(ray.Direction);
            var positionMin = boundingBox.Min - ray.Position;
            var positionMax = boundingBox.Max - ray.Position;
            //if ((absDirection.X < 1e-7f & (positionMin.X < 0 | positionMax.X > 0)) |
            //    (absDirection.Y < 1e-7f & (positionMin.Y < 0 | positionMax.Y > 0)) |
            //    (absDirection.Z < 1e-7f & (positionMin.Z < 0 | positionMax.Z > 0)))
            //{
            //    t = 0;
            //    return false;
            //}


            //Vector3 inverseDirection = Vector3.One / ray.Direction;




            Vector3 tMin, tMax;
            tMin = positionMin / ray.Direction;
            tMax = positionMax / ray.Direction;

            tMin = Vector3.Max(Vector3.Min(new Vector3(float.MaxValue), tMin), new Vector3(float.MinValue));
            tMax = Vector3.Max(Vector3.Min(new Vector3(float.MaxValue), tMax), new Vector3(float.MinValue));
         
            Vector3 tEarly = Vector3.Min(tMin, tMax);
            Vector3 tLate = Vector3.Max(tMin, tMax);

            //All intervals from tEarly to tLate must overlap for there to exist an intersection.
            //This would benefit from some more instructions...
            t = Math.Max(0, Math.Max(Math.Max(tEarly.X, tEarly.Y), tEarly.Z));
            var earliestLate = Math.Min(Math.Min(tLate.X, tLate.Y), tLate.Z);

            return t <= earliestLate;


        }

        /// <summary>
        /// Determines if and when a ray intersects the plane.
        /// </summary>
        /// <param name="ray">Ray to test against the plane.</param>
        /// <param name="plane">Plane to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(ref Ray ray, ref Plane plane, out float t)
        {
            float velocity = Vector3.Dot(ray.Direction, plane.Normal);
            if (Math.Abs(velocity) < Toolbox.Epsilon)
            {
                t = 0;
                return false;
            }
            float distanceAlongNormal = Vector3.Dot(ray.Position, plane.Normal);
            distanceAlongNormal += plane.D;
            t = -distanceAlongNormal / velocity;
            return t >= -Toolbox.Epsilon;
        }

        /// <summary>
        /// Computes a point along a ray given the length along the ray from the ray position.
        /// </summary>
        /// <param name="t">Length along the ray from the ray position in terms of the ray's direction.</param>
        /// <param name="v">Point along the ray at the given location.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPointOnRay(float t, out Vector3 v)
        {
            v = Position + t * Direction;
        }
    }
}
