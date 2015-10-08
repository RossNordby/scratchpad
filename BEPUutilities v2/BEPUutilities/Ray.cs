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
            var positionMin = boundingBox.Min - ray.Position;
            var positionMax = boundingBox.Max - ray.Position;

            var inverseDirection = Vector3.One / ray.Direction;

            Vector3 tMin, tMax;
            tMin = positionMin * inverseDirection;
            tMax = positionMax * inverseDirection;

            var positiveFilter = new Vector3(float.MaxValue);
            var negativeFilter = new Vector3(float.MinValue);

            //Careful! parameter order matters here- this is designed to deal with NaNs.
            //NaNs become float.MaxValue in tMax, and float.MinValue in tMin.
            //This ensures that any NaNs are used to expand the intervals.
            tMin = Vector3.Max(tMin, negativeFilter);
            tMax = Vector3.Min(tMax, positiveFilter);

            Vector3 tEarly = Vector3.Min(tMin, tMax);
            Vector3 tLate = Vector3.Max(tMin, tMax);

            //All intervals from tEarly to tLate must overlap for there to exist an intersection.
            //This would benefit from some more instructions...
            t = MathHelper.Max(0, MathHelper.Max(MathHelper.Max(tEarly.X, tEarly.Y), tEarly.Z));
            var earliestLate = MathHelper.Min(MathHelper.Min(tLate.X, tLate.Y), tLate.Z);

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
