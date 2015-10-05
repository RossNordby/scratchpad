﻿using System;
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
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(ref BoundingBox boundingBox, out float t)
        {
            if (Math.Abs(Direction.X) < Toolbox.Epsilon && (Position.X < boundingBox.Min.X || Position.X > boundingBox.Max.X))
            {
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                t = 0;
                return false;
            }
            float tmin = 0, tmax = float.MaxValue;
            float inverseDirection = 1 / Direction.X;
            float t1 = (boundingBox.Min.X - Position.X) * inverseDirection;
            float t2 = (boundingBox.Max.X - Position.X) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                t = 0;
                return false;
            }
            if (Math.Abs(Direction.Y) < Toolbox.Epsilon && (Position.Y < boundingBox.Min.Y || Position.Y > boundingBox.Max.Y))
            {
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                t = 0;
                return false;
            }
            inverseDirection = 1 / Direction.Y;
            t1 = (boundingBox.Min.Y - Position.Y) * inverseDirection;
            t2 = (boundingBox.Max.Y - Position.Y) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                t = 0;
                return false;
            }
            if (Math.Abs(Direction.Z) < Toolbox.Epsilon && (Position.Z < boundingBox.Min.Z || Position.Z > boundingBox.Max.Z))
            {
                //If the ray isn't pointing along the axis at all, and is outside of the box's interval, then it
                //can't be intersecting.
                t = 0;
                return false;
            }
            inverseDirection = 1 / Direction.Z;
            t1 = (boundingBox.Min.Z - Position.Z) * inverseDirection;
            t2 = (boundingBox.Max.Z - Position.Z) * inverseDirection;
            if (t1 > t2)
            {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tmin = Math.Max(tmin, t1);
            tmax = Math.Min(tmax, t2);
            if (tmin > tmax)
            {
                t = 0;
                return false;
            }
            t = tmin;
            return true;
        }

        /// <summary>
        /// Determines if and when the ray intersects the bounding box.
        /// </summary>
        /// <param name="boundingBox">Bounding box to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(BoundingBox boundingBox, out float t)
        {
            return Intersects(ref boundingBox, out t);
        }

        /// <summary>
        /// Determines if and when the ray intersects the plane.
        /// </summary>
        /// <param name="plane">Plane to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(ref Plane plane, out float t)
        {
            float velocity = Vector3.Dot(Direction, plane.Normal);
            if (Math.Abs(velocity) < Toolbox.Epsilon)
            {
                t = 0;
                return false;
            }
            float distanceAlongNormal = Vector3.Dot(Position, plane.Normal);
            distanceAlongNormal += plane.D;
            t = -distanceAlongNormal / velocity;
            return t >= -Toolbox.Epsilon;
        }

        /// <summary>
        /// Determines if and when the ray intersects the plane.
        /// </summary>
        /// <param name="plane">Plane to test against.</param>
        /// <param name="t">The length along the ray to the impact, if any impact occurs.</param>
        /// <returns>True if the ray intersects the target, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(Plane plane, out float t)
        {
            return Intersects(ref plane, out t);
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
