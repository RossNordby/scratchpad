using System;
using System.Collections.Generic;
using BEPUutilities2.DataStructures;
using BEPUutilities2.ResourceManagement;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2
{
    //TODO: It would be nice to split and improve this monolith into individually superior, organized components.


    /// <summary>
    /// Helper class with many algorithms for intersection testing and 3D math.
    /// </summary>
    public static class Toolbox
    {
        /// <summary>
        /// Large tolerance value. Defaults to 1e-5f.
        /// </summary>
        public static float BigEpsilon = 1E-5f;

        /// <summary>
        /// Tolerance value. Defaults to 1e-7f.
        /// </summary>
        public static float Epsilon = 1E-7f;

        /// <summary>
        /// Represents an invalid Vector3.
        /// </summary>
        public static readonly Vector3 NoVector = new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue);

        /// <summary>
        /// Reference for a vector with dimensions (0,0,1).
        /// </summary>
        public static Vector3 BackVector = new Vector3(0, 0, 1);

        /// <summary>
        /// Reference for a vector with dimensions (0,-1,0).
        /// </summary>
        public static Vector3 DownVector = new Vector3(0, -1, 0);

        /// <summary>
        /// Reference for a vector with dimensions (0,0,-1).
        /// </summary>
        public static Vector3 ForwardVector = new Vector3(0, 0, -1);

        /// <summary>
        /// Refers to the identity quaternion.
        /// </summary>
        public static Quaternion IdentityOrientation = Quaternion.Identity;

        /// <summary>
        /// Reference for a vector with dimensions (-1,0,0).
        /// </summary>
        public static Vector3 LeftVector = new Vector3(-1, 0, 0);

        /// <summary>
        /// Reference for a vector with dimensions (1,0,0).
        /// </summary>
        public static Vector3 RightVector = new Vector3(1, 0, 0);

        /// <summary>
        /// Reference for a vector with dimensions (0,1,0).
        /// </summary>
        public static Vector3 UpVector = new Vector3(0, 1, 0);

        /// <summary>
        /// Matrix containing zeroes for every element.
        /// </summary>
        public static Matrix ZeroMatrix = new Matrix(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        /// <summary>
        /// Reference for a vector with dimensions (0,0,0).
        /// </summary>
        public static Vector3 ZeroVector = Vector3.Zero;

        /// <summary>
        /// Refers to the rigid identity transformation.
        /// </summary>
        public static RigidTransform RigidIdentity = RigidTransform.Identity;

        /// <summary>
        /// Determines the intersection between a ray and a triangle.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length to travel in units of the direction's length.</param>
        /// <param name="a">First vertex of the triangle.</param>
        /// <param name="b">Second vertex of the triangle.</param>
        /// <param name="c">Third vertex of the triangle.</param>
        /// <param name="hitClockwise">True if the the triangle was hit on the clockwise face, false otherwise.</param>
        /// <param name="hit">Hit data of the ray, if any</param>
        /// <returns>Whether or not the ray and triangle intersect.</returns>
        public static bool FindRayTriangleIntersection(ref Ray ray, float maximumLength, ref Vector3 a, ref Vector3 b, ref Vector3 c, out bool hitClockwise, out RayHit hit)
        {
            hitClockwise = false;
            hit = new RayHit();
            Vector3 ab = b - a;
            Vector3 ac = c - a;

            Vector3x.Cross(ref ab, ref ac, out hit.Normal);
            if (hit.Normal.LengthSquared() < Epsilon)
                return false; //Degenerate triangle!

            float d = -Vector3.Dot(ray.Direction, hit.Normal);

            hitClockwise = d >= 0;

            Vector3 ap = ray.Position - a;

            hit.T = Vector3.Dot(ap, hit.Normal);
            hit.T /= d;
            if (hit.T < 0 || hit.T > maximumLength)
                return false;//Hit is behind origin, or too far away.

            hit.Location = ray.Position + hit.T * ray.Direction;

            // Compute barycentric coordinates
            ap = hit.Location - a;
            float ABdotAB, ABdotAC, ABdotAP;
            float ACdotAC, ACdotAP;
            ABdotAB = Vector3.Dot(ab, ab);
            ABdotAC = Vector3.Dot(ab, ac);
            ABdotAP = Vector3.Dot(ab, ap);
            ACdotAC = Vector3.Dot(ac, ac);
            ACdotAP = Vector3.Dot(ac, ap);

            float denom = 1 / (ABdotAB * ACdotAC - ABdotAC * ABdotAC);
            float u = (ACdotAC * ABdotAP - ABdotAC * ACdotAP) * denom;
            float v = (ABdotAB * ACdotAP - ABdotAC * ABdotAP) * denom;

            return (u >= -Toolbox.BigEpsilon) && (v >= -Toolbox.BigEpsilon) && (u + v <= 1 + Toolbox.BigEpsilon);

        }

        /// <summary>
        /// Determines the intersection between a ray and a triangle.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length to travel in units of the direction's length.</param>
        /// <param name="sidedness">Sidedness of the triangle to test.</param>
        /// <param name="a">First vertex of the triangle.</param>
        /// <param name="b">Second vertex of the triangle.</param>
        /// <param name="c">Third vertex of the triangle.</param>
        /// <param name="hit">Hit data of the ray, if any</param>
        /// <returns>Whether or not the ray and triangle intersect.</returns>
        public static bool FindRayTriangleIntersection(ref Ray ray, float maximumLength, TriangleSidedness sidedness, ref Vector3 a, ref Vector3 b, ref Vector3 c, out RayHit hit)
        {
            hit = new RayHit();
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3x.Cross(ref ab, ref ac, out hit.Normal);
            if (hit.Normal.LengthSquared() < Epsilon)
                return false; //Degenerate triangle!

            float d = -Vector3.Dot(ray.Direction, hit.Normal);
            switch (sidedness)
            {
                case TriangleSidedness.DoubleSided:
                    if (d <= 0) //Pointing the wrong way.  Flip the normal.
                    {
                        hit.Normal = -hit.Normal;
                        d = -d;
                    }
                    break;
                case TriangleSidedness.Clockwise:
                    if (d <= 0) //Pointing the wrong way.  Can't hit.
                        return false;

                    break;
                case TriangleSidedness.Counterclockwise:
                    if (d >= 0) //Pointing the wrong way.  Can't hit.
                        return false;

                    hit.Normal = -hit.Normal;
                    d = -d;
                    break;
            }

            Vector3 ap = ray.Position - a;

            hit.T = Vector3.Dot(ap, hit.Normal);
            hit.T /= d;
            if (hit.T < 0 || hit.T > maximumLength)
                return false;//Hit is behind origin, or too far away.

            hit.Location = ray.Position + hit.T * ray.Direction;

            // Compute barycentric coordinates
            ap = hit.Location - a;
            float ABdotAB, ABdotAC, ABdotAP;
            float ACdotAC, ACdotAP;
            ABdotAB = Vector3.Dot(ab, ab);
            ABdotAC = Vector3.Dot(ab, ac);
            ABdotAP = Vector3.Dot(ab, ap);
            ACdotAC = Vector3.Dot(ac, ac);
            ACdotAP = Vector3.Dot(ac, ap);

            float denom = 1 / (ABdotAB * ACdotAC - ABdotAC * ABdotAC);
            float u = (ACdotAC * ABdotAP - ABdotAC * ACdotAP) * denom;
            float v = (ABdotAB * ACdotAP - ABdotAC * ABdotAP) * denom;

            return (u >= -Toolbox.BigEpsilon) && (v >= -Toolbox.BigEpsilon) && (u + v <= 1 + Toolbox.BigEpsilon);

        }




        /// <summary>
        /// Computes the velocity of a point as if it were attached to an object with the given center and velocity.
        /// </summary>
        /// <param name="point">Point to compute the velocity of.</param>
        /// <param name="center">Center of the object to which the point is attached.</param>
        /// <param name="linearVelocity">Linear velocity of the object.</param>
        /// <param name="angularVelocity">Angular velocity of the object.</param>
        /// <param name="velocity">Velocity of the point.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetVelocityOfPoint(ref Vector3 point, ref Vector3 center, ref Vector3 linearVelocity, ref Vector3 angularVelocity, out Vector3 velocity)
        {
            Vector3 offset = point - center;
            Vector3x.Cross(ref angularVelocity, ref offset, out velocity);
            velocity += linearVelocity;
        }

        /// <summary>
        /// Computes the velocity of a point as if it were attached to an object with the given center and velocity.
        /// </summary>
        /// <param name="point">Point to compute the velocity of.</param>
        /// <param name="center">Center of the object to which the point is attached.</param>
        /// <param name="linearVelocity">Linear velocity of the object.</param>
        /// <param name="angularVelocity">Angular velocity of the object.</param>
        /// <returns>Velocity of the point.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 GetVelocityOfPoint(Vector3 point, Vector3 center, Vector3 linearVelocity, Vector3 angularVelocity)
        {
            Vector3 toReturn;
            GetVelocityOfPoint(ref point, ref center, ref linearVelocity, ref angularVelocity, out toReturn);
            return toReturn;
        }


    }
}