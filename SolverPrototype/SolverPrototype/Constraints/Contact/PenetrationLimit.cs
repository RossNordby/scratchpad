using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints.Contact
{
    /// <summary>
    /// Handles the solve iterations of a bunch of 1DOF two body inequality constraints.
    /// </summary>
    public static class PenetrationLimit
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobiansAndError(ref ContactData contact, out TwoBody1DOFJacobians jacobians, out Vector<float> error)
        {
            //Technically we could take advantage of the redundant form on the linear jacobians, but it's made complex by the pretransformations.
            //For now, just leave it fully generic and unspecialized.

            //The contact penetration constraint takes the form:
            //dot(positionA + offsetA, N) >= dot(positionB + offsetB, N)
            //Or:
            //dot(positionA + offsetA, N) - dot(positionB + offsetB, N) >= 0
            //dot(positionA + offsetA - positionB - offsetB, N) >= 0
            //where positionA and positionB are the center of mass positions of the bodies offsetA and offsetB are world space offsets from the center of mass to the contact,
            //and N is a unit length vector calibrated to point from B to A. (The normal pointing direction is important; it changes the sign.)
            //In practice, we'll use the collision detection system's penetration depth instead of trying to recompute the error here.

            //So, treating the normal as constant, the velocity constraint is:
            //dot(d/dt(positionA + offsetA - positionB - offsetB), N) >= 0
            //dot(linearVelocityA + d/dt(offsetA) - linearVelocityB - d/dt(offsetB)), N) >= 0
            //The velocity of the offsets are defined by the angular velocity.
            //dot(linearVelocityA + angularVelocityA x offsetA - linearVelocityB - angularVelocityB x offsetB), N) >= 0
            //dot(linearVelocityA, N) + dot(angularVelocityA x offsetA, N) - dot(linearVelocityB, N) - dot(angularVelocityB x offsetB), N) >= 0
            //Use the properties of the scalar triple product:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) - dot(linearVelocityB, N) - dot(offsetB x N, angularVelocityB) >= 0
            //Bake in the negations:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(-offsetB x N, angularVelocityB) >= 0
            //A x B = -B x A:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(N x offsetB, angularVelocityB) >= 0
            //And there you go, the jacobians!
            //linearA: N
            //angularA: offsetA x N
            //linearB: -N
            //angularB: N x offsetB
            jacobians.LinearA = contact.Normal;
            Vector3Wide.Negate(ref contact.Normal, out jacobians.LinearB);
            Vector3Wide.CrossWithoutOverlap(ref contact.OffsetA, ref contact.Normal, out jacobians.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref contact.Normal, ref contact.OffsetB, out jacobians.AngularB);

            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            error = contact.PenetrationDepth;
        }

        //The remainder of the constraint is the Inequality1DOF implementation.
        

    }
}
