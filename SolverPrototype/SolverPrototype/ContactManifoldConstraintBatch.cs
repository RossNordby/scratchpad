using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct ContactManifoldConstraintDescription : ITwoBodyConstraintDescription
    {
        int bodyHandleA;
        public int BodyHandleA => bodyHandleA;

        int bodyHandleB;
        public int BodyHandleB => bodyHandleB;

        public static int TypeId;
        public int ConstraintTypeId => TypeId;

        public ContactManifoldConstraintDescription(int bodyHandleA, int bodyHandleB)
        {
            this.bodyHandleA = bodyHandleA;
            this.bodyHandleB = bodyHandleB;
        }
    }

    public struct ContactData
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide Normal;
        public Vector<float> PenetrationDepth;
    }

    /// <summary>
    /// Handles a batch of contact manifold constraints. 
    /// </summary>
    /// <remarks>Note that the implementation of a constraint is handled at the batch level. Individual constraints are just blobs of data.
    /// There is no SuchAndSuchConstraint.cs that contains per-constraint Prestep/WarmStart/Solve functions. This is it!</remarks>
    public class ContactManifoldConstraintBatch : ConstraintTypeBatch
    {
        int bundleCount;
        int constraintCount;
        public override int ConstraintCount => constraintCount;

        public static int TypeId;
        public override int ConstraintTypeIndex => TypeId;

        /// <summary>
        /// Gets the bodies set that this batch operates on.
        /// </summary>
        public Bodies Bodies { get; private set; }

        internal IterationData2Body1DOF[] IterationData;
        internal Vector<float>[] AccumulatedImpulse;
        struct ContactManifoldPrestepData
        {
            public Vector<int> SomeKindOfContactReference;
            public BodyReferences BodyReferences;
        }
        ContactManifoldPrestepData[] prestepData;


        public ContactManifoldConstraintBatch(Bodies bodies)
        {
            Bodies = bodies;
        }

        public override void Add<TAddDescription>(ref TAddDescription genericDescription)
        {
            Debug.Assert(typeof(TAddDescription) == typeof(ContactManifoldConstraintDescription), "We can only accept a specific type. The generics just allowed us to maintain an unboxed description.");
            //Super hack! Didn't want to do a bunch of type specific work in the users, but wanted to maintain the use of structs without boxing.
            //Assuming the usage isn't bugged, this cast is always equivalent to Unsafe.As<T, T> at runtime- but doing this here gives us compile time type information.
            ref var description = ref Unsafe.As<TAddDescription, ContactManifoldConstraintDescription>(ref genericDescription);
        }


        public override void Remove(int constraintHandle)
        {
            throw new NotImplementedException();
        }

        public override void Reset()
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobiansAndError(ref ContactData contact, out TwoBody1DOFJacobians jacobians, out Vector<float> error)
        {
            //Technically we could take advantage of the redundant form on the linear jacobians, but it's made complex by the pretransformations.
            //For now, just leave it fully generic and unspecialized.

            //The contact penetration constraint takes the form:
            //dot(positionA + offsetA, N) <= dot(positionB + offsetB, N)
            //Or:
            //dot(positionA + offsetA, N) - dot(positionB + offsetB, N) <= 0
            //dot(positionA + offsetA - positionB - offsetB, N) <= 0
            //In practice, we'll use the collision detection system's penetration depth instead of trying to recompute the error here.

            //So, treating the normal as constant, the velocity constraint is:
            //dot(d/dt(positionA + offsetA - positionB - offsetB), N) <= 0
            //dot(linearVelocityA + d/dt(offsetA) - linearVelocityB - d/dt(offsetB)), N) <= 0
            //The velocity of the offsets are defined by the angular velocity.
            //dot(linearVelocityA + angularVelocityA x offsetA - linearVelocityB - angularVelocityB x offsetB), N) <= 0
            //dot(linearVelocityA, N) + dot(angularVelocityA x offsetA, N) - dot(linearVelocityB, N) - dot(angularVelocityB x offsetB), N) <= 0
            //Use the properties of the scalar triple product:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) - dot(linearVelocityB, N) - dot(offsetB x N, angularVelocityB) <= 0
            //Bake in the negations:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(-offsetB x N, angularVelocityB) <= 0
            //A x B = -B x A:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(N x offsetB, angularVelocityB) <= 0
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

        public override void Prestep()
        {
            for (int i = 0; i < bundleCount; ++i)
            {
                GatherContactData(ref prestepData[i], out var contactData, );
                ComputeJacobiansAndError(ref contactData, out var jacobians, out var error);
                Inequality2Body1DOF.Prestep(Bodies.InertiaBundles, ref prestepData[i].BodyReferences, ref IterationData[i], ref jacobians, ref contactData.SpringSettings, ref error, ref prestepData.MaximumReocver)
            }
        }
        public override void WarmStart()
        {
            throw new NotImplementedException();
        }
        public override void SolveIteration()
        {
            throw new NotImplementedException();
        }



    }
}
