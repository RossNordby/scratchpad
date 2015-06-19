using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public unsafe struct PenetrationConstraint
    {
        //Constraint Description
        public RigidBody* ConnectionA;
        public RigidBody* ConnectionB;
        /// <summary>
        /// Normal pointing out from ConnectionA.
        /// </summary>
        public Vector3 ContactNormal;
        public Vector3 ContactPosition;
        public float ContactPenetration;

        //Solver-Computed
        public Vector3 LinearJacobianA;
        public Vector3 LinearJacobianB;
        public Vector3 AngularJacobianA;
        public Vector3 AngularJacobianB;
        public float PenetrationBias;
        public float Softness;
        public float EffectiveMass;

        public float AccumulatedImpulse;

        public void Prestep(float inverseDt)
        {
            //Single constraint version.
            //C = dot(Pa - Pb, N) > 0
            //Jacobians:
            //LinearA: N
            //AngularA: cross(OffsetPa, N) 
            //LinearB: -N
            //AngularB: -cross(OffsetPb, N)

            LinearJacobianA = ContactNormal;
            LinearJacobianB = -ContactNormal;
            var offsetA = ContactPosition - ConnectionA->Position;
            var offsetB = ContactPosition - ConnectionB->Position;

            //Note scalar implementation; JIT doesn't do shuffles yet.
            Vector3Ex.Cross(ref offsetA, ref ContactNormal, out AngularJacobianA);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB, out AngularJacobianB); //note negation->parameter reverse

            //Allow velocity that closes a gap, and apply penetration correction against positive depth.
            //Bounciness not yet included.
            PenetrationBias = ContactPenetration * inverseDt;
            PenetrationBias = -Math.Min(Math.Min(PenetrationBias, PenetrationBias * 0.2f), 0.2f);

            //The inertia tensor is local only, so any nonunit jacobians must be pulled into local space.
            Vector3 localAngularJacobianA, localAngularJacobianB;
            Matrix3x3.TransformTranspose(ref AngularJacobianA, ref ConnectionA->Orientation, out localAngularJacobianA);
            Matrix3x3.TransformTranspose(ref AngularJacobianB, ref ConnectionB->Orientation, out localAngularJacobianB);

            Vector3 angularA, angularB;
            Matrix3x3.Transform(ref localAngularJacobianA, ref ConnectionA->LocalInertiaTensorInverse, out angularA);
            Matrix3x3.Transform(ref localAngularJacobianB, ref ConnectionB->LocalInertiaTensorInverse, out angularB);
            float inverseEffectiveMass = ConnectionA->InverseMass + ConnectionB->InverseMass + Vector3.Dot(angularA, angularA) + Vector3.Dot(angularB, angularB);
            const float CollisionSoftness = 5;
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            EffectiveMass = 1f / (Softness + inverseEffectiveMass);


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(float lambda)
        {
            ConnectionA->LinearVelocity -= (lambda * ConnectionA->InverseMass) * LinearJacobianA;
            ConnectionB->LinearVelocity -= (lambda * ConnectionB->InverseMass) * LinearJacobianB;
            //No world inertia available, so rotate in and out.
            Vector3 angularImpulseA = lambda * AngularJacobianA;
            Vector3 angularImpulseB = lambda * AngularJacobianB;
            Vector3 velocityChangeA, velocityChangeB;
            Matrix3x3.TransformTranspose(ref angularImpulseA, ref ConnectionA->Orientation, out velocityChangeA);
            Matrix3x3.TransformTranspose(ref angularImpulseB, ref ConnectionB->Orientation, out velocityChangeB);
            Matrix3x3.Transform(ref angularImpulseA, ref ConnectionA->LocalInertiaTensorInverse, out velocityChangeA);
            Matrix3x3.Transform(ref angularImpulseB, ref ConnectionB->LocalInertiaTensorInverse, out velocityChangeB);
            Matrix3x3.Transform(ref velocityChangeA, ref ConnectionA->Orientation, out velocityChangeA);
            Matrix3x3.Transform(ref velocityChangeB, ref ConnectionB->Orientation, out velocityChangeB);
            ConnectionA->AngularVelocity -= velocityChangeA;
            ConnectionB->AngularVelocity -= velocityChangeB;
        }

        public void WarmStart()
        {
            ApplyImpulse(AccumulatedImpulse);
        }



        public void SolveIteration()
        {
            var lambda = EffectiveMass * (
                Vector3.Dot(LinearJacobianA, ConnectionA->LinearVelocity) +
                Vector3.Dot(AngularJacobianA, ConnectionA->AngularVelocity) +
                Vector3.Dot(LinearJacobianB, ConnectionB->LinearVelocity) +
                Vector3.Dot(AngularJacobianB, ConnectionB->AngularVelocity) +
                PenetrationBias - AccumulatedImpulse * Softness);

            var previous = AccumulatedImpulse;
            AccumulatedImpulse = Math.Max(0, AccumulatedImpulse + lambda);
            lambda = AccumulatedImpulse - previous;

            ApplyImpulse(lambda);
        }
    }
}
