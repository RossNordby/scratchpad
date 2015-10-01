using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public unsafe struct SingleVectorizedPenetrationConstraint
    {
        //Constraint Description
        public RigidBody ConnectionA;
        public RigidBody ConnectionB;
        /// <summary>
        /// Normal pointing out from ConnectionA.
        /// </summary>
        public Vector3 ContactNormal;
        public Vector3 ContactPosition;
        public float ContactPenetration;

        //Solver-Computed
        public Vector3Width4 Jacobians;

        //"Independently transformed" jacobians; takes into account mass/inertia.
        public Vector3 linearITA;
        public Vector3 angularITA;
        public Vector3 linearITB;
        public Vector3 angularITB;
        public Vector3Width4 JacobiansIT;
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

            var LinearJacobianA = ContactNormal;
            var LinearJacobianB = -ContactNormal;
            var offsetA = ContactPosition - ConnectionA.Position;
            var offsetB = ContactPosition - ConnectionB.Position;

            //Note scalar implementation; JIT doesn't do shuffles yet.
            Vector3 AngularJacobianA, AngularJacobianB;
            Vector3Ex.Cross(ref offsetA, ref ContactNormal, out AngularJacobianA);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB, out AngularJacobianB); //note negation->parameter reverse

            //Allow velocity that closes a gap, and apply penetration correction against positive depth.
            //Bounciness not yet included.
            PenetrationBias = ContactPenetration * inverseDt;
            PenetrationBias = -Math.Min(Math.Min(PenetrationBias, PenetrationBias * 0.2f), 0.2f);

            linearITA = LinearJacobianA * ConnectionA.InverseMass;
            linearITB = LinearJacobianB * ConnectionB.InverseMass;
            Matrix3x3.Transform(ref AngularJacobianA, ref ConnectionA.InertiaTensorInverse, out angularITA);
            Matrix3x3.Transform(ref AngularJacobianB, ref ConnectionB.InertiaTensorInverse, out angularITB);
            float inverseEffectiveMass = ConnectionA.InverseMass + ConnectionB.InverseMass + Vector3.Dot(angularITA, angularITA) + Vector3.Dot(angularITB, angularITB);

            const float CollisionSoftness = 5;
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            EffectiveMass = 1f / (Softness + inverseEffectiveMass);


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(float lambda)
        {
            ConnectionA.LinearVelocity -= lambda * linearITA;
            ConnectionB.LinearVelocity -= lambda * angularITA;

            ConnectionA.AngularVelocity -= lambda * linearITB;
            ConnectionB.AngularVelocity -= lambda * angularITB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(float lambda, ref Vector3Width4 velocities)
        {
            Vector3Width4 change;
            Vector3Width4.Multiply(ref JacobiansIT, lambda, out change);
            Vector3Width4.Subtract(ref velocities, ref change, out velocities);
        }

        public void WarmStart()
        {
            ApplyImpulse(AccumulatedImpulse);
        }



        public void SolveIteration()
        {
            Vector3Width4 velocities = new Vector3Width4(ref ConnectionA.LinearVelocity, ref ConnectionA.AngularVelocity, ref ConnectionB.LinearVelocity, ref ConnectionB.AngularVelocity);
            
            Vector4 velocityContributions;
            Vector3Width4.Dot(ref velocities, ref Jacobians, out velocityContributions);

            var lambda = EffectiveMass * (
                velocityContributions.X + velocityContributions.Y + velocityContributions.Z + velocityContributions.W +
                PenetrationBias - AccumulatedImpulse * Softness);

            var previous = AccumulatedImpulse;
            AccumulatedImpulse = Math.Max(0, AccumulatedImpulse + lambda);
            lambda = AccumulatedImpulse - previous;

            ApplyImpulse(lambda, ref velocities);

            Vector3Width4.Transpose(ref velocities, out ConnectionA.LinearVelocity, out ConnectionA.AngularVelocity, out ConnectionB.LinearVelocity, out ConnectionB.AngularVelocity);
        }


    }
}
