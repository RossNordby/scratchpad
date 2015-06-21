using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{

    public unsafe struct HybridPenetrationConstraint
    {
        //Constraint Descriptions
        public RigidBody a0, b0, a1, b1, a2, b2, a3, b3;

        //Per-frame gathered values.

        public Vector4 InverseMassA;
        public Vector4 InverseMassB;

        //Just share contact information for now.
        public Vector3 ContactPosition;
        //Normal points out of A.
        public Vector3 ContactNormal;
        public float ContactPenetration;

        //Solver-Computed
        //Jacobians
        //"Effective mass" transformed jacobians. Used for computing relative velocity.
        Vector3 linearEMA0, angularEMA0, linearEMB0, angularEMB0;
        Vector3 linearEMA1, angularEMA1, linearEMB1, angularEMB1;
        Vector3 linearEMA2, angularEMA2, linearEMB2, angularEMB2;
        Vector3 linearEMA3, angularEMA3, linearEMB3, angularEMB3;

        //"Independently transformed" jacobians. These are jl * invMass and ja * invInertia. 
        Vector3 linearITA0, angularITA0, linearITB0, angularITB0;
        Vector3 linearITA1, angularITA1, linearITB1, angularITB1;
        Vector3 linearITA2, angularITA2, linearITB2, angularITB2;
        Vector3 linearITA3, angularITA3, linearITB3, angularITB3;
        public Vector4 PenetrationBias;
        public Vector4 Softness;
        public Vector4 AccumulatedImpulse;




        public void Prestep(float inverseDt)
        {
            //C = dot(Pa - Pb, N) > 0
            //Jacobians:
            //LinearA: N
            //AngularA: cross(OffsetPa, N) 
            //LinearB: -N
            //AngularB: -cross(OffsetPb, N)

            //var positionA = new Vector3Width4();
            //var positionB = new Vector3Width4();

            //Given that we're collecting position, inverse mass, and inertia all at once, it makes no sense to store position separately from inversemass and inertia. 
            //Since you should not expect the 4 involved bodies to be in memory *together*, the best you can do is to ensure that the set of values are together. 
            //Otherwise you're multiplying cache misses for no reason!
            InverseMassA = new Vector4(a0.InverseMass, a1.InverseMass, a2.InverseMass, a3.InverseMass);
            InverseMassB = new Vector4(b0.InverseMass, b1.InverseMass, b2.InverseMass, b3.InverseMass);

            linearEMA0 = ContactNormal;
            linearEMA1 = ContactNormal;
            linearEMA2 = ContactNormal;
            linearEMA3 = ContactNormal;

            linearEMB0 = -ContactNormal;
            linearEMB1 = -ContactNormal;
            linearEMB2 = -ContactNormal;
            linearEMB3 = -ContactNormal;

            Vector3 offsetA0, offsetB0, offsetA1, offsetB1, offsetA2, offsetB2, offsetA3, offsetB3;
            offsetA0 = ContactPosition - a0.Position;
            offsetA1 = ContactPosition - a1.Position;
            offsetA2 = ContactPosition - a2.Position;
            offsetA3 = ContactPosition - a3.Position;
            offsetB0 = ContactPosition - b0.Position;
            offsetB1 = ContactPosition - b1.Position;
            offsetB2 = ContactPosition - b2.Position;
            offsetB3 = ContactPosition - b3.Position;

            //Oof. All of this is scalar.
            Vector3Ex.Cross(ref offsetA0, ref ContactNormal, out angularEMA0);
            Vector3Ex.Cross(ref offsetA1, ref ContactNormal, out angularEMA1);
            Vector3Ex.Cross(ref offsetA2, ref ContactNormal, out angularEMA2);
            Vector3Ex.Cross(ref offsetA3, ref ContactNormal, out angularEMA3);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB0, out angularEMB0);// note negation->parameter reverse
            Vector3Ex.Cross(ref ContactNormal, ref offsetB1, out angularEMB1);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB2, out angularEMB2);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB3, out angularEMB3);

            var contactPenetrations = new Vector4(ContactPenetration);

            //Allow velocity that closes a gap, and apply penetration correction against positive depth.
            //Bounciness not yet included.
            PenetrationBias = contactPenetrations * inverseDt;
            PenetrationBias = -Vector4.Min(Vector4.Min(PenetrationBias, PenetrationBias * 0.2f), new Vector4(0.2f));


            linearITA0 = linearEMA0 * a0.InverseMass;
            linearITA1 = linearEMA1 * a1.InverseMass;
            linearITA2 = linearEMA2 * a2.InverseMass;
            linearITA3 = linearEMA3 * a3.InverseMass;

            linearITB0 = linearEMB0 * b0.InverseMass;
            linearITB1 = linearEMB1 * b1.InverseMass;
            linearITB2 = linearEMB2 * b2.InverseMass;
            linearITB3 = linearEMB3 * b3.InverseMass;

            //The inertia tensor is in world space, so no jacobian transformation is required.
            Matrix3x3.Transform(ref angularEMA0, ref a0.InertiaTensorInverse, out angularITA0);
            Matrix3x3.Transform(ref angularEMB0, ref b0.InertiaTensorInverse, out angularITB0);
            Matrix3x3.Transform(ref angularEMA1, ref a1.InertiaTensorInverse, out angularITA1);
            Matrix3x3.Transform(ref angularEMB1, ref b1.InertiaTensorInverse, out angularITB1);
            Matrix3x3.Transform(ref angularEMA2, ref a2.InertiaTensorInverse, out angularITA2);
            Matrix3x3.Transform(ref angularEMB2, ref b2.InertiaTensorInverse, out angularITB2);
            Matrix3x3.Transform(ref angularEMA3, ref a3.InertiaTensorInverse, out angularITA3);
            Matrix3x3.Transform(ref angularEMB3, ref b3.InertiaTensorInverse, out angularITB3);
            Vector4 angularContributionsA = new Vector4(Vector3.Dot(angularITA0, angularITA0), Vector3.Dot(angularITA1, angularITA1), Vector3.Dot(angularITA2, angularITA2), Vector3.Dot(angularITA3, angularITA3));
            Vector4 angularContributionsB = new Vector4(Vector3.Dot(angularITB0, angularITB0), Vector3.Dot(angularITB1, angularITB1), Vector3.Dot(angularITB2, angularITB2), Vector3.Dot(angularITB3, angularITB3));
            var inverseEffectiveMass = InverseMassA + InverseMassB + angularContributionsA + angularContributionsB;

            Vector4 CollisionSoftness = new Vector4(5);
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            var EffectiveMass = Vector4.One / (Softness + inverseEffectiveMass);

            angularEMA0 *= EffectiveMass.X;
            angularEMB0 *= EffectiveMass.X;
            angularEMA1 *= EffectiveMass.Y;
            angularEMB1 *= EffectiveMass.Y;
            angularEMA2 *= EffectiveMass.Z;
            angularEMB2 *= EffectiveMass.Z;
            angularEMA3 *= EffectiveMass.W;
            angularEMB3 *= EffectiveMass.W;

            PenetrationBias *= EffectiveMass;
            Softness *= EffectiveMass;


        }



        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(ref RigidBody a, ref RigidBody b, ref Vector3 linearA, ref Vector3 angularA, ref Vector3 linearB, ref Vector3 angularB, float lambda)
        {
            a.LinearVelocity -= (lambda * a.InverseMass) * linearA;
            b.LinearVelocity -= (lambda * b.InverseMass) * linearB;

            //World inertia available, so no need for extra transforms.
            Vector3 angularImpulseA = lambda * angularA;
            Vector3 angularImpulseB = lambda * angularB;
            Vector3 velocityChangeA, velocityChangeB;
            Matrix3x3.Transform(ref angularImpulseA, ref a.InertiaTensorInverse, out velocityChangeA);
            Matrix3x3.Transform(ref angularImpulseB, ref b.InertiaTensorInverse, out velocityChangeB);
            a.AngularVelocity -= velocityChangeA;
            b.AngularVelocity -= velocityChangeB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(ref Vector4 lambda)
        {
            a0.LinearVelocity -= lambda.X * linearITA0;
            a1.LinearVelocity -= lambda.Y * linearITA1;
            a2.LinearVelocity -= lambda.Z * linearITA2;
            a3.LinearVelocity -= lambda.W * linearITA3;

            b0.LinearVelocity -= lambda.X * linearITB0;
            b1.LinearVelocity -= lambda.Y * linearITB1;
            b2.LinearVelocity -= lambda.Z * linearITB2;
            b3.LinearVelocity -= lambda.W * linearITB3;

            a0.AngularVelocity -= lambda.X * angularITB0;
            a1.AngularVelocity -= lambda.Y * angularITB1;
            a2.AngularVelocity -= lambda.Z * angularITB2;
            a3.AngularVelocity -= lambda.W * angularITB3;

            b0.AngularVelocity -= lambda.X * angularITB0;
            b1.AngularVelocity -= lambda.Y * angularITB1;
            b2.AngularVelocity -= lambda.Z * angularITB2;
            b3.AngularVelocity -= lambda.W * angularITB3;

        }

        public void WarmStart()
        {
            ApplyImpulse(ref AccumulatedImpulse);
            //ApplyImpulse(ref a0, ref b0, ref linearA0, ref angularA0, ref linearB0, ref angularB0, AccumulatedImpulse.X);
            //ApplyImpulse(ref a1, ref b1, ref linearA1, ref angularA1, ref linearB1, ref angularB1, AccumulatedImpulse.Y);
            //ApplyImpulse(ref a2, ref b2, ref linearA2, ref angularA2, ref linearB2, ref angularB2, AccumulatedImpulse.Z);
            //ApplyImpulse(ref a3, ref b3, ref linearA3, ref angularA3, ref linearB3, ref angularB3, AccumulatedImpulse.W);
        }




        public void SolveIteration()
        {
            var linearA = new Vector4(
                Vector3.Dot(linearEMA0, a0.LinearVelocity),
                Vector3.Dot(linearEMA1, a1.LinearVelocity),
                Vector3.Dot(linearEMA2, a2.LinearVelocity),
                Vector3.Dot(linearEMA3, a3.LinearVelocity));
            var angularA = new Vector4(
                Vector3.Dot(angularEMA0, a0.AngularVelocity),
                Vector3.Dot(angularEMA1, a1.AngularVelocity),
                Vector3.Dot(angularEMA2, a2.AngularVelocity),
                Vector3.Dot(angularEMA3, a3.AngularVelocity));
            var linearB = new Vector4(
                Vector3.Dot(linearEMB0, b0.LinearVelocity),
                Vector3.Dot(linearEMB1, b1.LinearVelocity),
                Vector3.Dot(linearEMB2, b2.LinearVelocity),
                Vector3.Dot(linearEMB3, b3.LinearVelocity));
            var angularB = new Vector4(
                Vector3.Dot(angularEMB0, b0.AngularVelocity),
                Vector3.Dot(angularEMB1, b1.AngularVelocity),
                Vector3.Dot(angularEMB2, b2.AngularVelocity),
                Vector3.Dot(angularEMB3, b3.AngularVelocity));

            var lambda =
                linearA + linearB + angularA + angularB +
                PenetrationBias - AccumulatedImpulse * Softness;

            var previous = AccumulatedImpulse;
            AccumulatedImpulse = Vector4.Max(Vector4.Zero, AccumulatedImpulse + lambda);
            lambda = AccumulatedImpulse - previous;

            ApplyImpulse(ref lambda);

            //ApplyImpulse(ref a0, ref b0, ref linearA0, ref angularA0, ref linearB0, ref angularB0, lambda.X);
            //ApplyImpulse(ref a1, ref b1, ref linearA1, ref angularA1, ref linearB1, ref angularB1, lambda.Y);
            //ApplyImpulse(ref a2, ref b2, ref linearA2, ref angularA2, ref linearB2, ref angularB2, lambda.Z);
            //ApplyImpulse(ref a3, ref b3, ref linearA3, ref angularA3, ref linearB3, ref angularB3, lambda.W);
        }
    }
}
