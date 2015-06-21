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
        Vector3 linearA0, angularA0, linearB0, angularB0;
        Vector3 linearA1, angularA1, linearB1, angularB1;
        Vector3 linearA2, angularA2, linearB2, angularB2;
        Vector3 linearA3, angularA3, linearB3, angularB3;
        public Vector4 PenetrationBias;
        public Vector4 Softness;
        public Vector4 EffectiveMass;
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

            linearA0 = ContactNormal;
            linearA1 = ContactNormal;
            linearA2 = ContactNormal;
            linearA3 = ContactNormal;

            linearB0 = -ContactNormal;
            linearB1 = -ContactNormal;
            linearB2 = -ContactNormal;
            linearB3 = -ContactNormal;

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
            Vector3Ex.Cross(ref offsetA0, ref ContactNormal, out angularA0);
            Vector3Ex.Cross(ref offsetA1, ref ContactNormal, out angularA1);
            Vector3Ex.Cross(ref offsetA2, ref ContactNormal, out angularA2);
            Vector3Ex.Cross(ref offsetA3, ref ContactNormal, out angularA3);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB0, out angularB0);// note negation->parameter reverse
            Vector3Ex.Cross(ref ContactNormal, ref offsetB1, out angularB1);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB2, out angularB2);
            Vector3Ex.Cross(ref ContactNormal, ref offsetB3, out angularB3);

            var contactPenetrations = new Vector4(ContactPenetration);

            //Allow velocity that closes a gap, and apply penetration correction against positive depth.
            //Bounciness not yet included.
            PenetrationBias = contactPenetrations * inverseDt;
            PenetrationBias = -Vector4.Min(Vector4.Min(PenetrationBias, PenetrationBias * 0.2f), new Vector4(0.2f));


            //The inertia tensor is in world space, so no jacobian transformation is required.
            Vector3 jA0, jB0;
            Vector3 jA1, jB1;
            Vector3 jA2, jB2;
            Vector3 jA3, jB3;
            Matrix3x3.Transform(ref angularA0, ref a0.InertiaTensorInverse, out jA0);
            Matrix3x3.Transform(ref angularB0, ref b0.InertiaTensorInverse, out jB0);
            Matrix3x3.Transform(ref angularA1, ref a1.InertiaTensorInverse, out jA1);
            Matrix3x3.Transform(ref angularB1, ref b1.InertiaTensorInverse, out jB1);
            Matrix3x3.Transform(ref angularA2, ref a2.InertiaTensorInverse, out jA2);
            Matrix3x3.Transform(ref angularB2, ref b2.InertiaTensorInverse, out jB2);
            Matrix3x3.Transform(ref angularA3, ref a3.InertiaTensorInverse, out jA3);
            Matrix3x3.Transform(ref angularB3, ref b3.InertiaTensorInverse, out jB3);
            Vector4 angularContributionsA = new Vector4(Vector3.Dot(jA0, jA0), Vector3.Dot(jA1, jA1), Vector3.Dot(jA2, jA2), Vector3.Dot(jA3, jA3));
            Vector4 angularContributionsB = new Vector4(Vector3.Dot(jB0, jB0), Vector3.Dot(jB1, jB1), Vector3.Dot(jB2, jB2), Vector3.Dot(jB3, jB3));
            var inverseEffectiveMass = InverseMassA + InverseMassB + angularContributionsA + angularContributionsB;

            Vector4 CollisionSoftness = new Vector4(5);
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            EffectiveMass = Vector4.One / (Softness + inverseEffectiveMass);


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
            var linearA = lambda * InverseMassA;
            var linearB = lambda * InverseMassB;

            a0.LinearVelocity -= linearA.X * linearA0;
            a1.LinearVelocity -= linearA.Y * linearA1;
            a2.LinearVelocity -= linearA.Z * linearA2;
            a3.LinearVelocity -= linearA.W * linearA3;

            b0.LinearVelocity -= linearB.X * linearB0;
            b1.LinearVelocity -= linearB.Y * linearB1;
            b2.LinearVelocity -= linearB.Z * linearB2;
            b3.LinearVelocity -= linearB.W * linearB3;

            var angularImpulseA0 = lambda.X * angularA0;
            var angularImpulseA1 = lambda.Y * angularA1;
            var angularImpulseA2 = lambda.Z * angularA2;
            var angularImpulseA3 = lambda.W * angularA3;

            var angularImpulseB0 = lambda.X * angularB0;
            var angularImpulseB1 = lambda.Y * angularB1;
            var angularImpulseB2 = lambda.Z * angularB2;
            var angularImpulseB3 = lambda.W * angularB3;

            Vector3 velocityChangeA0, velocityChangeA1, velocityChangeA2, velocityChangeA3;
            Matrix3x3.Transform(ref angularImpulseA0, ref a0.InertiaTensorInverse, out velocityChangeA0);
            Matrix3x3.Transform(ref angularImpulseA1, ref a1.InertiaTensorInverse, out velocityChangeA1);
            Matrix3x3.Transform(ref angularImpulseA2, ref a2.InertiaTensorInverse, out velocityChangeA2);
            Matrix3x3.Transform(ref angularImpulseA3, ref a3.InertiaTensorInverse, out velocityChangeA3);

            Vector3 velocityChangeB0, velocityChangeB1, velocityChangeB2, velocityChangeB3;
            Matrix3x3.Transform(ref angularImpulseB0, ref b0.InertiaTensorInverse, out velocityChangeB0);
            Matrix3x3.Transform(ref angularImpulseB1, ref b1.InertiaTensorInverse, out velocityChangeB1);
            Matrix3x3.Transform(ref angularImpulseB2, ref b2.InertiaTensorInverse, out velocityChangeB2);
            Matrix3x3.Transform(ref angularImpulseB3, ref b3.InertiaTensorInverse, out velocityChangeB3);

            a0.AngularVelocity -= velocityChangeA0;
            a1.AngularVelocity -= velocityChangeA1;
            a2.AngularVelocity -= velocityChangeA2;
            a3.AngularVelocity -= velocityChangeA3;

            b0.AngularVelocity -= velocityChangeB0;
            b1.AngularVelocity -= velocityChangeB1;
            b2.AngularVelocity -= velocityChangeB2;
            b3.AngularVelocity -= velocityChangeB3;

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
                Vector3.Dot(linearA0, a0.LinearVelocity),
                Vector3.Dot(linearA1, a1.LinearVelocity),
                Vector3.Dot(linearA2, a2.LinearVelocity),
                Vector3.Dot(linearA3, a3.LinearVelocity));
            var angularA = new Vector4(
                Vector3.Dot(angularA0, a0.AngularVelocity),
                Vector3.Dot(angularA1, a1.AngularVelocity),
                Vector3.Dot(angularA2, a2.AngularVelocity),
                Vector3.Dot(angularA3, a3.AngularVelocity));
            var linearB = new Vector4(
                Vector3.Dot(linearB0, b0.LinearVelocity),
                Vector3.Dot(linearB1, b1.LinearVelocity),
                Vector3.Dot(linearB2, b2.LinearVelocity),
                Vector3.Dot(linearB3, b3.LinearVelocity));
            var angularB = new Vector4(
                Vector3.Dot(angularB0, b0.AngularVelocity),
                Vector3.Dot(angularB1, b1.AngularVelocity),
                Vector3.Dot(angularB2, b2.AngularVelocity),
                Vector3.Dot(angularB3, b3.AngularVelocity));

            var lambda = EffectiveMass * (
                linearA + linearB + angularA + angularB +
                PenetrationBias - AccumulatedImpulse * Softness);

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
