using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public unsafe struct VectorizedPenetrationConstraint
    {
        //Constraint Descriptions
        public RigidBody* ABodies;
        public RigidBody* BBodies;

        //Per-frame gathered values.

        public Vector4 InverseMassA;
        public Vector4 InverseMassB;
        public Matrix3x3Width4 InverseInertiaTensorA;
        public Matrix3x3Width4 InverseInertiaTensorB;

        public Vector3Width4 ContactPosition;
        //Normal points out of A.
        public Vector3Width4 ContactNormal;
        public Vector4 ContactPenetration;

        //Solver-Computed
        public Vector3Width4 LinearJacobianA;
        public Vector3Width4 LinearJacobianB;
        public Vector3Width4 AngularJacobianA;
        public Vector3Width4 AngularJacobianB;
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
            Vector3Width4 positionA = new Vector3Width4(ref ABodies[0].Position, ref ABodies[1].Position, ref ABodies[2].Position, ref ABodies[3].Position);
            Vector3Width4 positionB = new Vector3Width4(ref BBodies[0].Position, ref BBodies[1].Position, ref BBodies[2].Position, ref BBodies[3].Position);


            InverseMassA = new Vector4(ABodies[0].InverseMass, ABodies[1].InverseMass, ABodies[2].InverseMass, ABodies[3].InverseMass);
            InverseMassB = new Vector4(BBodies[0].InverseMass, BBodies[1].InverseMass, BBodies[2].InverseMass, BBodies[3].InverseMass);


            InverseInertiaTensorA = new Matrix3x3Width4(ref ABodies[0].InertiaTensorInverse, ref ABodies[1].InertiaTensorInverse, ref ABodies[2].InertiaTensorInverse, ref ABodies[3].InertiaTensorInverse);
            InverseInertiaTensorB = new Matrix3x3Width4(ref BBodies[0].InertiaTensorInverse, ref BBodies[1].InertiaTensorInverse, ref BBodies[2].InertiaTensorInverse, ref BBodies[3].InertiaTensorInverse);

            LinearJacobianA = ContactNormal;
            Vector3Width4.Negate(ref ContactNormal, out LinearJacobianB);

            Vector3Width4 offsetA, offsetB;
            Vector3Width4.Subtract(ref ContactPosition, ref positionA, out offsetA);
            Vector3Width4.Subtract(ref ContactPosition, ref positionB, out offsetB);

            Vector3Width4.Cross(ref offsetA, ref ContactNormal, out AngularJacobianA);
            Vector3Width4.Cross(ref ContactNormal, ref offsetB, out AngularJacobianB);// note negation->parameter reverse


            //Allow velocity that closes a gap, and apply penetration correction against positive depth.
            //Bounciness not yet included.
            PenetrationBias = ContactPenetration * inverseDt;
            PenetrationBias = -Vector4.Min(Vector4.Min(PenetrationBias, PenetrationBias * 0.2f), new Vector4(0.2f));


            //The inertia tensor is in world space, so no jacobian transformation is required.
            Vector3Width4 angularA, angularB;
            Matrix3x3Width4.Transform(ref AngularJacobianA, ref InverseInertiaTensorA, out angularA);
            Matrix3x3Width4.Transform(ref AngularJacobianB, ref InverseInertiaTensorB, out angularB);
            Vector4 angularContributionA, angularContributionB;
            Vector3Width4.Dot(ref angularA, ref angularA, out angularContributionA);
            Vector3Width4.Dot(ref angularB, ref angularB, out angularContributionB);
            var inverseEffectiveMass = InverseMassA + InverseMassB + angularContributionA + angularContributionB;

            Vector4 CollisionSoftness = new Vector4(5);
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            EffectiveMass = Vector4.One / (Softness + inverseEffectiveMass);


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(ref Vector4 lambda)
        {
            var linearScaleChangeA = lambda * InverseMassA;
            var linearScaleChangeB = lambda * InverseMassB;
            Vector3Width4 linearChangeA, linearChangeB;
            Vector3Width4.Multiply(ref LinearJacobianA, ref linearScaleChangeA, out linearChangeA);
            Vector3Width4.Multiply(ref LinearJacobianB, ref linearScaleChangeB, out linearChangeB);

            Vector3 linearChangeA0, linearChangeA1, linearChangeA2, linearChangeA3;
            Vector3 linearChangeB0, linearChangeB1, linearChangeB2, linearChangeB3;
            Vector3Width4.Transpose(ref linearChangeA, out linearChangeA0, out linearChangeA1, out linearChangeA2, out linearChangeA3);
            Vector3Width4.Transpose(ref linearChangeB, out linearChangeB0, out linearChangeB1, out linearChangeB2, out linearChangeB3);


            //World inertia available, so no need for extra transforms.
            Vector3Width4 angularImpulseA, angularImpulseB;
            Vector3Width4.Multiply(ref AngularJacobianA, ref lambda, out angularImpulseA);
            Vector3Width4.Multiply(ref AngularJacobianB, ref lambda, out angularImpulseB);
            Vector3Width4 angularChangeA, angularChangeB;
            Matrix3x3Width4.Transform(ref angularImpulseA, ref InverseInertiaTensorA, out angularChangeA);
            Matrix3x3Width4.Transform(ref angularImpulseB, ref InverseInertiaTensorB, out angularChangeB);


            Vector3 angularChangeA0, angularChangeA1, angularChangeA2, angularChangeA3;
            Vector3 angularChangeB0, angularChangeB1, angularChangeB2, angularChangeB3;
            Vector3Width4.Transpose(ref angularChangeA, out angularChangeA0, out angularChangeA1, out angularChangeA2, out angularChangeA3);
            Vector3Width4.Transpose(ref angularChangeB, out angularChangeB0, out angularChangeB1, out angularChangeB2, out angularChangeB3);

            ABodies[0].LinearVelocity -= linearChangeA0;
            ABodies[0].AngularVelocity -= angularChangeA0;
            ABodies[1].LinearVelocity -= linearChangeA1;
            ABodies[1].AngularVelocity -= angularChangeA1;
            ABodies[2].LinearVelocity -= linearChangeA2;
            ABodies[2].AngularVelocity -= angularChangeA2;
            ABodies[3].LinearVelocity -= linearChangeA3;
            ABodies[3].AngularVelocity -= angularChangeA3;

            BBodies[0].LinearVelocity -= linearChangeB0;
            BBodies[0].AngularVelocity -= angularChangeB0;
            BBodies[1].LinearVelocity -= linearChangeB1;
            BBodies[1].AngularVelocity -= angularChangeB1;
            BBodies[2].LinearVelocity -= linearChangeB2;
            BBodies[2].AngularVelocity -= angularChangeB2;
            BBodies[3].LinearVelocity -= linearChangeB3;
            BBodies[3].AngularVelocity -= angularChangeB3;

        }

        public void WarmStart()
        {
            ApplyImpulse(ref AccumulatedImpulse);
        }



        public void SolveIteration()
        {
            //May be wise to hold pre-transposed from the prestep. Then you could....? asdfasdf
            Vector3Width4 linearVelocityA = new Vector3Width4(ref ABodies[0].LinearVelocity, ref ABodies[1].LinearVelocity, ref ABodies[2].LinearVelocity, ref ABodies[3].LinearVelocity);
            Vector3Width4 angularVelocityA = new Vector3Width4(ref ABodies[0].AngularVelocity, ref ABodies[1].AngularVelocity, ref ABodies[2].AngularVelocity, ref ABodies[3].AngularVelocity);
            Vector3Width4 linearVelocityB = new Vector3Width4(ref BBodies[0].LinearVelocity, ref BBodies[1].LinearVelocity, ref BBodies[2].LinearVelocity, ref BBodies[3].LinearVelocity);
            Vector3Width4 angularVelocityB = new Vector3Width4(ref BBodies[0].AngularVelocity, ref BBodies[1].AngularVelocity, ref BBodies[2].AngularVelocity, ref BBodies[3].AngularVelocity);

            Vector4 linearA, angularA, linearB, angularB;
            Vector3Width4.Dot(ref LinearJacobianA, ref linearVelocityA, out linearA);
            Vector3Width4.Dot(ref AngularJacobianA, ref angularVelocityA, out angularA);
            Vector3Width4.Dot(ref LinearJacobianB, ref linearVelocityB, out linearB);
            Vector3Width4.Dot(ref AngularJacobianB, ref angularVelocityB, out angularB);
            var lambda = EffectiveMass * (linearA + angularA + linearB + angularB + PenetrationBias - AccumulatedImpulse * Softness);

            var previous = AccumulatedImpulse;
            AccumulatedImpulse = Vector4.Max(Vector4.Zero, AccumulatedImpulse + lambda);
            lambda = AccumulatedImpulse - previous;

            ApplyImpulse(ref lambda);
        }
    }
}
