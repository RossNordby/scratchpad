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

    public unsafe struct VectorizedManifoldConstraint
    {
        public Velocities VelocitiesA0;
        public Velocities VelocitiesA1;
        public Velocities VelocitiesA2;
        public Velocities VelocitiesA3;

        public BodyState BodyA0;
        public BodyState BodyA1;
        public BodyState BodyA2;
        public BodyState BodyA3;

        public Velocities VelocitiesB0;
        public Velocities VelocitiesB1;
        public Velocities VelocitiesB2;
        public Velocities VelocitiesB3;

        public BodyState BodyB0;
        public BodyState BodyB1;
        public BodyState BodyB2;
        public BodyState BodyB3;

        public VectorizedManifoldPenetrationConstraint a;
        public VectorizedManifoldPenetrationConstraint b;
        public VectorizedManifoldPenetrationConstraint c;
        public VectorizedManifoldPenetrationConstraint d;
        //public VectorizedManifoldPenetrationConstraint Constraints;

        public void Prestep(float inverseDt)
        {
            //Given that we're collecting position, inverse mass, and inertia all at once, it makes no sense to store position separately from inversemass and inertia. 
            //Since you should not expect the 4 involved bodies to be in memory *together*, the best you can do is to ensure that the set of values are together. 
            //Otherwise you're multiplying cache misses for no reason!
            var InverseMassA = new Vector4(BodyA0.InverseMass, BodyA1.InverseMass, BodyA2.InverseMass, BodyA3.InverseMass);
            var InverseMassB = new Vector4(BodyB0.InverseMass, BodyB1.InverseMass, BodyB2.InverseMass, BodyB3.InverseMass);

            var InverseInertiaTensorA = new Matrix3x3Width4(ref BodyA0.InertiaTensorInverse, ref BodyA1.InertiaTensorInverse, ref BodyA2.InertiaTensorInverse, ref BodyA3.InertiaTensorInverse);
            var InverseInertiaTensorB = new Matrix3x3Width4(ref BodyB0.InertiaTensorInverse, ref BodyB1.InertiaTensorInverse, ref BodyB2.InertiaTensorInverse, ref BodyB3.InertiaTensorInverse);

            Vector3Width4 positionA = new Vector3Width4(ref BodyA0.Position, ref BodyA1.Position, ref BodyA2.Position, ref BodyA3.Position);
            Vector3Width4 positionB = new Vector3Width4(ref BodyA0.Position, ref BodyB1.Position, ref BodyB2.Position, ref BodyB3.Position);

            //for (int i = 0; i < 4; ++i)
            //{
            //    Constraints[i].Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);
            //}

            for (int i = 0; i < 4; ++i)
            {
                a.Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);

            }

            //a.Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);
            //b.Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);
            //c.Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);
            //d.Prestep(inverseDt, ref InverseMassA, ref InverseMassB, ref InverseInertiaTensorA, ref InverseInertiaTensorB, ref positionA, ref positionB);
        }

        public void WarmStart()
        {
            Vector3Width4 linearVelocityA = new Vector3Width4(ref VelocitiesA0.LinearVelocity, ref VelocitiesA1.LinearVelocity, ref VelocitiesA2.LinearVelocity, ref VelocitiesA3.LinearVelocity);
            Vector3Width4 angularVelocityA = new Vector3Width4(ref VelocitiesA0.AngularVelocity, ref VelocitiesA1.AngularVelocity, ref VelocitiesA2.AngularVelocity, ref VelocitiesA3.AngularVelocity);
            Vector3Width4 linearVelocityB = new Vector3Width4(ref VelocitiesB0.LinearVelocity, ref VelocitiesB1.LinearVelocity, ref VelocitiesB2.LinearVelocity, ref VelocitiesB3.LinearVelocity);
            Vector3Width4 angularVelocityB = new Vector3Width4(ref VelocitiesB0.AngularVelocity, ref VelocitiesB1.AngularVelocity, ref VelocitiesB2.AngularVelocity, ref VelocitiesB3.AngularVelocity);

            //for (int i = 0; i < 4; ++i)
            //{
            //    Constraints[i].WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //}
            for (int i = 0; i < 4; ++i)
            {
                a.WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            }
            //a.WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //b.WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //c.WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //d.WarmStart(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);

            Vector3Width4.Transpose(ref linearVelocityA, out VelocitiesA0.LinearVelocity, out VelocitiesA1.LinearVelocity, out VelocitiesA2.LinearVelocity, out VelocitiesA3.LinearVelocity);
            Vector3Width4.Transpose(ref linearVelocityB, out VelocitiesB0.LinearVelocity, out VelocitiesB1.LinearVelocity, out VelocitiesB2.LinearVelocity, out VelocitiesB3.LinearVelocity);
            Vector3Width4.Transpose(ref angularVelocityA, out VelocitiesA0.AngularVelocity, out VelocitiesA1.AngularVelocity, out VelocitiesA2.AngularVelocity, out VelocitiesA3.AngularVelocity);
            Vector3Width4.Transpose(ref angularVelocityB, out VelocitiesB0.AngularVelocity, out VelocitiesB1.AngularVelocity, out VelocitiesB2.AngularVelocity, out VelocitiesB3.AngularVelocity);

        }

        public void SolveIteration()
        {

            Vector3Width4 linearVelocityA = new Vector3Width4(ref VelocitiesA0.LinearVelocity, ref VelocitiesA1.LinearVelocity, ref VelocitiesA2.LinearVelocity, ref VelocitiesA3.LinearVelocity);
            Vector3Width4 angularVelocityA = new Vector3Width4(ref VelocitiesA0.AngularVelocity, ref VelocitiesA1.AngularVelocity, ref VelocitiesA2.AngularVelocity, ref VelocitiesA3.AngularVelocity);
            Vector3Width4 linearVelocityB = new Vector3Width4(ref VelocitiesB0.LinearVelocity, ref VelocitiesB1.LinearVelocity, ref VelocitiesB2.LinearVelocity, ref VelocitiesB3.LinearVelocity);
            Vector3Width4 angularVelocityB = new Vector3Width4(ref VelocitiesB0.AngularVelocity, ref VelocitiesB1.AngularVelocity, ref VelocitiesB2.AngularVelocity, ref VelocitiesB3.AngularVelocity);

            //for (int i = 0; i < 4; ++i)
            //{
            //    Constraints[i].SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //}

            for (int i = 0; i < 4; ++i)
            {
                a.SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            }

            //a.SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //b.SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //c.SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
            //d.SolveIteration(ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);


            Vector3Width4.Transpose(ref linearVelocityA, out VelocitiesA0.LinearVelocity, out VelocitiesA1.LinearVelocity, out VelocitiesA2.LinearVelocity, out VelocitiesA3.LinearVelocity);
            Vector3Width4.Transpose(ref linearVelocityB, out VelocitiesB0.LinearVelocity, out VelocitiesB1.LinearVelocity, out VelocitiesB2.LinearVelocity, out VelocitiesB3.LinearVelocity);
            Vector3Width4.Transpose(ref angularVelocityA, out VelocitiesA0.AngularVelocity, out VelocitiesA1.AngularVelocity, out VelocitiesA2.AngularVelocity, out VelocitiesA3.AngularVelocity);
            Vector3Width4.Transpose(ref angularVelocityB, out VelocitiesB0.AngularVelocity, out VelocitiesB1.AngularVelocity, out VelocitiesB2.AngularVelocity, out VelocitiesB3.AngularVelocity);

        }
    }
    public unsafe struct VectorizedManifoldPenetrationConstraint
    {

        //Per-frame gathered values.

        public Vector3Width4 ContactPosition;
        //Normal points out of A.
        public Vector3Width4 ContactNormal;
        public Vector4 ContactPenetration;

        //Solver-Computed
        public Vector3Width4 LinearJacobianA;
        public Vector3Width4 LinearJacobianB;
        public Vector3Width4 AngularJacobianA;
        public Vector3Width4 AngularJacobianB;
        public Vector3Width4 LinearJacobianITA;
        public Vector3Width4 LinearJacobianITB;
        public Vector3Width4 AngularJacobianITA;
        public Vector3Width4 AngularJacobianITB;
        public Vector4 PenetrationBias;
        public Vector4 Softness;
        public Vector4 EffectiveMass;

        public Vector4 AccumulatedImpulse;



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(float inverseDt,
            ref Vector4 inverseMassA, ref Vector4 inverseMassB, 
            ref Matrix3x3Width4 inverseInertiaTensorA, ref Matrix3x3Width4 inverseInertiaTensorB,
            ref Vector3Width4 positionA, ref Vector3Width4 positionB)
        {
            //C = dot(Pa - Pb, N) > 0
            //Jacobians:
            //LinearA: N
            //AngularA: cross(OffsetPa, N) 
            //LinearB: -N
            //AngularB: -cross(OffsetPb, N)

            //var positionA = new Vector3Width4();
            //var positionB = new Vector3Width4();
            
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
            Vector3Width4.Multiply(ref LinearJacobianA, ref inverseMassA, out LinearJacobianITA);
            Vector3Width4.Multiply(ref LinearJacobianB, ref inverseMassB, out LinearJacobianITB);
            Matrix3x3Width4.Transform(ref AngularJacobianA, ref inverseInertiaTensorA, out AngularJacobianITA);
            Matrix3x3Width4.Transform(ref AngularJacobianB, ref inverseInertiaTensorB, out AngularJacobianITB);
            Vector4 angularContributionA, angularContributionB;
            Vector3Width4.Dot(ref AngularJacobianITA, ref AngularJacobianITA, out angularContributionA);
            Vector3Width4.Dot(ref AngularJacobianITB, ref AngularJacobianITB, out angularContributionB);
            var inverseEffectiveMass = inverseMassA + inverseMassB + angularContributionA + angularContributionB;

            Vector4 CollisionSoftness = new Vector4(5);
            Softness = CollisionSoftness * inverseEffectiveMass * inverseDt;
            EffectiveMass = Vector4.One / (Softness + inverseEffectiveMass);


        }




        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ApplyImpulse(ref Vector4 lambda, ref Vector3Width4 linearVelocityA, ref Vector3Width4 angularVelocityA, ref Vector3Width4 linearVelocityB, ref Vector3Width4 angularVelocityB)
        {
            Vector3Width4 linearChangeA, linearChangeB;
            Vector3Width4.Multiply(ref LinearJacobianITA, ref lambda, out linearChangeA);
            Vector3Width4.Multiply(ref LinearJacobianITB, ref lambda, out linearChangeB);

            Vector3Width4.Add(ref linearVelocityA, ref linearChangeA, out linearVelocityA);
            Vector3Width4.Add(ref linearVelocityB, ref linearChangeB, out linearVelocityB);



            //World inertia available, so no need for extra transforms.
            Vector3Width4 angularChangeA, angularChangeB;
            Vector3Width4.Multiply(ref AngularJacobianITA, ref lambda, out angularChangeA);
            Vector3Width4.Multiply(ref AngularJacobianITB, ref lambda, out angularChangeB);


            Vector3Width4.Add(ref angularVelocityA, ref angularChangeA, out angularVelocityA);
            Vector3Width4.Add(ref angularVelocityB, ref angularChangeB, out angularVelocityB);




        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref Vector3Width4 linearVelocityA, ref Vector3Width4 angularVelocityA, ref Vector3Width4 linearVelocityB, ref Vector3Width4 angularVelocityB)
        {
            ApplyImpulse(ref AccumulatedImpulse, ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveIteration(ref Vector3Width4 linearVelocityA, ref Vector3Width4 angularVelocityA, ref Vector3Width4 linearVelocityB, ref Vector3Width4 angularVelocityB)
        {
            Vector4 linearA, angularA, linearB, angularB;
            Vector3Width4.Dot(ref LinearJacobianA, ref linearVelocityA, out linearA);
            Vector3Width4.Dot(ref AngularJacobianA, ref angularVelocityA, out angularA);
            Vector3Width4.Dot(ref LinearJacobianB, ref linearVelocityB, out linearB);
            Vector3Width4.Dot(ref AngularJacobianB, ref angularVelocityB, out angularB);
            var lambda = EffectiveMass * (linearA + angularA + linearB + angularB + PenetrationBias - AccumulatedImpulse * Softness);

            var previous = AccumulatedImpulse;
            AccumulatedImpulse = Vector4.Max(Vector4.Zero, AccumulatedImpulse + lambda);
            lambda = AccumulatedImpulse - previous;

            ApplyImpulse(ref lambda, ref linearVelocityA, ref angularVelocityA, ref linearVelocityB, ref angularVelocityB);

        }
    }
}
