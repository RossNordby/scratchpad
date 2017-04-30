using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public static class LocalsinitCodegen
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void ApplyImpulseBaseline(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //no locals initialization
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(ref angularA, ref correctiveImpulse, out var correctiveAngularImpulseA);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseA, ref inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Scale(ref angularB, ref correctiveImpulse, out var correctiveAngularImpulseB);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseB, ref inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);

            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityALinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityAAngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveVelocityBLinearVelocity, out wsvB.LinearVelocity); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveVelocityBAngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void ApplyImpulseTest(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //no locals initialization
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityALinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Scale(ref angularA, ref correctiveImpulse, out var correctiveAngularImpulseA);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseA, ref inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityAAngularVelocity, out wsvA.AngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveVelocityBLinearVelocity, out wsvB.LinearVelocity); //Note subtract; normal = -jacobianLinearB

            Vector3Wide.Scale(ref angularB, ref correctiveImpulse, out var correctiveAngularImpulseB);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseB, ref inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);
            Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveVelocityBAngularVelocity, out wsvB.AngularVelocity);
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void MicroTest(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //no locals initialization
            var tempX = angularA.X + wsvA.LinearVelocity.X;
            var tempY = angularA.Y + wsvA.LinearVelocity.Y;
            var tempZ = angularA.Z + wsvA.LinearVelocity.Z;
            wsvB.AngularVelocity.X = tempX;
            wsvB.AngularVelocity.Y = tempY;
            wsvB.AngularVelocity.Z = tempZ;
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void MicroTest2(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //no locals initialization
            Vector3Wide.Add(ref angularA, ref wsvA.LinearVelocity, out var temp);
            wsvB.AngularVelocity = temp;
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void MicroTest3(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //no locals initialization
            Vector3Wide.Add(ref angularA, ref wsvA.LinearVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TangentPrestepBaseline(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            out TangentFrictionProjection projection)
        {
            TangentFriction.ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, ref offsetB, out var jacobians);
            //Compute effective mass matrix contributions.
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaA.InverseMass, out var linearIntermediateA);
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaB.InverseMass, out var linearIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateA, ref jacobians.LinearA, out var linearContributionA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateB, ref jacobians.LinearA, out var linearContributionB);

            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out var angularIntermediateA);
            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out var angularIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateA, ref jacobians.AngularA, out var angularContributionA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateB, ref jacobians.AngularB, out var angularContributionB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Matrix2x2Wide.Add(ref linearContributionA, ref linearContributionB, out var linear);
            Matrix2x2Wide.Subtract(ref angularContributionA, ref angularContributionB, out var angular);
            Matrix2x2Wide.Add(ref linear, ref angular, out var inverseEffectiveMass);
            Matrix2x2Wide.InvertWithoutOverlap(ref inverseEffectiveMass, out projection.EffectiveMass);
            projection.OffsetA = offsetA;
            projection.OffsetB = offsetB;

            //Note that friction constraints have no bias velocity. They target zero velocity.
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TangentPrestepTest(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            out TangentFrictionProjection projection)
        {
            TangentFriction.ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, ref offsetB, out var jacobians);
            //Compute effective mass matrix contributions.
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaA.InverseMass, out var linearIntermediateA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateA, ref jacobians.LinearA, out var linearContributionA);
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaB.InverseMass, out var linearIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateB, ref jacobians.LinearA, out var linearContributionB);
            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Matrix2x2Wide.Add(ref linearContributionA, ref linearContributionB, out var linear);

            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out var angularIntermediateA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateA, ref jacobians.AngularA, out var angularContributionA);
            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out var angularIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateB, ref jacobians.AngularB, out var angularContributionB);
            Matrix2x2Wide.Subtract(ref angularContributionA, ref angularContributionB, out var angular);

            Matrix2x2Wide.Add(ref linear, ref angular, out var inverseEffectiveMass);
            Matrix2x2Wide.InvertWithoutOverlap(ref inverseEffectiveMass, out projection.EffectiveMass);
            projection.OffsetA = offsetA;
            projection.OffsetB = offsetB;

            //Note that friction constraints have no bias velocity. They target zero velocity.
        }

        public struct InnerStruct
        {
            public Vector<float> X;
            public Vector<float> Y;

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Scale(ref InnerStruct v, ref Vector<float> s, out InnerStruct result)
            {
                result.X = v.X * s;
                result.Y = v.Y * s;
            }
        }
        public struct OuterStruct
        {
            public InnerStruct X;
            public InnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
            {
                result.X.X = m.X.X * s;
                result.X.Y = m.X.Y * s;
                result.Y.X = m.Y.X * s;
                result.Y.Y = m.Y.Y * s;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void ScaleSubcalls(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
            {
                InnerStruct.Scale(ref m.X, ref s, out result.X);
                InnerStruct.Scale(ref m.Y, ref s, out result.Y);
            }
        }

        public struct FlattenedStruct
        {
            public Vector<float> XX;
            public Vector<float> XY;
            public Vector<float> YX;
            public Vector<float> YY;

            //public ref InnerStruct X { get { return ref Unsafe.As<Vector<float>, InnerStruct>(ref XX); } }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ref InnerStruct GetX(ref FlattenedStruct m)
            {
                return ref Unsafe.As<Vector<float>, InnerStruct>(ref m.XX);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ref InnerStruct GetY(ref FlattenedStruct m)
            {
                return ref Unsafe.As<Vector<float>, InnerStruct>(ref m.YX);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
            {
                result.XX = m.XX * s;
                result.XY = m.XY * s;
                result.YX = m.YX * s;
                result.YY = m.YY * s;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void ScaleRenested(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
            {
                ref var x = ref GetX(ref m);
                result.XX = x.X * s;
                result.XY = x.Y * s;
                ref var y = ref GetY(ref m);
                result.YX = y.X * s;
                result.YY = y.Y * s;
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void NestedStructTest(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes)
            OuterStruct.Scale(ref m, ref s, out var intermediate);
            OuterStruct.Scale(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void SubcallsNestedStructTest(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes), with or without inlining on the interior subcall
            OuterStruct.ScaleSubcalls(ref m, ref s, out var intermediate);
            OuterStruct.ScaleSubcalls(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void FlattenedStructTest(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
        {
            //no locals initialization
            FlattenedStruct.Scale(ref m, ref s, out var intermediate);
            FlattenedStruct.Scale(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void RenestedStructTest(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
        {
            //rep stos, ecx 8 (32 bytes)
            FlattenedStruct.ScaleRenested(ref m, ref s, out var intermediate);
            FlattenedStruct.ScaleRenested(ref intermediate, ref s, out result);
        }


        public struct InnerStructForDummy
        {
            public Vector<float> XX;
            public Vector<float> XY;
            public Vector<float> YX;
            public Vector<float> YY;
        }
        public struct DummyOuterStruct
        {
            public InnerStructForDummy Inner;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref DummyOuterStruct m, ref Vector<float> s, out DummyOuterStruct result)
            {
                result.Inner.XX = m.Inner.XX * s;
                result.Inner.XY = m.Inner.XY * s;
                result.Inner.YX = m.Inner.YX * s;
                result.Inner.YY = m.Inner.YY * s;
            }
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void DummyNestedStructTest(ref DummyOuterStruct m, ref Vector<float> s, out DummyOuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes)
            DummyOuterStruct.Scale(ref m, ref s, out var intermediate);
            DummyOuterStruct.Scale(ref intermediate, ref s, out result);
        }


        public struct ScalarInnerStruct
        {
            public float X;
            public float Y;
        }
        public struct ScalarOuterStruct
        {
            public ScalarInnerStruct X;
            public ScalarInnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref ScalarOuterStruct m, float s, out ScalarOuterStruct result)
            {
                result.X.X = m.X.X * s;
                result.X.Y = m.X.Y * s;
                result.Y.X = m.Y.X * s;
                result.Y.Y = m.Y.Y * s;
            }
        }

        public struct ScalarFlattenedStruct
        {
            public float XX;
            public float XY;
            public float YX;
            public float YY;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
            {
                result.XX = m.XX * s;
                result.XY = m.XY * s;
                result.YX = m.YX * s;
                result.YY = m.YY * s;
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void ScalarNestedStructTest(ref ScalarOuterStruct m, float s, out ScalarOuterStruct result)
        {
            //rep stos, ecx 4 (16 bytes)
            ScalarOuterStruct.Scale(ref m, s, out var intermediate);
            ScalarOuterStruct.Scale(ref intermediate, s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void ScalarFlattenedStructTest(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
        {
            //no locals initialization
            ScalarFlattenedStruct.Scale(ref m, s, out var intermediate);
            ScalarFlattenedStruct.Scale(ref intermediate, s, out result);
        }



        public static void Test()
        {
            var angularA = new Vector3Wide();
            var angularB = new Vector3Wide();
            var bodyInertiasA = new BodyInertias();
            var bodyInertiasB = new BodyInertias();
            var normal = new Vector3Wide();
            var correctiveImpulse = new Vector<float>();
            var wsvA = new BodyVelocities();
            var wsvB = new BodyVelocities();

            ApplyImpulseBaseline(ref angularA, ref angularB, ref bodyInertiasA, ref bodyInertiasB, ref normal, ref correctiveImpulse, ref wsvA, ref wsvB);
            ApplyImpulseTest(ref angularA, ref angularB, ref bodyInertiasA, ref bodyInertiasB, ref normal, ref correctiveImpulse, ref wsvA, ref wsvB);

            MicroTest(ref angularA, ref angularB, ref wsvA, ref wsvB);
            MicroTest2(ref angularA, ref angularB, ref wsvA, ref wsvB);
            MicroTest3(ref angularA, ref angularB, ref wsvA, ref wsvB);
            {
                var tangentX = new Vector3Wide();
                var tangentY = new Vector3Wide();
                var offsetA = new Vector3Wide();
                var offsetB = new Vector3Wide();
                Console.WriteLine($"Vector3Wide: {Unsafe.SizeOf<Vector3Wide>()}");
                Console.WriteLine($"Matrix2x2Wide: {Unsafe.SizeOf<Matrix2x2Wide>()}");
                Console.WriteLine($"Matrix2x3Wide: {Unsafe.SizeOf<Matrix2x3Wide>()}");
                Console.WriteLine($"Vector3Wide: {Unsafe.SizeOf<Vector3Wide>()}");
                Console.WriteLine($"Tangent Jacobians: {Unsafe.SizeOf<TangentFriction.Jacobians>()}");
                TangentPrestepBaseline(ref tangentX, ref tangentY, ref offsetA, ref offsetB, ref bodyInertiasA, ref bodyInertiasB, out var projection);
                TangentPrestepTest(ref tangentX, ref tangentY, ref offsetA, ref offsetB, ref bodyInertiasA, ref bodyInertiasB, out projection);
            }
            {
                var m = new OuterStruct();
                var s = new Vector<float>();
                NestedStructTest(ref m, ref s, out var result);
                SubcallsNestedStructTest(ref m, ref s, out result);

                var m2 = new FlattenedStruct();
                FlattenedStructTest(ref m2, ref s, out var result2);
                RenestedStructTest(ref m2, ref s, out result2);

                var m3 = new DummyOuterStruct();
                DummyNestedStructTest(ref m3, ref s, out var result3);
            }
            {
                var m = new ScalarOuterStruct();
                var s = 0f;
                ScalarNestedStructTest(ref m, s, out var result);

                var m2 = new ScalarFlattenedStruct();
                ScalarFlattenedStructTest(ref m2, s, out var result2);

            }
        }

    }
}
