using SolverPrototype;
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
        public static void MicroTest(ref Vector3Wide angularA, ref Vector3Wide angularB,            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //NO LOCALSINIT REQUIRED.
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
            //LOCALSINIT REQUIRED.
            Vector3Wide.Add(ref angularA, ref wsvA.LinearVelocity, out var temp);
            wsvB.AngularVelocity = temp;
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void MicroTest3(ref Vector3Wide angularA, ref Vector3Wide angularB, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //NO LOCALSINIT REQUIRED.
            Vector3Wide.Add(ref angularA, ref wsvA.LinearVelocity, out wsvB.AngularVelocity);
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
        }

    }
}
