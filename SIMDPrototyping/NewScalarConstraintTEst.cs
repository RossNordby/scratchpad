using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public class NewScalarConstraintTest
    {
        public unsafe static void Test()
        {
            var identityMatrix = new Matrix3x3 { X = new Vector3(1, 0, 0), Y = new Vector3(0, 1, 0), Z = new Vector3(0, 0, 1) };
            var a = new RigidBody
            {
                Position = new Vector3(0, 0, 0),
                Orientation = identityMatrix,
                InertiaTensorInverse = identityMatrix,
                InverseMass = 1,
                LinearVelocity = new Vector3(0, 0, 0)
            };
            var b = new RigidBody
            {
                Position = new Vector3(0, 1, 0),
                Orientation = identityMatrix,
                InertiaTensorInverse = identityMatrix,
                InverseMass = 1,
                LinearVelocity = new Vector3(0, -1, 0)
            };
            PenetrationConstraint constraint = new PenetrationConstraint
            {
                ConnectionA = &a,
                ConnectionB = &b,
                ContactPosition = new Vector3(),
                ContactNormal = new Vector3(0, 1, 0),
                ContactPenetration = 0
            };
            float dt = 1 / 60f;
            float inverseDt = 1/ dt;
            constraint.Prestep(inverseDt);
            constraint.WarmStart();
            constraint.SolveIteration();

            const int testCount = 1000000;
            const int iterationCount = 10;

            var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < testCount; ++i)
            {
                constraint.Prestep(inverseDt);
                constraint.WarmStart();
                for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
                {
                    constraint.SolveIteration();
                }
            }


            var endtime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            Console.WriteLine($"Scalar New: {endtime - startTime}, acc: {constraint.AccumulatedImpulse}");


        }

    }
}
