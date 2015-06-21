using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public class HybridConstraintTest
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
            var constraint = new HybridPenetrationConstraint
            {
                a0 = a,
                a1 = a,
                a2 = a,
                a3 = a,
                b0 = b,
                b1 = b,
                b2 = b,
                b3 = b,
                ContactPosition = new Vector3(),
                ContactNormal = new Vector3(0, 1, 0),
                ContactPenetration = 0
            };
            float dt = 1 / 60f;
            float inverseDt = 1 / dt;
            constraint.Prestep(inverseDt);
            constraint.WarmStart();
            constraint.SolveIteration();

            const int testCount = VectorizedConstraintTest.TestCount;
            const int iterationCount = VectorizedConstraintTest.IterationCount;

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

            Console.WriteLine($"Hybrid: {endtime - startTime}, acc: {constraint.AccumulatedImpulse}");


        }

    }
}
