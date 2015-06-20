using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public class VectorizedConstraintTest
    {
        public const int TestCount = 1000000;
        public const int IterationCount = 10;
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
            RigidBody* aBodies = stackalloc RigidBody[4];
            RigidBody* bBodies = stackalloc RigidBody[4];
            for (int i = 0; i < 4; ++i)
            {
                aBodies[i] = a;
                bBodies[i] = b;
            }
            Vector3 up = Vector3.UnitY;
            VectorizedPenetrationConstraint constraint = new VectorizedPenetrationConstraint
            {
                ABodies0 = a,
                ABodies1 = a,
                ABodies2 = a,
                ABodies3 = a,
                BBodies0 = b,
                BBodies1 = b,
                BBodies2 = b,
                BBodies3 = b,
                ContactPosition = new Vector3Width4(),
                ContactNormal = new Vector3Width4(ref up, ref up, ref up, ref up),
                ContactPenetration = new Vector4()
            };
            float dt = 1 / 60f;
            float inverseDt = 1 / dt;
            constraint.Prestep(inverseDt);
            constraint.WarmStart();
            constraint.SolveIteration();

            var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < TestCount; ++i)
            {
                constraint.Prestep(inverseDt);
                constraint.WarmStart();
                for (int iterationIndex = 0; iterationIndex < IterationCount; ++iterationIndex)
                {
                    constraint.SolveIteration();
                }
            }


            var endtime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            Console.WriteLine($"Vectorized: {endtime - startTime}, acc: {constraint.AccumulatedImpulse}");


        }

    }
}
