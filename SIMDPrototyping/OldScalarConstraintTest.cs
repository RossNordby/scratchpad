using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.Entities;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUutilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace SIMDPrototyping
{
    public class OldScalarConstraintTest
    {
        public unsafe static void Test()
        {
            ContactPenetrationConstraint constraint = new ContactPenetrationConstraint();
            Contact contact = new Contact { Normal = new Vector3(0, 1, 0), PenetrationDepth = 0, Position = new Vector3() };
            var pairHandler = new BoxPairHandler();
            var a = new Entity(new BoxShape(1, 1, 1), 1)
            {
                Position = new Vector3(0, 0, 0),
                Orientation = Quaternion.Identity,
                LinearVelocity = new Vector3(0, 0, 0)
            };
            var b = new Entity(new BoxShape(1, 1, 1), 1)
            {
                Position = new Vector3(0, 1, 0),
                Orientation = Quaternion.Identity,
                LinearVelocity = new Vector3(0, -1, 0)
            };
            pairHandler.Initialize(a.CollisionInformation, b.CollisionInformation);
            ContactManifoldConstraint manifoldConstraint = new ConvexContactManifoldConstraint(pairHandler);
            manifoldConstraint.Initialize(a, b);

            constraint.Setup(manifoldConstraint, contact);
            


            float dt = 1 / 60f;
            float inverseDt = 1 / dt;
            constraint.Update(dt);
            constraint.ExclusiveUpdate();
            constraint.SolveIteration();

            const int testCount = VectorizedConstraintTest.TestCount * 4;
            const int iterationCount = VectorizedConstraintTest.IterationCount;

            var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            for (int i = 0; i < testCount; ++i)
            {
                constraint.Update(dt);
                constraint.ExclusiveUpdate();
                for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
                {
                    constraint.SolveIteration();
                }
            }


            var endtime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

            Console.WriteLine($"Scalar Old: {endtime - startTime}");
        }

    }
}
