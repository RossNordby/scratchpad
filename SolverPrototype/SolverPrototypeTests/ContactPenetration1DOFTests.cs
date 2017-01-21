﻿using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    static class ContactPenetration1DOFTests
    {
        public static void Test()
        {
            Bodies bodies = new Bodies();
            const int bodyCount = 16384;
            var handleIndices = new int[bodyCount];
            for (int i = 0; i < bodyCount; ++i)
            {
                var description = new BodyDescription
                {
                    LocalInertia = new BodyInertia
                    {
                        InverseInertiaTensor = new Matrix3x3
                        {
                            X = new Vector3(1, 0, 0),
                            Y = new Vector3(0, 1, 0),
                            Z = new Vector3(0, 0, 1),
                        },
                        InverseMass = 1
                    },
                };
                var handleIndex = bodies.Add(ref description);
                handleIndices[i] = handleIndex;

                if ((i & 1) == 0)
                {
                    BodyVelocity velocity;
                    velocity.Linear = new Vector3(0, 1, 0);
                    velocity.Angular = new Vector3();
                    bodies.SetVelocity(handleIndex, ref velocity);
                }
            }

            int constraintCount = bodyCount / 2;
            int constraintBundleCount = (int)Math.Ceiling(constraintCount / (double)Vector<float>.Count);
            var bodyReferences = new BodyReferences[constraintBundleCount];
            var springSettings = new SpringSettings[constraintBundleCount];
            var iterationData = new IterationData2Body1DOF[constraintBundleCount];
            var contactData = new ContactData[constraintBundleCount];
            var accumulatedImpulses = new Vector<float>[constraintBundleCount];
            for (int i = 0; i < constraintCount; ++i)
            {
                var bodyAIndex = bodies.BodyHandles[handleIndices[i * 2]];
                var bodyBIndex = bodies.BodyHandles[handleIndices[i * 2 + 1]];
                BundleIndexing.GetBundleIndices(bodyAIndex, out var bodyABundleIndex, out var bodyAInnerIndex);
                BundleIndexing.GetBundleIndices(bodyBIndex, out var bodyBBundleIndex, out var bodyBInnerIndex);
                BundleIndexing.GetBundleIndices(i, out var constraintBundleIndex, out var constraintInnerIndex);
                ref var constraintBodies = ref bodyReferences[constraintBundleIndex];
                GatherScatter.Get(ref constraintBodies.BundleIndexA, constraintInnerIndex) = bodyABundleIndex;
                GatherScatter.Get(ref constraintBodies.InnerIndexA, constraintInnerIndex) = bodyAInnerIndex;
                GatherScatter.Get(ref constraintBodies.BundleIndexB, constraintInnerIndex) = bodyBBundleIndex;
                GatherScatter.Get(ref constraintBodies.InnerIndexB, constraintInnerIndex) = bodyBInnerIndex;
                var constraintBundleBaseIndex = constraintBundleIndex << BundleIndexing.VectorShift;
                ++constraintBodies.Count;

                ref var springs = ref springSettings[constraintBundleIndex];
                GatherScatter.Get(ref springs.NaturalFrequency, constraintInnerIndex) = (float)(Math.PI * 2 * 60);
                GatherScatter.Get(ref springs.DampingRatio, constraintInnerIndex) = 1;

                ref var contacts = ref contactData[constraintBundleIndex];
                GatherScatter.Get(ref contacts.Normal.Y, constraintInnerIndex) = 1;
                GatherScatter.Get(ref contacts.OffsetA.Y, constraintInnerIndex) = 0.5f;
                GatherScatter.Get(ref contacts.OffsetB.Y, constraintInnerIndex) = -0.5f;
                GatherScatter.Get(ref contacts.PenetrationDepth, constraintInnerIndex) = 0.02f;


            }

            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 5;
            const int frameCount = 1000;

            void SolveFrame()
            {
                for (int i = 0; i < constraintBundleCount; ++i)
                {
                    ContactPenetrationLimit.ComputeJacobiansAndError(ref contactData[i], out var jacobians, out var error);
                    var maximumRecoveryVelocity = new Vector<float>(1);
                    GatherScatter.GatherInertia(bodies.LocalInertiaBundles, ref bodyReferences[i], out var inertiaA, out var inertiaB);
                    Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians, ref springSettings[i],
                        ref error, dt, inverseDt, out iterationData[i]);

                    GatherScatter.GatherVelocities(bodies.VelocityBundles, ref bodyReferences[i], out var wsvA, out var wsvB);
                    Inequality2Body1DOF.WarmStart(ref iterationData[i], ref accumulatedImpulses[i], ref wsvA, ref wsvB);
                    GatherScatter.ScatterVelocities(bodies.VelocityBundles, ref bodyReferences[i], ref wsvA, ref wsvB);
                }
                for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
                {
                    for (int i = 0; i < constraintBundleCount; ++i)
                    {
                        GatherScatter.GatherVelocities(bodies.VelocityBundles, ref bodyReferences[i], out var wsvA, out var wsvB);
                        Inequality2Body1DOF.Solve(ref iterationData[i], ref accumulatedImpulses[i], ref wsvA, ref wsvB);
                        GatherScatter.ScatterVelocities(bodies.VelocityBundles, ref bodyReferences[i], ref wsvA, ref wsvB);
                    }
                }
            }
            //prejit
            SolveFrame();
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            var start = Stopwatch.GetTimestamp();
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                SolveFrame();
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Time (ms): {(1e3 * (end - start)) / Stopwatch.Frequency}");

        }


    }
}
