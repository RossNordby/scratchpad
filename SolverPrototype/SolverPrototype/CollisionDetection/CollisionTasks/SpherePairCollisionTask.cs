using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Test(
            ref Vector<float> radiiA, ref Vector<float> radiiB,
            ref Vector3Wide relativePositionB,
            out Vector3Wide relativeContactPosition, out Vector3Wide contactNormal, out Vector<float> depth)
        {
            Vector3Wide.Length(ref relativePositionB, out var centerDistance);
            var inverseDistance = Vector<float>.One / centerDistance;
            Vector3Wide.Scale(ref relativePositionB, ref inverseDistance, out contactNormal);
            var normalIsValid = Vector.GreaterThan(centerDistance, Vector<float>.Zero);
            //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
            contactNormal.X = Vector.ConditionalSelect(normalIsValid, contactNormal.X, Vector<float>.Zero);
            contactNormal.Y = Vector.ConditionalSelect(normalIsValid, contactNormal.Y, Vector<float>.One);
            contactNormal.Z = Vector.ConditionalSelect(normalIsValid, contactNormal.Z, Vector<float>.Zero);
            depth = radiiA + radiiB - centerDistance;
            //The position should be placed at the average of the extremePoint(a, a->b) and extremePoint(b, b->a). That puts it in the middle of the overlapping or nonoverlapping interval.
            //The contact normal acts as the direction from a to b.
            Vector3Wide.Scale(ref contactNormal, ref radiiA, out var extremeA);
            Vector3Wide.Scale(ref contactNormal, ref radiiB, out var extremeB);
            //note the following subtraction: contactNormal goes from a to b, so the negation pushes the extreme point in the proper direction from b to a.
            Vector3Wide.Subtract(ref relativePositionB, ref extremeB, out extremeB);
            Vector3Wide.Add(ref extremeA, ref extremeB, out relativeContactPosition);
            var scale = new Vector<float>(0.5f);
            Vector3Wide.Scale(ref relativeContactPosition, ref scale, out relativeContactPosition);
        }
    }

    public class SpherePairCollisionTask : CollisionTask
    {
        public SpherePairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = TypeIds<IShape>.GetId<Sphere>();
            ShapeTypeIndexB = ShapeTypeIndexA;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LocalAOSToSOA<TSourceContainer, TLane>(ref TLane source, int count, out Vector<TLane> lane) where TLane : struct
        {
            //TODO: Check for codegen issues. Type punning concerns.
            Debug.Assert(count <= Vector<TLane>.Count);
            int containerSizeInLanes = Unsafe.SizeOf<TSourceContainer>() / Unsafe.SizeOf<TLane>();
            Debug.Assert(containerSizeInLanes * Unsafe.SizeOf<TLane>() == Unsafe.SizeOf<TSourceContainer>(),
                "We assume a container evenly divisible by the size of the type of its lanes so that the container step can be expressed as lane steps.");
            ref var laneSlot = ref Unsafe.As<Vector<TLane>, TLane>(ref lane);
            laneSlot = source;
            int sourceIndex = 0;
            for (int i = 1; i < count; ++i)
            {
                sourceIndex += containerSizeInLanes;
                Unsafe.Add(ref laneSlot, i) = Unsafe.Add(ref source, sourceIndex);
            }
        }

        struct TestWide
        {
            public Vector<float> RadiiA;
            public Vector<float> RadiiB;
            public Vector3Wide PositionA;
            public Vector3Wide PositionB;
        }
        [StructLayout(LayoutKind.Explicit)]
        struct TestLane
        {
            [FieldOffset(0)]
            public float A;
            [FieldOffset(16)]
            public float B;
            [FieldOffset(32)]
            public float AX;
            [FieldOffset(48)]
            public float AY;
            [FieldOffset(64)]
            public float AZ;
            [FieldOffset(80)]
            public float BX;
            [FieldOffset(96)]
            public float BY;
            [FieldOffset(112)]
            public float BZ;
        }


        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Sphere>>(ref batch.Buffer[0]);
            ContactManifold manifold;
            var trustMeThisManifoldIsTotallyInitialized = &manifold;
            manifold.SetConvexityAndCount(1, true);

            TestWide wide;
            ref var baseLane = ref Unsafe.As<TestWide, float>(ref wide);

            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;
                ////TODO: It's likely that the compiler won't be able to fold all these independent transposition loops. Would be better to do all data at once in a single loop.
                ////Could do some generics abuse.
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.A.Radius, countInBundle, out var radiiA);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.B.Radius, countInBundle, out var radiiB);
                //Vector3Wide positionA, positionB;
                ////TODO: It's very possible that on most hardware using a scalar Vector3 subtract is faster than doing the AOS->SOA transpose followed by wide subtract.
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseA.Position.X, countInBundle, out positionA.X);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseA.Position.Y, countInBundle, out positionA.Y);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseA.Position.Z, countInBundle, out positionA.Z);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseB.Position.X, countInBundle, out positionB.X);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseB.Position.Y, countInBundle, out positionB.Y);
                //LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.PoseB.Position.Z, countInBundle, out positionB.Z);
                //Vector3Wide.Subtract(ref positionB, ref positionA, out var relativePosition);
                //SpherePairTester.Test(ref radiiA, ref radiiB, ref relativePosition, out var contactPosition, out var contactNormal, out var depth);


                //Vector<float> radiiA, radiiB;
                //ref var slotsRadiiA = ref Unsafe.As<Vector<float>, float>(ref radiiA);
                //ref var slotsRadiiB = ref Unsafe.As<Vector<float>, float>(ref radiiB);
                //Vector3Wide positionA, positionB;
                //ref var slotsPositionAX = ref Unsafe.As<Vector<float>, float>(ref positionA.X);
                //ref var slotsPositionAY = ref Unsafe.As<Vector<float>, float>(ref positionA.Y);
                //ref var slotsPositionAZ = ref Unsafe.As<Vector<float>, float>(ref positionA.Z);
                //ref var slotsPositionBX = ref Unsafe.As<Vector<float>, float>(ref positionB.X);
                //ref var slotsPositionBY = ref Unsafe.As<Vector<float>, float>(ref positionB.Y);
                //ref var slotsPositionBZ = ref Unsafe.As<Vector<float>, float>(ref positionB.Z);
                //for (int j = 0; j < countInBundle; ++j)
                //{
                //    ref var pair = ref Unsafe.Add(ref bundleStart, j);
                //    Unsafe.Add(ref slotsRadiiA, j) = pair.A.Radius;
                //    Unsafe.Add(ref slotsRadiiB, j) = pair.B.Radius;
                //    Unsafe.Add(ref slotsPositionAX, j) = pair.PoseA.Position.X;
                //    Unsafe.Add(ref slotsPositionAY, j) = pair.PoseA.Position.Y;
                //    Unsafe.Add(ref slotsPositionAZ, j) = pair.PoseA.Position.Z;
                //    Unsafe.Add(ref slotsPositionBX, j) = pair.PoseB.Position.X;
                //    Unsafe.Add(ref slotsPositionBY, j) = pair.PoseB.Position.Y;
                //    Unsafe.Add(ref slotsPositionBZ, j) = pair.PoseB.Position.Z;
                //}
                //Vector3Wide.Subtract(ref positionB, ref positionA, out var relativePosition);
                //SpherePairTester.Test(ref radiiA, ref radiiB, ref relativePosition, out var contactPosition, out var contactNormal, out var depth);

                for (int j = 0; j < countInBundle; ++j)
                {
                    ref var pair = ref Unsafe.Add(ref bundleStart, j);
                    ref var lane = ref Unsafe.As<float, TestLane>(ref Unsafe.Add(ref baseLane, j));
                    lane.A = pair.A.Radius;
                    lane.B = pair.B.Radius;
                    lane.AX = pair.PoseA.Position.X;
                    lane.AY = pair.PoseA.Position.Y;
                    lane.AZ = pair.PoseA.Position.Z;
                    lane.BX = pair.PoseB.Position.X;
                    lane.BY = pair.PoseB.Position.Y;
                    lane.BZ = pair.PoseB.Position.Z;
                }
                Vector3Wide.Subtract(ref wide.PositionB, ref wide.PositionA, out var relativePosition);
                SpherePairTester.Test(ref wide.RadiiA, ref wide.RadiiB, ref relativePosition, out var contactPosition, out var contactNormal, out var depth);

                Console.WriteLine(contactNormal);
                for (int j = 0; j < countInBundle; ++j)
                {
                    GatherScatter.GetLane(ref contactNormal.X, j, ref manifold.ConvexNormal.X, 3);
                    GatherScatter.GetLane(ref contactPosition.X, j, ref manifold.Offset0.X, 3);
                    manifold.Depth0 = GatherScatter.Get(ref depth, j);
                    GatherScatter.GetLane(ref relativePosition.X, j, ref manifold.OffsetB.X, 3);

                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifold);
                }
            }
        }
    }
}
