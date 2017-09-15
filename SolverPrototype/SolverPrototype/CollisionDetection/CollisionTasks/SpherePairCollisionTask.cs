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

        struct PairWide
        {
            public Vector<float> RadiiA;
            public Vector<float> RadiiB;
            public Vector3Wide PositionA;
            public Vector3Wide PositionB;
        }
        //If we choose to go down this road, either you specify a unique lane stride for every intrinsic width or just pick a fixed length (64 byte+ for AVX512).
        //If you pick a fixed width, the user will have to be aware that the structure spans multiple registers on most hardware.
        //This has implications across the entire engine- the solver would be heavily hit. The question is... would the extra pipelining of beyond-intrinsic width win out over the
        //extra register pressure?
        //oooooooooooooooooooooooooooough. May be no other choice to take advantage of the next corefx intrinsics api.
        [StructLayout(LayoutKind.Explicit)]
        struct PairLane4
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

        [StructLayout(LayoutKind.Explicit)]
        struct Vector3Lane4
        {
            [FieldOffset(0)]
            public float X;
            [FieldOffset(16)]
            public float Y;
            [FieldOffset(32)]
            public float Z;
        }

        struct Remap2<T> where T : struct
        {
            public T T0;
            public T T1;
        }
        struct Remap4<T> where T : struct
        {
            public T T0;
            public T T1;
            public T T2;
            public T T3;
        }
        struct Remap8<T> where T : struct
        {
            public T T0;
            public T T1;
            public T T2;
            public T T3;
            public T T4;
            public T T5;
            public T T6;
            public T T7;
        }
        struct Remap16<T> where T : struct
        {
            public T T0;
            public T T1;
            public T T2;
            public T T3;
            public T T4;
            public T T5;
            public T T6;
            public T T7;
            public T T8;
            public T T9;
            public T T10;
            public T T11;
            public T T12;
            public T T13;
            public T T14;
            public T T15;
        }
        
        struct WhiningNoises4<TContainer> where TContainer : struct
        {
            //This is very gross.
            public TContainer T0;
            public TContainer T1;
            public TContainer T2;
            public TContainer T3;
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Gather<T, TContainer>(ref Vector<T> vector, ref TContainer containers, int offsetInElements) where T : struct where TContainer : struct
        {
            //Unfortunately, there is a fundamental mismatch between the ideal form of gathering with hardware acceleration and without hardware acceleration.
            //The 'count' serves as a mask in a scalar version, but introduces full branches.
            //TODO: May be worth an unconditional gather... would require a remainder loop.
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref containers), offsetInElements));
            if (Vector<T>.Count == 2)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap2<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
            }
            else if (Vector<T>.Count == 4)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap4<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
            }
            else if (Vector<T>.Count == 8)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap8<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
            }
            else if (Vector<T>.Count == 16)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap16<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
                remapped.T8 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 8));
                remapped.T9 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 9));
                remapped.T10 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 10));
                remapped.T11 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 11));
                remapped.T12 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 12));
                remapped.T13 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 13));
                remapped.T14 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 14));
                remapped.T15 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 15));
            }
            else
            {
                throw new InvalidOperationException("Unsupported type or vector size.");
            }
        }

        static void Gather<T, TContainer>(ref Vector<T> vector, ref TContainer containers, int offsetInElements, int count) where T : struct where TContainer : struct
        {
            //Unfortunately, there is a fundamental mismatch between the ideal form of gathering with hardware acceleration and without hardware acceleration.
            //The 'count' serves as a mask in a scalar version, but introduces full branches.
            //TODO: May be worth an unconditional gather... would require a remainder loop.
            Debug.Assert(count > 0);
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref containers), offsetInElements));
            if (Vector<T>.Count == 2)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap2<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 1)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
            }
            else if (Vector<T>.Count == 4)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap4<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 1)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 2)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 3)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
            }
            else if (Vector<T>.Count == 8)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap8<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 1)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 2)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 3)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                if (count >= 4)
                    remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                if (count >= 5)
                    remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                if (count >= 6)
                    remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                if (count >= 7)
                    remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
            }
            else if (Vector<T>.Count == 16)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap16<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 1)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 2)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 3)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                if (count >= 4)
                    remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                if (count >= 5)
                    remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                if (count >= 6)
                    remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                if (count >= 7)
                    remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
                if (count >= 8)
                    remapped.T8 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 8));
                if (count >= 9)
                    remapped.T9 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 9));
                if (count >= 10)
                    remapped.T10 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 10));
                if (count >= 11)
                    remapped.T11 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 11));
                if (count >= 12)
                    remapped.T12 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 12));
                if (count >= 13)
                    remapped.T13 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 13));
                if (count >= 14)
                    remapped.T14 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 14));
                if (count >= 15)
                    remapped.T15 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 15));
            }
            else
            {
                throw new InvalidOperationException("Unsupported type or vector size.");
            }
        }

        //Every single collision task type will mirror this general layout.
        //
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Sphere>>(ref batch.Buffer[0]);
            ContactManifold manifold;
            var trustMeThisManifoldIsTotallyInitialized = &manifold;
            manifold.SetConvexityAndCount(1, true);

            PairWide wide;
            ref var baseLane = ref Unsafe.As<PairWide, float>(ref wide);

            Vector3Wide contactNormal, contactPosition, relativePosition;
            Vector<float> depth;
            ref var normalLaneStart = ref Unsafe.As<Vector3Wide, float>(ref contactNormal);
            ref var positionLaneStart = ref Unsafe.As<Vector3Wide, float>(ref contactPosition);
            ref var relativePositionLaneStart = ref Unsafe.As<Vector3Wide, float>(ref relativePosition);
            ref var depthLaneStart = ref Unsafe.As<Vector<float>, float>(ref depth);

            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;

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

                //Gather(ref wide.RadiiA, ref bundleStart, 0);
                //Gather(ref wide.RadiiB, ref bundleStart, 1);
                //Gather(ref wide.PositionA.X, ref bundleStart, 2);
                //Gather(ref wide.PositionA.Y, ref bundleStart, 3);
                //Gather(ref wide.PositionA.Z, ref bundleStart, 4);
                //Gather(ref wide.PositionB.X, ref bundleStart, 9);
                //Gather(ref wide.PositionB.Y, ref bundleStart, 10);
                //Gather(ref wide.PositionB.Z, ref bundleStart, 11);


                //TODO: Boy howdy it sure would be nice to have gathers and scatters. They aren't fundamentally faster on most hardware than just 
                //direct scalar loads and insertions, but we don't exactly have access to high efficiency codegen for direct scalar loads and insertions either.
                for (int j = 0; j < countInBundle; ++j)
                {
                    ref var pair = ref Unsafe.Add(ref bundleStart, j);
                    ref var lane = ref Unsafe.As<float, PairLane4>(ref Unsafe.Add(ref baseLane, j));
                    lane.A = pair.A.Radius;
                    lane.B = pair.B.Radius;
                    lane.AX = pair.PoseA.Position.X;
                    lane.AY = pair.PoseA.Position.Y;
                    lane.AZ = pair.PoseA.Position.Z;
                    lane.BX = pair.PoseB.Position.X;
                    lane.BY = pair.PoseB.Position.Y;
                    lane.BZ = pair.PoseB.Position.Z;
                }
                Vector3Wide.Subtract(ref wide.PositionB, ref wide.PositionA, out relativePosition);
                SpherePairTester.Test(ref wide.RadiiA, ref wide.RadiiB, ref relativePosition, out contactPosition, out contactNormal, out depth);

                for (int j = 0; j < countInBundle; ++j)
                {
                    //If this doesn't suffer from the same type punning compiler bug, I'll be surprised.
                    ref var normalLane = ref Unsafe.As<float, Vector3Lane4>(ref Unsafe.Add(ref normalLaneStart, j));
                    ref var positionLane = ref Unsafe.As<float, Vector3Lane4>(ref Unsafe.Add(ref positionLaneStart, j));
                    ref var relativePositionLane = ref Unsafe.As<float, Vector3Lane4>(ref Unsafe.Add(ref relativePositionLaneStart, j));
                    manifold.ConvexNormal = new Vector3(normalLane.X, normalLane.Y, normalLane.Z);
                    manifold.Offset0 = new Vector3(positionLane.X, positionLane.Y, positionLane.Z);
                    manifold.OffsetB = new Vector3(relativePositionLane.X, relativePositionLane.Y, relativePositionLane.Z);
                    manifold.Depth0 = Unsafe.Add(ref depthLaneStart, j);
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifold);
                }

                //for (int j = 0; j < countInBundle; ++j)
                //{
                //    manifold.ConvexNormal = new Vector3(contactNormal.X[j], contactNormal.Y[j], contactNormal.Z[j]);
                //    manifold.Offset0 = new Vector3(contactPosition.X[j], contactPosition.Y[j], contactPosition.Z[j]);
                //    manifold.OffsetB = new Vector3(relativePosition.X[j], relativePosition.Y[j], relativePosition.Z[j]);
                //    manifold.Depth0 = depth[j];
                //    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifold);
                //}
            }
        }
    }
}
