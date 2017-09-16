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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Gather<T, TContainer>(ref Vector<T> vector, ref TContainer sourceContainers, int offsetInElements) where T : struct where TContainer : struct
        {
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref sourceContainers), offsetInElements));
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Scatter<T, TContainer>(ref Vector<T> vector, ref TContainer targetContainers, int offsetInElements) where T : struct where TContainer : struct
        {
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref targetContainers), offsetInElements));
            if (Vector<T>.Count == 2)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap2<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
            }
            else if (Vector<T>.Count == 4)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap4<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
            }
            else if (Vector<T>.Count == 8)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap8<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4)) = remapped.T4;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5)) = remapped.T5;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6)) = remapped.T6;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7)) = remapped.T7;
            }
            else if (Vector<T>.Count == 16)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap16<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4)) = remapped.T4;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5)) = remapped.T5;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6)) = remapped.T6;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7)) = remapped.T7;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 8)) = remapped.T8;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 9)) = remapped.T9;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 10)) = remapped.T10;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 11)) = remapped.T11;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 12)) = remapped.T12;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 13)) = remapped.T13;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 14)) = remapped.T14;
                Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 15)) = remapped.T15;
            }
            else
            {
                throw new InvalidOperationException("Unsupported type or vector size.");
            }
        }


        //TODO: In the future, maybe once codegen improves a little bit, it might be a good idea to revisit this remainder gather to get rid of the redundant per-property branches.
        //Worst case scenario, you could create a bunch of overloads for different property counts. ooooooooooooooof.
        static void Gather<T, TContainer>(ref Vector<T> vector, ref TContainer containers, int offsetInElements, int count) where T : struct where TContainer : struct
        {
            Debug.Assert(count > 0 && count <= Vector<float>.Count);
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref containers), offsetInElements));
            if (Vector<T>.Count == 2)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap2<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 2)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
            }
            else if (Vector<T>.Count == 4)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap4<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 2)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 3)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 4)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
            }
            else if (Vector<T>.Count == 8)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap8<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 2)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 3)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 4)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                if (count >= 5)
                    remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                if (count >= 6)
                    remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                if (count >= 7)
                    remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                if (count >= 8)
                    remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
            }
            else if (Vector<T>.Count == 16)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap16<T>>(ref vector);
                remapped.T0 = Unsafe.As<TContainer, T>(ref start);
                if (count >= 2)
                    remapped.T1 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1));
                if (count >= 3)
                    remapped.T2 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2));
                if (count >= 4)
                    remapped.T3 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3));
                if (count >= 5)
                    remapped.T4 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4));
                if (count >= 6)
                    remapped.T5 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5));
                if (count >= 7)
                    remapped.T6 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6));
                if (count >= 8)
                    remapped.T7 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7));
                if (count >= 9)
                    remapped.T8 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 8));
                if (count >= 10)
                    remapped.T9 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 9));
                if (count >= 11)
                    remapped.T10 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 10));
                if (count >= 12)
                    remapped.T11 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 11));
                if (count >= 13)
                    remapped.T12 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 12));
                if (count >= 14)
                    remapped.T13 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 13));
                if (count >= 15)
                    remapped.T14 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 14));
                if (count == 16)
                    remapped.T15 = Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 15));
            }
            else
            {
                throw new InvalidOperationException("Unsupported type or vector size.");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Scatter<T, TContainer>(ref Vector<T> vector, ref TContainer targetContainers, int offsetInElements, int count) where T : struct where TContainer : struct
        {
            ref var start = ref Unsafe.As<T, TContainer>(ref Unsafe.Add(ref Unsafe.As<TContainer, T>(ref targetContainers), offsetInElements));
            if (Vector<T>.Count == 2)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap2<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                if (count >= 2)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
            }
            else if (Vector<T>.Count == 4)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap4<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                if (count >= 2)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                if (count >= 3)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                if (count >= 4)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
            }
            else if (Vector<T>.Count == 8)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap8<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                if (count >= 2)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                if (count >= 3)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                if (count >= 4)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
                if (count >= 5)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4)) = remapped.T4;
                if (count >= 6)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5)) = remapped.T5;
                if (count >= 7)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6)) = remapped.T6;
                if (count >= 8)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7)) = remapped.T7;
            }
            else if (Vector<T>.Count == 16)
            {
                ref var remapped = ref Unsafe.As<Vector<T>, Remap16<T>>(ref vector);
                Unsafe.As<TContainer, T>(ref start) = remapped.T0;
                if (count >= 2)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 1)) = remapped.T1;
                if (count >= 3)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 2)) = remapped.T2;
                if (count >= 4)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 3)) = remapped.T3;
                if (count >= 5)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 4)) = remapped.T4;
                if (count >= 6)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 5)) = remapped.T5;
                if (count >= 7)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 6)) = remapped.T6;
                if (count >= 8)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 7)) = remapped.T7;
                if (count >= 9)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 8)) = remapped.T8;
                if (count >= 10)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 9)) = remapped.T9;
                if (count >= 11)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 10)) = remapped.T10;
                if (count >= 12)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 11)) = remapped.T11;
                if (count >= 13)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 12)) = remapped.T12;
                if (count >= 14)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 13)) = remapped.T13;
                if (count >= 15)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 14)) = remapped.T14;
                if (count >= 16)
                    Unsafe.As<TContainer, T>(ref Unsafe.Add(ref start, 15)) = remapped.T15;
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
            var manifolds = stackalloc ContactManifold[Vector<float>.Count];
            var trustMeThisManifoldIsTotallyInitialized = &manifolds;
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                manifolds[i].SetConvexityAndCount(1, true);
            }
            PairWide wide;
            ref var baseLane = ref Unsafe.As<PairWide, float>(ref wide);

            Vector3Wide contactNormal, contactPosition, relativePosition;
            Vector<float> depth;

            var fullBundleCount = batch.Count / Vector<float>.Count;
            var pairsInFullBundlesCount = fullBundleCount * Vector<float>.Count;
            for (int i = 0; i < pairsInFullBundlesCount; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);

                Gather(ref wide.RadiiA, ref bundleStart, 0);
                Gather(ref wide.RadiiB, ref bundleStart, 1);
                Gather(ref wide.PositionA.X, ref bundleStart, 2);
                Gather(ref wide.PositionA.Y, ref bundleStart, 3);
                Gather(ref wide.PositionA.Z, ref bundleStart, 4);
                Gather(ref wide.PositionB.X, ref bundleStart, 9);
                Gather(ref wide.PositionB.Y, ref bundleStart, 10);
                Gather(ref wide.PositionB.Z, ref bundleStart, 11);

                Vector3Wide.Subtract(ref wide.PositionB, ref wide.PositionA, out relativePosition);
                SpherePairTester.Test(ref wide.RadiiA, ref wide.RadiiB, ref relativePosition, out contactPosition, out contactNormal, out depth);

                Scatter(ref relativePosition.X, ref *manifolds, 0);
                Scatter(ref relativePosition.Y, ref *manifolds, 1);
                Scatter(ref relativePosition.Z, ref *manifolds, 2);
                Scatter(ref contactPosition.X, ref *manifolds, 4);
                Scatter(ref contactPosition.Y, ref *manifolds, 5);
                Scatter(ref contactPosition.Z, ref *manifolds, 6);
                Scatter(ref depth, ref *manifolds, 16);
                Scatter(ref contactNormal.X, ref *manifolds, 24);
                Scatter(ref contactNormal.Y, ref *manifolds, 25);
                Scatter(ref contactNormal.Z, ref *manifolds, 26);
                for (int j = 0; j < Vector<float>.Count;)
                {
                    Debug.Assert(Vector<float>.Count >= 4);
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]); ++j;
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]); ++j;
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]); ++j;
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]); ++j;
                }
            }

            if (batch.Count > pairsInFullBundlesCount)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, pairsInFullBundlesCount);
                int countInBundle = batch.Count - pairsInFullBundlesCount;

                Gather(ref wide.RadiiA, ref bundleStart, 0, countInBundle);
                Gather(ref wide.RadiiB, ref bundleStart, 1, countInBundle);
                Gather(ref wide.PositionA.X, ref bundleStart, 2, countInBundle);
                Gather(ref wide.PositionA.Y, ref bundleStart, 3, countInBundle);
                Gather(ref wide.PositionA.Z, ref bundleStart, 4, countInBundle);
                Gather(ref wide.PositionB.X, ref bundleStart, 9, countInBundle);
                Gather(ref wide.PositionB.Y, ref bundleStart, 10, countInBundle);
                Gather(ref wide.PositionB.Z, ref bundleStart, 11, countInBundle);

                Vector3Wide.Subtract(ref wide.PositionB, ref wide.PositionA, out relativePosition);
                SpherePairTester.Test(ref wide.RadiiA, ref wide.RadiiB, ref relativePosition, out contactPosition, out contactNormal, out depth);

                Scatter(ref relativePosition.X, ref *manifolds, 0, countInBundle);
                Scatter(ref relativePosition.Y, ref *manifolds, 1, countInBundle);
                Scatter(ref relativePosition.Z, ref *manifolds, 2, countInBundle);
                Scatter(ref contactPosition.X, ref *manifolds, 4, countInBundle);
                Scatter(ref contactPosition.Y, ref *manifolds, 5, countInBundle);
                Scatter(ref contactPosition.Z, ref *manifolds, 6, countInBundle);
                Scatter(ref depth, ref *manifolds, 16, countInBundle);
                Scatter(ref contactNormal.X, ref *manifolds, 24, countInBundle);
                Scatter(ref contactNormal.Y, ref *manifolds, 25, countInBundle);
                Scatter(ref contactNormal.Z, ref *manifolds, 26, countInBundle);
                for (int j = 0; j < countInBundle; ++j)
                {
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]);
                }
            }

            //for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            //{
            //    ref var bundleStart = ref Unsafe.Add(ref start, i);
            //    int countInBundle = batch.Count - i;
            //    if (countInBundle > Vector<float>.Count)
            //        countInBundle = Vector<float>.Count;

            //    Gather(ref wide.RadiiA, ref bundleStart, 0, countInBundle);
            //    Gather(ref wide.RadiiB, ref bundleStart, 1, countInBundle);
            //    Gather(ref wide.PositionA.X, ref bundleStart, 2, countInBundle);
            //    Gather(ref wide.PositionA.Y, ref bundleStart, 3, countInBundle);
            //    Gather(ref wide.PositionA.Z, ref bundleStart, 4, countInBundle);
            //    Gather(ref wide.PositionB.X, ref bundleStart, 9, countInBundle);
            //    Gather(ref wide.PositionB.Y, ref bundleStart, 10, countInBundle);
            //    Gather(ref wide.PositionB.Z, ref bundleStart, 11, countInBundle);

            //    Vector3Wide.Subtract(ref wide.PositionB, ref wide.PositionA, out relativePosition);
            //    SpherePairTester.Test(ref wide.RadiiA, ref wide.RadiiB, ref relativePosition, out contactPosition, out contactNormal, out depth);

            //    Scatter(ref relativePosition.X, ref *manifolds, 0, countInBundle);
            //    Scatter(ref relativePosition.Y, ref *manifolds, 1, countInBundle);
            //    Scatter(ref relativePosition.Z, ref *manifolds, 2, countInBundle);
            //    Scatter(ref contactPosition.X, ref *manifolds, 4, countInBundle);
            //    Scatter(ref contactPosition.Y, ref *manifolds, 5, countInBundle);
            //    Scatter(ref contactPosition.Z, ref *manifolds, 6, countInBundle);
            //    Scatter(ref depth, ref *manifolds, 16, countInBundle);
            //    Scatter(ref contactNormal.X, ref *manifolds, 24, countInBundle);
            //    Scatter(ref contactNormal.Y, ref *manifolds, 25, countInBundle);
            //    Scatter(ref contactNormal.Z, ref *manifolds, 26, countInBundle);
            //    for (int j = 0; j < countInBundle; ++j)
            //    {
            //        continuations.Notify(Unsafe.Add(ref bundleStart, j).Continuation, ref manifolds[j]);
            //    }
            //}
        }
    }
}
