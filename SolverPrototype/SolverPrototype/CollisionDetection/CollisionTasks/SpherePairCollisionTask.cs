using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
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

        public static void BuildOrthnormalBasis(ref Vector3Wide normal, out QuaternionWide basis)
        {
            //This could probably be improved.
            var sign = Vector.ConditionalSelect(Vector.LessThan(normal.Z, Vector<float>.Zero), -Vector<float>.One, Vector<float>.One);

            //This has a discontinuity at z==0. Raw frisvad has only one discontinuity, though that region is more unpredictable than the revised version.
            var scale = -Vector<float>.One / (sign + normal.Z);
            Vector3Wide t1, t2;
            t1.X = normal.X * normal.Y * scale;
            t1.Y = sign + normal.Y * normal.Y * scale;
            t1.Z = -normal.Y;

            t2.X = Vector<float>.One + sign * normal.X * normal.X * scale;
            t2.Y = sign * t1.X;
            t2.Z = -sign * normal.X;

            QuaternionWide.CreateFromRotationMatrix()
        }

        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            ref var start = ref Unsafe.As<byte, RigidPair<Sphere, Sphere>>(ref batch.Buffer[0]);
            ContactManifold manifold;
            var trustMeThisManifoldIsTotallyInitialized = &manifold;
            manifold.SetConvexityAndCount(1, true);
            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;
                LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.A.Radius, countInBundle, out var radiiA);
                LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.B.Radius, countInBundle, out var radiiB);
                Vector3Wide relativePosition;
                LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.RelativePose.Position.X, countInBundle, out relativePosition.X);
                LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.RelativePose.Position.Y, countInBundle, out relativePosition.Y);
                LocalAOSToSOA<RigidPair<Sphere, Sphere>, float>(ref bundleStart.RelativePose.Position.Z, countInBundle, out relativePosition.Z);
                SpherePairTester.Test(ref radiiA, ref radiiB, ref relativePosition, out var contactPosition, out var contactNormal, out var depth);
                for (int j = 0; j < countInBundle; ++j)
                {
                    manifold.ConvexSurfaceBasis = GatherScatter.GetLane()
                    continuations.Notify()
                }
            }
        }
    }
}
