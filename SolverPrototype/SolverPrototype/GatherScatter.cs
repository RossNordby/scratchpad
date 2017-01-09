using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public class GatherScatter
    {

        /// <summary>
        /// Gets a reference to an element from a vector without using pointers, bypassing direct vector access for codegen reasons.
        /// This appears to produce identical assembly to taking the pointer and applying an offset. You can do slightly better for batched accesses
        /// by taking the pointer or reference only once, though the performance difference is small.
        /// This performs no bounds testing!
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe ref T Get<T>(ref Vector<T> vector, int index) where T : struct
        {
            return ref Unsafe.Add(ref Unsafe.As<Vector<T>, T>(ref vector), index);

            //For comparison, an implementation like this:
            //return ref *((float*)Unsafe.AsPointer(ref vector) + index);
            //doesn't inline (sometimes?). 
            //The good news is that, in addition to inlining and producing decent assembly, the pointerless approach doesn't open the door
            //for GC related problems and the user doesn't need to pin memory.
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void GatherVelocities(BodyVelocities[] allBodyVelocities,
            ref Vector<int> bundleIndicesVector, ref Vector<int> innerIndicesVector, int bodyCount, ref BodyVelocities velocities)
        {
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var baseLinearX = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);

            //Grab the base references for the body indices.
            ref var baseBundle = ref Unsafe.As<Vector<int>, int>(ref bundleIndicesVector);
            ref var baseInner = ref Unsafe.As<Vector<int>, int>(ref innerIndicesVector);

            for (int i = 0; i < bodyCount; ++i)
            {
                //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
                //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
                //The benefit of not having to load data is extremely weak, and often introducing null connections actually slows things down further
                //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
                //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
                var bundleIndex = Unsafe.Add(ref baseBundle, i);
                var innerIndex = Unsafe.Add(ref baseInner, i);
                ref var bundleLinearX = ref Get(ref allBodyVelocities[bundleIndex].LinearVelocity.X, innerIndex);
                ref var linearX = ref Unsafe.Add(ref baseLinearX, i);
                linearX = bundleLinearX;
                Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);

            }

            //You may notice that this is a pretty dang heavy weight preamble to every solve iteration. Two points:
            //1) You should make sure to make this preamble as common as possible to all constraints, so that every time a new optimization becomes possible, you needn't
            //revisit every single constraint.
            //2) When there's a possibility to increase the quality of a constraint solve by doing a little more effort within the solve iteration, chances are it's
            //worth it. For cheap low quality solves on low-DOF constraints, the pregather and postscatter may take significantly longer than the actual math!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            GatherVelocities(velocities, ref references.BundleIndexA, ref references.InnerIndexA, references.Count, ref velocitiesA);
            GatherVelocities(velocities, ref references.BundleIndexB, ref references.InnerIndexB, references.Count, ref velocitiesB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities2(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var baseLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var baseLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);


            //Grab the base references for the body indices.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);
            ref var baseInnerA = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexA);
            ref var baseBundleB = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexB);
            ref var baseInnerB = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexB);

            for (int i = 0; i < references.Count; ++i)
            {
                //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
                //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
                //The benefit of not having to load data is extremely weak, and often introducing null connections actually slows things down further
                //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
                //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
                {
                    var bundleIndex = Unsafe.Add(ref baseBundleA, i);
                    var innerIndex = Unsafe.Add(ref baseInnerA, i);

                    ref var bundleLinearX = ref Get(ref velocities[bundleIndex].LinearVelocity.X, innerIndex);
                    ref var linearX = ref Unsafe.Add(ref baseLinearAX, i);
                    linearX = bundleLinearX;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);
                }

                {
                    var bundleIndex = Unsafe.Add(ref baseBundleB, i);
                    var innerIndex = Unsafe.Add(ref baseInnerB, i);

                    ref var bundleLinearX = ref Get(ref velocities[bundleIndex].LinearVelocity.X, innerIndex);
                    ref var linearX = ref Unsafe.Add(ref baseLinearBX, i);
                    linearX = bundleLinearX;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);
                }
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities3(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var baseLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var baseLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
                //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
                //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
                //The benefit of not having to load data is extremely weak, and often introducing null connections actually slows things down further
                //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
                //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                {
                    var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);

                    ref var bundleLinearX = ref Get(ref velocities[bundleIndexA].LinearVelocity.X, innerIndexA);
                    ref var linearX = ref Unsafe.Add(ref baseLinearAX, i);
                    linearX = bundleLinearX;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);
                }

                {
                    var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                    var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);

                    ref var bundleLinearX = ref Get(ref velocities[bundleIndexB].LinearVelocity.X, innerIndexB);
                    ref var linearX = ref Unsafe.Add(ref baseLinearBX, i);
                    linearX = bundleLinearX;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GatherLane4(int laneIndex, BodyVelocities[] velocities, ref float baseLinearX, int bundleIndex, int innerIndex)
        {
            ref var bundleLinearX = ref Get(ref velocities[bundleIndex].LinearVelocity.X, innerIndex);
            ref var linearX = ref Unsafe.Add(ref baseLinearX, laneIndex);
            linearX = bundleLinearX;
            Unsafe.Add(ref linearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
            Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
            Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
            Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
            Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities4(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var baseLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var baseLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
                //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
                //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
                //The benefit of not having to load data is extremely weak, and often introducing null connections actually slows things down further
                //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
                //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                {
                    var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);
                    GatherLane4(i, velocities, ref baseLinearAX, bundleIndexA, innerIndexA);
                }

                {
                    var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                    var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);
                    GatherLane4(i, velocities, ref baseLinearBX, bundleIndexB, innerIndexB);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);
            ref var baseInnerA = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexA);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);

            ref var baseBundleB = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexB);
            ref var baseInnerB = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexB);
            ref var baseSourceLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            for (int i = 0; i < references.Count; ++i)
            {
                //We'll use the memory layout of the EntityVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearAX, i);
                var bundleIndexA = Unsafe.Add(ref baseBundleA, i);
                var innerIndexA = Unsafe.Add(ref baseInnerA, i);
                ref var targetLinearAX = ref Get(ref velocities[bundleIndexA].LinearVelocity.X, innerIndexA);
                targetLinearAX = sourceLinearAX;
                Unsafe.Add(ref targetLinearAX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, Vector<float>.Count);
                Unsafe.Add(ref targetLinearAX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearAX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearAX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearAX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count);

                ref var sourceLinearBX = ref Unsafe.Add(ref baseSourceLinearBX, i);
                var bundleIndexB = Unsafe.Add(ref baseBundleB, i);
                var innerIndexB = Unsafe.Add(ref baseInnerB, i);
                ref var targetLinearBX = ref Get(ref velocities[bundleIndexB].LinearVelocity.X, innerIndexB);
                targetLinearBX = sourceLinearBX;
                Unsafe.Add(ref targetLinearBX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, Vector<float>.Count);
                Unsafe.Add(ref targetLinearBX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 2 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearBX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 3 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearBX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 4 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearBX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 5 * Vector<float>.Count);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities2(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var baseSourceLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            for (int i = 0; i < references.Count; ++i)
            {
                //We'll use the memory layout of the EntityVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                //Same for the bundles.
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                {
                    var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);
                    ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearAX, i);
                    ref var targetLinearAX = ref Get(ref velocities[bundleIndexA].LinearVelocity.X, innerIndexA);
                    targetLinearAX = sourceLinearAX;
                    Unsafe.Add(ref targetLinearAX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, Vector<float>.Count);
                    Unsafe.Add(ref targetLinearAX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearAX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearAX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearAX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count);
                }

                {
                    var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                    var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);
                    ref var sourceLinearBX = ref Unsafe.Add(ref baseSourceLinearBX, i);
                    ref var targetLinearBX = ref Get(ref velocities[bundleIndexB].LinearVelocity.X, innerIndexB);
                    targetLinearBX = sourceLinearBX;
                    Unsafe.Add(ref targetLinearBX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, Vector<float>.Count);
                    Unsafe.Add(ref targetLinearBX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 2 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearBX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 3 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearBX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 4 * Vector<float>.Count);
                    Unsafe.Add(ref targetLinearBX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 5 * Vector<float>.Count);
                }
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ScatterLane3(int laneIndex, BodyVelocities[] velocities, ref float baseSourceLinearAX, ref int baseBundleA, ref int baseInnerA,
            ref float baseSourceLinearBX, ref int baseBundleB, ref int baseInnerB)
        {
            //We'll use the memory layout of the EntityVelocities struct. 
            //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
            ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearAX, laneIndex);
            var bundleIndexA = Unsafe.Add(ref baseBundleA, laneIndex);
            var innerIndexA = Unsafe.Add(ref baseInnerA, laneIndex);
            ref var targetLinearAX = ref Get(ref velocities[bundleIndexA].LinearVelocity.X, innerIndexA);
            targetLinearAX = sourceLinearAX;
            Unsafe.Add(ref targetLinearAX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count);

            ref var sourceLinearBX = ref Unsafe.Add(ref baseSourceLinearBX, laneIndex);
            var bundleIndexB = Unsafe.Add(ref baseBundleB, laneIndex);
            var innerIndexB = Unsafe.Add(ref baseInnerB, laneIndex);
            ref var targetLinearBX = ref Get(ref velocities[bundleIndexB].LinearVelocity.X, innerIndexB);
            targetLinearBX = sourceLinearBX;
            Unsafe.Add(ref targetLinearBX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, Vector<float>.Count);
            Unsafe.Add(ref targetLinearBX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 2 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearBX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 3 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearBX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 4 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearBX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearBX, 5 * Vector<float>.Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities3(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);
            ref var baseInnerA = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexA);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);

            ref var baseBundleB = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexB);
            ref var baseInnerB = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexB);
            ref var baseSourceLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            for (int i = 0; i < 4; ++i)
            {
                ScatterLane3(i, velocities, ref baseSourceLinearAX, ref baseBundleA, ref baseInnerA, ref baseSourceLinearBX, ref baseBundleB, ref baseInnerB);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ScatterLane4(int laneIndex, BodyVelocities[] velocities, ref float baseSourceLinearX, ref int baseBundle, ref int baseInner)
        {
            //We'll use the memory layout of the EntityVelocities struct. 
            //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
            ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearX, laneIndex);
            var bundleIndex = Unsafe.Add(ref baseBundle, laneIndex);
            var innerIndex = Unsafe.Add(ref baseInner, laneIndex);
            ref var targetLinearAX = ref Get(ref velocities[bundleIndex].LinearVelocity.X, innerIndex);
            targetLinearAX = sourceLinearAX;
            Unsafe.Add(ref targetLinearAX, Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 4 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count);
            Unsafe.Add(ref targetLinearAX, 5 * Vector<float>.Count) = Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities4(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);
            ref var baseInnerA = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexA);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);

            ref var baseBundleB = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexB);
            ref var baseInnerB = ref Unsafe.As<Vector<int>, int>(ref references.InnerIndexB);
            ref var baseSourceLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);
            
            for (int i = 0; i < references.Count; ++i)
            {
                ScatterLane4(i, velocities, ref baseSourceLinearAX, ref baseBundleA, ref baseInnerA);
                ScatterLane4(i, velocities, ref baseSourceLinearBX, ref baseBundleB, ref baseInnerB);
            }
        }
    }
}

