using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public class GatherScatter
    {

        //IMPLEMENTATION NOTES:

        //UNSAFE CASTS FOR VECTOR MEMORY ACCESS
        //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
        //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
        //This logic is coupled with the layout of the EntityVelocities and BodyReferences structs and makes assumptions about the memory layout of the types.
        //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
        //With any luck, it will be later enough that a proper solution exists.

        //'NULL' CONSTRAINT CONNECTIONS
        //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
        //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
        //The benefit of not having to read/write data is extremely weak, and often introducing null connections actually slows things down further
        //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
        //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
        //During scatters, there is a risk of false sharing on the reserved slot when running with multiple threads. 
        //However, we should take measures to avoid false sharing for all constraints. More measurement should be done to check the impact later.

        //MULTIBODY CONSTRAINTS AND MANUAL INLINING
        //Unfortunately, as of this writing, there still seems to be a very small value in manually inlining all involved bodies.
        //This is going to get pretty annoying if there are a variety of different constraint body counts. For runtime-defined N-body constraints,
        //we will likely end up just having a per-body gather, and that will be fine. I'm not gonna make 128 variants of these functions!

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

        /// <summary>
        /// Copies from one bundle lane to another. The bundle must be a contiguous block of Vector types.
        /// </summary>
        /// <typeparam name="T">Type of the copied bundles.</typeparam>
        /// <param name="sourceBundle">Source bundle of the data to copy.</param>
        /// <param name="sourceInnerIndex">Index of the lane within the source bundle.</param>
        /// <param name="targetBundle">Target bundle of the data to copy.</param>
        /// <param name="targetInnerIndex">Index of the lane within the target bundle.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector that isn't yet unrolled.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CopyLane<T>(ref T sourceBundle, int sourceInnerIndex, ref T targetBundle, int targetInnerIndex)
        {
            //Note the truncation. Currently used for some types that don't have a size evenly divisible by the Vector<int>.Count * sizeof(int).
            var sizeInInts = (Unsafe.SizeOf<T>() >> 2) & ~BundleIndexing.VectorMask;

            ref var sourceBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref sourceBundle), sourceInnerIndex);
            ref var targetBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref targetBundle), targetInnerIndex);

            targetBase = sourceBase;
            //Would be nice if this just auto-unrolled based on the size, considering the jit considers all the relevant bits to be constants!
            //Unfortunately, as of this writing, the jit doesn't.
            //for (int i = Vector<int>.Count; i < sizeInInts; i += Vector<int>.Count)
            //{
            //    Unsafe.Add(ref targetBase, i) = Unsafe.Add(ref sourceBase, i);
            //}

            //To compensate for the compiler, here we go:
            int offset = Vector<int>.Count;
            //8 wide unroll empirically chosen.
            while (offset + Vector<int>.Count * 8 <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + 4 * Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + 2 * Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset);
            }
        }

        /// <summary>
        /// Swaps lanes between two bundles. The bundle type must be a contiguous block of Vector types.
        /// </summary>
        /// <typeparam name="T">Type of the swapped bundles.</typeparam>
        /// <param name="bundleA">Source bundle of the data to copy.</param>
        /// <param name="innerIndexA">Index of the lane within the source bundle.</param>
        /// <param name="bundleB">Target bundle of the data to copy.</param>
        /// <param name="innerIndexB">Index of the lane within the target bundle.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        public static void SwapLanes<T>(ref T bundleA, int innerIndexA, ref T bundleB, int innerIndexB)
        {
            Debug.Assert((Unsafe.SizeOf<T>() & BundleIndexing.VectorMask) == 0,
                "This implementation doesn't truncate the count under the assumption that the type is evenly divisible by the bundle size." +
                "If you later use SwapLanes with a type that breaks this assumption, introduce a truncation as in CopyLanes.");
            var sizeInInts = Unsafe.SizeOf<T>() >> 2;
            ref var aBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref bundleA), innerIndexA);
            ref var bBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref bundleB), innerIndexB);
            for (int i = 0; i < sizeInInts; i += Vector<int>.Count)
            {
                var oldA = Unsafe.Add(ref aBase, i);
                ref var b = ref Unsafe.Add(ref bBase, i);
                Unsafe.Add(ref aBase, i) = b;
                b = oldA;
            }
        }

        /// <summary>
        /// Clears a bundle lane using the default value of the specified type. The bundle must be a contiguous block of Vector types, all sharing the same type,
        /// and the first vector must start at the address pointed to by the bundle reference.
        /// </summary>
        /// <typeparam name="TOuter">Type containing one or more Vectors.</typeparam>
        /// <typeparam name="TVector">Type of the vectors to clear.</typeparam>
        /// <param name="bundle">Target bundle to clear a lane in.</param>
        /// <param name="innerIndex">Index of the lane within the target bundle to clear.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClearLane<TOuter, TVector>(ref TOuter bundle, int innerIndex) where TVector : struct
        {
            //Note the truncation. This is used on some types that aren't evenly divisible.
            //This should be folded into a single constant by the jit.
            var sizeInElements = (Unsafe.SizeOf<TOuter>() / (Vector<TVector>.Count * Unsafe.SizeOf<TVector>())) * Unsafe.SizeOf<TVector>();
            ref var laneBase = ref Unsafe.Add(ref Unsafe.As<TOuter, TVector>(ref bundle), innerIndex);
            for (int i = 0; i < sizeInElements; i += Vector<int>.Count)
            {
                Unsafe.Add(ref laneBase, i) = default(TVector);
            }
        }
        /// <summary>
        /// Clears a bundle lane using the default value of the specified type. The bundle must be a contiguous block of Vector types, all sharing the same type,
        /// and the first vector must start at the address pointed to by the bundle reference.
        /// </summary>
        /// <typeparam name="TOuter">Type containing one or more Vectors.</typeparam>
        /// <typeparam name="TVector">Type of the vectors to clear.</typeparam>
        /// <param name="bundle">Target bundle to clear a lane in.</param>
        /// <param name="innerIndex">Index of the lane within the target bundle to clear.</param>
        /// <param name="count">Number of elements in the lane to clear.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClearLane<TOuter, TVector>(ref TOuter bundle, int innerIndex, int count) where TVector : struct
        {
            ref var laneBase = ref Unsafe.Add(ref Unsafe.As<TOuter, TVector>(ref bundle), innerIndex);
            for (int i = 0; i < count; ++i)
            {
                Unsafe.Add(ref laneBase, i * Vector<TVector>.Count) = default(TVector);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocities> velocities, ref UnpackedTwoBodyReferences references, out BodyVelocities velocitiesA, out BodyVelocities velocitiesB)
        {
            ref var baseLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var baseLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
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
        public static unsafe void ScatterVelocities(ref Buffer<BodyVelocities> velocities, ref UnpackedTwoBodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
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

                //Note that no attempt is made to avoid writing to kinematic or null velocities. Branching to avoid the write takes longer than just writing it.
                //No constraint should actually result in a change to the velocity of a kinematic or null body since they have infinite inertia.
            }
        }


        /// <summary>
        /// Gathers the inertia associated with the bodies of a bundle.
        /// </summary>
        /// <param name="bodyInertias">Set of all body inertias.</param>
        /// <param name="references">Body references of the constraint to gather.</param>
        /// <param name="inertiaA">Gathered inertia of the first body in the pair.</param>
        /// <param name="inertiaB">Gathered inertia of the second body in the pair.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GatherInertia(ref Buffer<BodyInertias> bodyInertias, ref UnpackedTwoBodyReferences references,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            //Note that there is no special handling of null or kinematic entities here. We gather them unconditionally.
            //Branches are not particularly cheap, especially when they mispredict. Better to just gather it regardless.            
            ref var targetBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.X.X);
            ref var targetBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.X.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                {
                    var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);

                    ref var bundleSlot = ref Get(ref bodyInertias[bundleIndexA].InverseInertiaTensor.X.X, innerIndexA);
                    ref var targetSlot = ref Unsafe.Add(ref targetBaseA, i);
                    targetSlot = bundleSlot;
                    Unsafe.Add(ref targetSlot, Vector<float>.Count) = Unsafe.Add(ref bundleSlot, Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 2 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 3 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 4 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 5 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 6 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 6 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 7 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 7 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 8 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 8 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 9 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 9 * Vector<float>.Count);
                }

                {
                    var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                    var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);

                    ref var bundleSlot = ref Get(ref bodyInertias[bundleIndexB].InverseInertiaTensor.X.X, innerIndexB);
                    ref var targetSlot = ref Unsafe.Add(ref targetBaseB, i);
                    targetSlot = bundleSlot;
                    Unsafe.Add(ref targetSlot, Vector<float>.Count) = Unsafe.Add(ref bundleSlot, Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 2 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 3 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 4 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 5 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 6 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 6 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 7 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 7 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 8 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 8 * Vector<float>.Count);
                    Unsafe.Add(ref targetSlot, 9 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 9 * Vector<float>.Count);
                }
            }
        }
        /// <summary>
        /// Sets a lane of a container of vectors, assuming that the vectors are contiguous.
        /// </summary>
        /// <typeparam name="T">Type of the values to copy into the container lane.</typeparam>
        /// <param name="startVector">First vector of the contiguous vector region to set a lane within.</param>
        /// <param name="innerIndex">Index of the lane within the vectors.</param>
        /// <param name="values">Reference to a contiguous set of values to copy into the vector lane slots.</param>
        /// <param name="valueCount">Number of values to iterate over.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetLane<T>(ref Vector<T> startVector, int innerIndex, ref T values, int valueCount) where T : struct
        {
            ref var lane = ref Get(ref startVector, innerIndex);
            lane = values;
            //Even if the jit recognizes the count as constant, it doesn't unroll anything. Could do it manually, like we did in CopyLane, but 
            //SetLane is typically not as performance sensitive.
            for (int vectorIndex = Vector<T>.Count; vectorIndex < valueCount; ++vectorIndex)
            {
                //The multiplication should become a shift; the jit recognizes the count as constant.
                Unsafe.Add(ref lane, vectorIndex * Vector<T>.Count) = Unsafe.Add(ref values, vectorIndex);
            }
        }


    }
}

