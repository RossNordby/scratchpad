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
            //The user guarantees that the entity velocities are pointer safe.
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a really gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var linearX = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);
            ref var linearY = ref Unsafe.Add(ref linearX, Vector<float>.Count);
            ref var linearZ = ref Unsafe.Add(ref linearX, 2 * Vector<float>.Count);
            ref var angularX = ref Unsafe.Add(ref linearX, 3 * Vector<float>.Count);
            ref var angularY = ref Unsafe.Add(ref linearX, 4 * Vector<float>.Count);
            ref var angularZ = ref Unsafe.Add(ref linearX, 5 * Vector<float>.Count);

            //Grab the base references for the body indices.
            ref var baseBundle = ref Unsafe.As<Vector<int>, int>(ref bundleIndicesVector);
            //Note that the inner indices are encoded. They have the form:
            //(leading zeroes)(1 bit, null if set)(1 bit, kinematic if set)(actualIndex)
            //In other words, you can think of it as a 2 bit enum: 0 is dynamic, 1 is kinematic, 2 is null.
            //The actual index bit length depends on the Vector<int>.Count. 4 slots require 2 bits, 8 slots require 3, 16 slots require 4.
            //When gathering, we only care about whether the object is null. We don't gather from null bodies. (We still gather from kinematic bodies!)
            //TODO: Does those constant vectors fold?
            var bodyIsNullVector = Vector.BitwiseAnd(innerIndicesVector, new Vector<int>(Vector<int>.Count << 1));
            ref var bodyIsNull = ref Unsafe.As<Vector<int>, int>(ref bodyIsNullVector);
            var decodedInnerIndices = Vector.BitwiseAnd(innerIndicesVector, new Vector<int>(Solver.VectorMask));
            ref var baseInner = ref Unsafe.As<Vector<int>, int>(ref decodedInnerIndices);

            //TODO: 6 separate loops to contiguously march through memory? Right now all the written memory is close, but not contiguous.
            //Not sure if that has any value.
            for (int i = 0; i < bodyCount; ++i)
            {
                //When gathering velocities, there is no reason to consider null connections.
                //Null connections are those which are not actually associated with a kinematic or dynamic body.
                //They exist as a convenience for using multibody constraints that are connected to the 'world'.
                //(There is a question of whether such a feature should even exist. You can always use a kinematic object...
                //It's relatively cheap, though. The branch overhead is small.)
                if (Unsafe.Add(ref bodyIsNull, i) == 0)
                {
                    var bundleIndex = Unsafe.Add(ref baseBundle, i);
                    var innerIndex = Unsafe.Add(ref baseInner, i);
                    ref var bundle = ref allBodyVelocities[bundleIndex];
                    //TODO: Cache reference and use offsets rather than repeated As requests?
                    Unsafe.Add(ref linearX, i) = Get(ref bundle.LinearVelocity.X, innerIndex);
                    Unsafe.Add(ref linearY, i) = Get(ref bundle.LinearVelocity.Y, innerIndex);
                    Unsafe.Add(ref linearZ, i) = Get(ref bundle.LinearVelocity.Z, innerIndex);
                    Unsafe.Add(ref angularX, i) = Get(ref bundle.AngularVelocity.X, innerIndex);
                    Unsafe.Add(ref angularY, i) = Get(ref bundle.AngularVelocity.Y, innerIndex);
                    Unsafe.Add(ref angularZ, i) = Get(ref bundle.AngularVelocity.Z, innerIndex);
                }

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
        internal static unsafe void GatherVelocities2(BodyVelocities[] allBodyVelocities,
            ref Vector<int> bundleIndicesVector, ref Vector<int> innerIndicesVector, int bodyCount, ref BodyVelocities velocities)
        {
            //The user guarantees that the entity velocities are pointer safe.
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a really gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var linearX = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);
            ref var linearY = ref Unsafe.Add(ref linearX, Vector<float>.Count);
            ref var linearZ = ref Unsafe.Add(ref linearX, 2 * Vector<float>.Count);
            ref var angularX = ref Unsafe.Add(ref linearX, 3 * Vector<float>.Count);
            ref var angularY = ref Unsafe.Add(ref linearX, 4 * Vector<float>.Count);
            ref var angularZ = ref Unsafe.Add(ref linearX, 5 * Vector<float>.Count);

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
                Unsafe.Add(ref linearX, i) = bundleLinearX;
                Unsafe.Add(ref linearY, i) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                Unsafe.Add(ref linearZ, i) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                Unsafe.Add(ref angularX, i) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                Unsafe.Add(ref angularY, i) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                Unsafe.Add(ref angularZ, i) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);

            }

            //You may notice that this is a pretty dang heavy weight preamble to every solve iteration. Two points:
            //1) You should make sure to make this preamble as common as possible to all constraints, so that every time a new optimization becomes possible, you needn't
            //revisit every single constraint.
            //2) When there's a possibility to increase the quality of a constraint solve by doing a little more effort within the solve iteration, chances are it's
            //worth it. For cheap low quality solves on low-DOF constraints, the pregather and postscatter may take significantly longer than the actual math!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities2(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            GatherVelocities2(velocities, ref references.BundleIndexA, ref references.InnerIndexA, references.Count, ref velocitiesA);
            GatherVelocities2(velocities, ref references.BundleIndexB, ref references.InnerIndexB, references.Count, ref velocitiesB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void GatherVelocities3(BodyVelocities[] allBodyVelocities,
            ref Vector<int> bundleIndicesVector, ref Vector<int> innerIndicesVector, int bodyCount, ref BodyVelocities velocities)
        {
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            ref var linearX = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);

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
                ref var targetLinearX = ref Unsafe.Add(ref linearX, i);
                targetLinearX = bundleLinearX;
                Unsafe.Add(ref targetLinearX, Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, Vector<float>.Count);
                Unsafe.Add(ref targetLinearX, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 2 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearX, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 3 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearX, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 4 * Vector<float>.Count);
                Unsafe.Add(ref targetLinearX, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleLinearX, 5 * Vector<float>.Count);

            }

            //You may notice that this is a pretty dang heavy weight preamble to every solve iteration. Two points:
            //1) You should make sure to make this preamble as common as possible to all constraints, so that every time a new optimization becomes possible, you needn't
            //revisit every single constraint.
            //2) When there's a possibility to increase the quality of a constraint solve by doing a little more effort within the solve iteration, chances are it's
            //worth it. For cheap low quality solves on low-DOF constraints, the pregather and postscatter may take significantly longer than the actual math!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities3(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            GatherVelocities3(velocities, ref references.BundleIndexA, ref references.InnerIndexA, references.Count, ref velocitiesA);
            GatherVelocities3(velocities, ref references.BundleIndexB, ref references.InnerIndexB, references.Count, ref velocitiesB);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Scatter is actually harder than gathering. For every single constraint, we have to find the associated entities independently.
            //We can't precache the source pointers or anything.
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                //If there is no constraint here, we don't care about the contents of this or any of the remaining slots in this bundle.
                var bundleIndexA = references.BundleIndexA[i];
                if (bundleIndexA < 0)
                    break;
                //We'll use the memory layout of the EntityVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                var linearAX = (float*)Unsafe.AsPointer(ref velocities[bundleIndexA].LinearVelocity.X) + references.InnerIndexA[i];
                *linearAX = velocitiesA.LinearVelocity.X[i];
                *(linearAX + Vector<float>.Count) = velocitiesA.LinearVelocity.Y[i];
                *(linearAX + 2 * Vector<float>.Count) = velocitiesA.LinearVelocity.Z[i];
                *(linearAX + 3 * Vector<float>.Count) = velocitiesA.AngularVelocity.X[i];
                *(linearAX + 4 * Vector<float>.Count) = velocitiesA.AngularVelocity.Y[i];
                *(linearAX + 5 * Vector<float>.Count) = velocitiesA.AngularVelocity.Z[i];

                var linearBX = (float*)Unsafe.AsPointer(ref velocities[references.BundleIndexB[i]].LinearVelocity.X) + references.InnerIndexB[i];
                *linearBX = velocitiesB.LinearVelocity.X[i];
                *(linearBX + Vector<float>.Count) = velocitiesB.LinearVelocity.Y[i];
                *(linearBX + 2 * Vector<float>.Count) = velocitiesB.LinearVelocity.Z[i];
                *(linearBX + 3 * Vector<float>.Count) = velocitiesB.AngularVelocity.X[i];
                *(linearBX + 4 * Vector<float>.Count) = velocitiesB.AngularVelocity.Y[i];
                *(linearBX + 5 * Vector<float>.Count) = velocitiesB.AngularVelocity.Z[i];
            }
        }
    }
}
