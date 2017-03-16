using BEPUutilities2;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Body data is stored in AOSOA for the integration step.
    /// From the solver's perspective, some form of gather is required for velocities regardless of the layout, so it might as well be optimal for some other stage.
    /// We also reuse this layout for storing constraint space velocities.
    /// </summary>
    public struct BodyVelocities
    {
        public Vector3Wide LinearVelocity;
        public Vector3Wide AngularVelocity;
    }


    public struct BodyInertias
    {
        public Matrix3x3Wide InverseInertiaTensor;
        public Vector<float> InverseMass;
    }
    /// <summary>
    /// A constraint's body references. Stored separately from the iteration data since it is accessed by both the prestep and solve.
    /// Two address streams isn't much of a problem for prefetching.
    /// </summary>
    public struct TwoBodyReferences
    {
        //Unfortunately, there does not exist any Vector<int>.Shift instruction yet, so we cannot efficiently derive the bundle and inner indices from the 'true' indices on the fly.
        //Instead, group references are preconstructed and cached in a nonvectorized way.
        public Vector<int> BundleIndexA;
        public Vector<int> InnerIndexA;
        public Vector<int> BundleIndexB;
        public Vector<int> InnerIndexB;
        public int Count;

        //TODO: there may be an argument to make the count a full Vector<int> for padding reasons. We'd only ever access one component, but if alignment becomes an issue it could be a net win.
        //It would look something like this. A bit awkward due to the restrictions on ref returns, but functionally workable. This isn't something an external user is expected to deal with
        //so as long as it is speedy, it doesn't matter.
        //Vector<int> paddedCount;
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static ref int PaddedCount(ref BodyReferences references)
        //{
        //    return ref Unsafe.As<Vector<int>, int>(ref references.paddedCount);
        //}
    }
    public struct BodyDescription
    {
        public BodyInertia LocalInertia;
        public BodyVelocity Velocity;
    }
    public struct BodyInertia
    {
        public Matrix3x3 InverseInertiaTensor;
        public float InverseMass;
    }
    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;
    }
    /// <summary>
    /// Collection of allocated bodies. For now, it is assumed that all bodies are active and dynamic.
    /// </summary>
    public class Bodies
    {
        /// <summary>
        /// Remaps a body handle to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public int[] HandleToIndex;
        /// <summary>
        /// Remaps a body index to its handle.
        /// </summary>
        public int[] IndexToHandle;

        public BodyVelocities[] VelocityBundles;
        public BodyInertias[] LocalInertiaBundles;
        //TODO: While our current tests do not actually integrate orientation, when we do, we'll need to also update the world inertia.
        //The constraints will need to gather from the transformed inertia rather than from the local variants.
        //Note that the inverse mass is included in the BodyInertias bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        //public BodyInertias[] InertiaBundles;
        public IdPool IdPool;
        public int BodyCount;

        public unsafe Bodies(int initialCapacity = 4096)
        {
            var initialCapacityInBundles = BundleIndexing.GetBundleCount(initialCapacity);
            VelocityBundles = new BodyVelocities[initialCapacityInBundles];
            LocalInertiaBundles = new BodyInertias[initialCapacityInBundles];

            IdPool = new IdPool(initialCapacity);
            HandleToIndex = new int[initialCapacity];
            IndexToHandle = new int[initialCapacity];
            //Initialize all the indices to -1.
            InitializeIndices(HandleToIndex, 0, HandleToIndex.Length);
            InitializeIndices(IndexToHandle, 0, IndexToHandle.Length);
        }

        unsafe static void InitializeIndices(int[] array, int start, int count)
        {
            fixed (int* pointer = &array[start])
            {
                Unsafe.InitBlock(pointer, 0xFF, (uint)(sizeof(int) * count));
            }
        }

        public unsafe int Add(ref BodyDescription bodyDescription)
        {
            if (BodyCount == HandleToIndex.Length)
            {
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                Array.Resize(ref VelocityBundles, newSize);
                Array.Resize(ref LocalInertiaBundles, newSize);
                Array.Resize(ref IndexToHandle, newSize);
                Array.Resize(ref HandleToIndex, newSize);

                InitializeIndices(HandleToIndex, BodyCount, BodyCount);
                InitializeIndices(IndexToHandle, BodyCount, BodyCount);
            }
            var handle = IdPool.Take();
            var index = BodyCount++;
            HandleToIndex[handle] = index;
            IndexToHandle[index] = handle;

            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            GatherScatter.SetLane(ref LocalInertiaBundles[bundleIndex], indexInBundle, ref bodyDescription.LocalInertia);
            GatherScatter.SetLane(ref VelocityBundles[bundleIndex], indexInBundle, ref bodyDescription.Velocity);

            return handle;
        }

        public void Remove(int handle)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            if (index == -1)
            {
                throw new ArgumentException("Handle index not associated with any body; cannot remove.");
            }
            //Move the last body into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            if (BodyCount > 0)
            {
                var lastIndex = --BodyCount;
                //Copy the memory state of the last element down.
                VelocityBundles[index] = VelocityBundles[lastIndex];
                LocalInertiaBundles[index] = LocalInertiaBundles[lastIndex];
                //Point the body handles at the new location.
                var lastHandle = IndexToHandle[lastIndex];
                HandleToIndex[lastHandle] = index;
                IndexToHandle[index] = lastHandle;
                IndexToHandle[lastIndex] = -1;
            }
            IdPool.Return(handle);
            HandleToIndex[handle] = -1;
        }

        [Conditional("DEBUG")]
        void ValidateHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle >= HandleToIndex.Length || HandleToIndex[handle] < 0 || IndexToHandle[HandleToIndex[handle]] == handle,
                "If a handle exists, both directions should match.");
        }
        [Conditional("DEBUG")]
        void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle < HandleToIndex.Length && HandleToIndex[handle] >= 0 && IndexToHandle[HandleToIndex[handle]] == handle,
                "If a handle exists, both directions should match.");
        }
        public bool Contains(int handle)
        {
            ValidateHandle(handle);
            return handle < HandleToIndex.Length && HandleToIndex[handle] >= 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void GetBundleIndices(int handle, out int bundleIndex, out int innerIndex)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out bundleIndex, out innerIndex);
        }
        public void SetVelocity(int handle, ref BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref VelocityBundles[bundleIndex], innerIndex, ref velocity);
        }
        public void GetVelocity(int handle, out BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.GetLane(ref VelocityBundles[bundleIndex], innerIndex, out velocity);
        }
        public void SetLocalInertia(int handle, ref BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref LocalInertiaBundles[bundleIndex], innerIndex, ref inertia);
        }
        public void GetLocalInertia(int handle, ref BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.GetLane(ref LocalInertiaBundles[bundleIndex], innerIndex, out inertia);
        }

        public float GetBodyEnergyHeuristic()
        {
            float accumulated = 0;
            var lastBundleIndex = (BodyCount - 1) >> BundleIndexing.VectorShift;
            //Mask away the unused lanes. We're modifying the actual velocities; that's valid because they're unused.
            var lastBundleCount = BodyCount - (lastBundleIndex << BundleIndexing.VectorShift);
            var zeroVelocity = new BodyVelocity();
            for (int i = lastBundleCount; i < Vector<float>.Count; ++i)
            {
                GatherScatter.SetLane(ref VelocityBundles[lastBundleIndex], lastBundleCount, ref zeroVelocity);
            }
            for (int bundleIndex = 0; bundleIndex <= lastBundleIndex; ++bundleIndex)
            {
                Vector3Wide.Dot(ref VelocityBundles[bundleIndex].LinearVelocity, ref VelocityBundles[bundleIndex].LinearVelocity, out var linearDot);
                Vector3Wide.Dot(ref VelocityBundles[bundleIndex].AngularVelocity, ref VelocityBundles[bundleIndex].AngularVelocity, out var angularDot);

                accumulated += Vector.Dot(linearDot, Vector<float>.One) + Vector.Dot(angularDot, Vector<float>.One);
            }
            return accumulated;
        }


        /// <summary>
        /// Swaps the memory of two bodies. Indexed by memory slot, not by handle index.
        /// </summary>
        /// <param name="slotA">Memory slot of the first body to swap.</param>
        /// <param name="slotB">Memory slot of the second body to swap.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Swap(int slotA, int slotB)
        {
            HandleToIndex[IndexToHandle[slotA]] = slotB;
            HandleToIndex[IndexToHandle[slotB]] = slotA;
            var oldHandleA = IndexToHandle[slotA];
            IndexToHandle[slotA] = IndexToHandle[slotB];
            IndexToHandle[slotB] = oldHandleA;
            BundleIndexing.GetBundleIndices(slotA, out var bundleA, out var innerA);
            BundleIndexing.GetBundleIndices(slotB, out var bundleB, out var innerB);
            GatherScatter.SwapLanes(ref VelocityBundles[bundleA], innerA, ref VelocityBundles[bundleB], innerB);
            GatherScatter.SwapLanes(ref LocalInertiaBundles[bundleA], innerA, ref LocalInertiaBundles[bundleB], innerB);
        }
    }
}
