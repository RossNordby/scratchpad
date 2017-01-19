using BEPUutilities2;
using BEPUutilities2.ResourceManagement;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
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
        /// Remaps a body handle index to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public int[] BodyHandles;
        /// <summary>
        /// Remaps a body index to its handle index.
        /// </summary>
        public int[] IndicesToHandleIndices;

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



        public unsafe Bodies(int initialCapacityInBundles = 1024)
        {
            VelocityBundles = new BodyVelocities[initialCapacityInBundles];
            LocalInertiaBundles = new BodyInertias[initialCapacityInBundles];

            int initialBodyCapacity = initialCapacityInBundles << BundleIndexing.VectorShift;
            IdPool = new IdPool(initialBodyCapacity);
            BodyHandles = new int[initialBodyCapacity];
            IndicesToHandleIndices = new int[initialBodyCapacity];
            //Initialize all the indices to -1.
            InitializeIndices(BodyHandles, 0, BodyHandles.Length);
            InitializeIndices(IndicesToHandleIndices, 0, IndicesToHandleIndices.Length);
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
            if (BodyCount == BodyHandles.Length)
            {
                //Out of room; need to resize.
                var newSize = BodyHandles.Length << 1;
                Array.Resize(ref VelocityBundles, newSize);
                Array.Resize(ref LocalInertiaBundles, newSize);
                Array.Resize(ref IndicesToHandleIndices, newSize);
                Array.Resize(ref BodyHandles, newSize);

                InitializeIndices(BodyHandles, BodyCount, BodyCount);
                InitializeIndices(IndicesToHandleIndices, BodyCount, BodyCount);
            }
            var handleIndex = IdPool.Take();
            var index = BodyCount++;
            BodyHandles[handleIndex] = index;
            IndicesToHandleIndices[index] = handleIndex;

            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            GatherScatter.SetLane(ref LocalInertiaBundles[bundleIndex], indexInBundle, ref bodyDescription.LocalInertia);
            GatherScatter.SetLane(ref VelocityBundles[bundleIndex], indexInBundle, ref bodyDescription.Velocity);

            return handleIndex;
        }

        public void Remove(int handleIndex)
        {
            var index = BodyHandles[handleIndex];
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
                var lastHandleIndex = IndicesToHandleIndices[lastIndex];
                BodyHandles[lastHandleIndex] = index;
                IndicesToHandleIndices[index] = lastHandleIndex;
                IndicesToHandleIndices[lastIndex] = -1;
            }
            IdPool.Return(handleIndex);
            BodyHandles[handleIndex] = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void GetBundleIndices(int handleIndex, out int bundleIndex, out int innerIndex)
        {
            var index = BodyHandles[handleIndex];
            BundleIndexing.GetBundleIndices(index, out bundleIndex, out innerIndex);
        }
        public void SetVelocity(int handleIndex, ref BodyVelocity velocity)
        {
            GetBundleIndices(handleIndex, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref VelocityBundles[bundleIndex], innerIndex, ref velocity);
        }
        public void GetVelocity(int handleIndex, out BodyVelocity velocity)
        {
            GetBundleIndices(handleIndex, out var bundleIndex, out var innerIndex);
            GatherScatter.GetLane(ref VelocityBundles[bundleIndex], innerIndex, out velocity);
        }
        public void SetLocalInertia(int handleIndex, ref BodyInertia inertia)
        {
            GetBundleIndices(handleIndex, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref LocalInertiaBundles[bundleIndex], innerIndex, ref inertia);
        }
        public void GetLocalInertia(int handleIndex, ref BodyInertia inertia)
        {
            GetBundleIndices(handleIndex, out var bundleIndex, out var innerIndex);
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
    }
}
