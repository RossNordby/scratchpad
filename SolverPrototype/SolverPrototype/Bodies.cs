using BEPUutilities2;
using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    public struct BodyDescription
    {
        public Matrix3x3 InverseLocalInertiaTensor;
        public float InverseMass;
    }
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
        public BodyInertias[] InertiaBundles;
        public IdPool IdPool;
        public int BodyCount;

        public BodyReference this[int bodyHandleIndex]
        {
            get
            {
                return new BodyReference(bodyHandleIndex, this);
            }
        }

        public unsafe Bodies(int initialCapacityInBundles = 1024)
        {
            VelocityBundles = new BodyVelocities[initialCapacityInBundles];
            InertiaBundles = new BodyInertias[initialCapacityInBundles];

            int initialBodyCapacity = initialCapacityInBundles << Solver.VectorShift;
            IdPool = new IdPool(initialBodyCapacity);
            BodyHandles = new int[initialBodyCapacity];
            IndicesToHandleIndices = new int[initialBodyCapacity];
            //Initialize all the indices to -1.
            InitializeIndices(BodyHandles, 0, BodyHandles.Length);
            InitializeIndices(IndicesToHandleIndices, 0, IndicesToHandleIndices.Length);
        }

        unsafe static void InitializeIndices(int[] array, int start, int count)
        {
            fixed (int* pointer = array)
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
                Array.Resize(ref InertiaBundles, newSize);
                Array.Resize(ref IndicesToHandleIndices, newSize);
                Array.Resize(ref BodyHandles, newSize);

                InitializeIndices(BodyHandles, BodyCount, BodyCount);
                InitializeIndices(IndicesToHandleIndices, BodyCount, BodyCount);
            }
            var handleIndex = IdPool.Take();
            var index = BodyCount++;
            BodyHandles[handleIndex] = index;
            IndicesToHandleIndices[index] = handleIndex;

            Solver.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            GatherScatter.Scatter(ref InertiaBundles[bundleIndex], indexInBundle, ref bodyDescription.InverseLocalInertiaTensor, bodyDescription.InverseMass);

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
                InertiaBundles[index] = InertiaBundles[lastIndex];
                //Point the body handles at the new location.
                var lastHandleIndex = IndicesToHandleIndices[lastIndex];
                BodyHandles[lastHandleIndex] = index;
                IndicesToHandleIndices[index] = lastHandleIndex;
                IndicesToHandleIndices[lastIndex] = -1;
            }
            IdPool.Return(handleIndex);
            BodyHandles[handleIndex] = -1;
        }
    }

    /// <summary>
    /// Convenience structure for AOS-style access of body properties.
    /// </summary>
    public struct BodyReference
    {
        public readonly Bodies Bodies;
        public readonly int HandleIndex;

        public BodyReference(int bodyHandleIndex, Bodies bodies)
        {
            HandleIndex = bodyHandleIndex;
            Bodies = bodies;
        }

        public Matrix3x3 InverseInertiaTensor
        {
            get
            {
            }
        }



    }
}
