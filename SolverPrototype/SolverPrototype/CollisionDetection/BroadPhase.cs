using BEPUutilities2;
using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{
    public unsafe class BroadPhase : IDisposable
    {
        BoundingBox[] test = new BoundingBox[65536];
        BoundingBox* testPointer;
        GCHandle handle;
        public BroadPhase()
        {
            handle = GCHandle.Alloc(test, GCHandleType.Pinned);
            testPointer = (BoundingBox*)handle.AddrOfPinnedObject();
        }

        int Count;

        public int HighestPossibleLeafIndex { get { return Count - 1; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Add(CollidableReference collidable, ref Vector3 initialMin, ref Vector3 initialMax)
        {
            return Count++;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveAt(int index)
        {
            Debug.Assert(index >= 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBoundsPointers(int broadPhaseIndex, out float* minPointer, out float* maxPointer)
        {
            minPointer = &testPointer[broadPhaseIndex].Min.X;
            maxPointer = &testPointer[broadPhaseIndex].Max.X;
        }
        //Note that some systems (like the demos renderer bounding box line extractor) iterate over the leaves. However, they're not contiguously stored.
        //So, the user needs a way to know if the leaf index exists. Hence, a 'try' variant. If there happen to be more use cases for checking existence, a
        //dedicated 'leafexists' method would probably be a better idea.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetBoundsPointers(int broadPhaseIndex, out float* minPointer, out float* maxPointer)
        {
            GetBoundsPointers(broadPhaseIndex, out minPointer, out maxPointer);
            return true;
        }


        public void Dispose()
        {
            handle.Free();
        }
    }
}
