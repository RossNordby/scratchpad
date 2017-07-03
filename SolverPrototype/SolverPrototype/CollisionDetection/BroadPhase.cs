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


        public void Dispose()
        {
            handle.Free();
        }
    }
}
