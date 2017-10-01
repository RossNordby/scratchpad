using System;
using System.Collections.Generic;
using System.Text;
using BEPUutilities2;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System.Runtime.CompilerServices;
using BEPUutilities2.Memory;
using System.Runtime.InteropServices;
using System.Diagnostics;

namespace SolverPrototype.CollisionDetection
{
    /// <summary>
    /// When notified of a new constraint, immediately adds it to the solver.
    /// </summary>
    public partial class NarrowPhase<TCallbacks>
    {
        internal struct PendingConstraintAddCache
        {
            BufferPool pool;
            struct PendingConstraint<TBodyHandles, TDescription, TContactImpulses> where TDescription : IConstraintDescription<TDescription>
            {
                public PairCacheIndex ConstraintCacheIndex;
                public TBodyHandles BodyHandles;
                public TDescription ConstraintDescription;
                public TContactImpulses Impulses;
            }

            //TODO: If we add in nonconvex manifolds with up to 8 contacts, this will need to change- we preallocate enough space to hold all possible narrowphase generated types.
            const int constraintTypeCount = 16;
            Buffer<UntypedList> pendingConstraintsByType;
            int minimumConstraintCountPerCache;

            public PendingConstraintAddCache(BufferPool pool, int minimumConstraintCountPerCache = 128)
            {
                this.pool = pool;
                pool.SpecializeFor<UntypedList>().Take(constraintTypeCount, out pendingConstraintsByType);
                //Have to clear the memory before use to avoid trash data sticking around.
                pendingConstraintsByType.Clear(0, constraintTypeCount);
                this.minimumConstraintCountPerCache = minimumConstraintCountPerCache;
            }

            public unsafe void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(int manifoldConstraintType,
                PairCacheIndex constraintCacheIndex, TBodyHandles bodyHandles, ref TDescription constraintDescription, ref TContactImpulses impulses)
                where TDescription : IConstraintDescription<TDescription>
            {
                ref var cache = ref pendingConstraintsByType[manifoldConstraintType];
                var index = cache.Allocate<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(minimumConstraintCountPerCache, pool);
                ref var pendingAdd = ref Unsafe.AsRef<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(cache.Buffer.Memory + index);
                pendingAdd.BodyHandles = bodyHandles;
                pendingAdd.ConstraintDescription = constraintDescription;
                pendingAdd.ConstraintCacheIndex = constraintCacheIndex;
                pendingAdd.Impulses = impulses;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void CompletePendingAdds<TBodyHandles, TDescription, TContactImpulses>(ref UntypedList list, int narrowPhaseConstraintTypeId, Simulation simulation, ref PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                if (list.Buffer.Allocated)
                {
                    ref var start = ref Unsafe.As<byte, PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>(ref *list.Buffer.Memory);
                    Debug.Assert(list.Buffer.Length > Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    Debug.Assert(list.ByteCount == Unsafe.SizeOf<PendingConstraint<TBodyHandles, TDescription, TContactImpulses>>() * list.Count);
                    for (int i = 0; i < list.Count; ++i)
                    {
                        ref var add = ref Unsafe.Add(ref start, i);
                        var handle = simulation.Add(ref Unsafe.As<TBodyHandles, int>(ref add.BodyHandles), typeof(TBodyHandles) == typeof(TwoBodyHandles) ? 2 : 1, ref add.ConstraintDescription);
                        pairCache.CompleteConstraintAdd(simulation.Solver, ref add.Impulses, add.ConstraintCacheIndex, handle);
                    }
                    pool.Return(ref list.Buffer);
                }
            }
            public void Flush(Simulation simulation, ref PairCache pairCache)
            {
                //This is going to be pretty horrible!
                //There is no type information beyond the index of the cache.
                //In other words, we basically have to do a switch statement (or equivalently manually unrolled loop) to gather objects of the proper type from it. 

                //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                //Constraints cover 16 possible cases:
                //1-4 contacts: 0x3
                //convex vs nonconvex: 0x4
                //1 body versus 2 body: 0x8
                //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                CompletePendingAdds<TwoBodyHandles, ContactManifold1Constraint, ContactImpulses1>(ref pendingConstraintsByType[8 + 0 + 0], 8 + 0 + 0, simulation, ref pairCache);
                CompletePendingAdds<TwoBodyHandles, ContactManifold4Constraint, ContactImpulses4>(ref pendingConstraintsByType[8 + 0 + 3], 8 + 0 + 3, simulation, ref pairCache);
                pool.SpecializeFor<UntypedList>().Return(ref pendingConstraintsByType);
            }

            internal int CountConstraints()
            {
                int count = 0;
                for (int i = 0; i < constraintTypeCount; ++i)
                {
                    count += pendingConstraintsByType[i].Count;
                }
                return count;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(int workerIndex, int manifoldConstraintType,
            PairCacheIndex constraintCacheIndex, ref TContactImpulses impulses, TBodyHandles bodyHandles, ref TDescription constraintDescription)
            where TDescription : IConstraintDescription<TDescription>
        {
            overlapWorkers[workerIndex].PendingConstraints.AddConstraint(manifoldConstraintType, constraintCacheIndex, bodyHandles, ref constraintDescription, ref impulses);
        }


        void FlushPendingConstraintAdds(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            Debug.Assert(overlapWorkers.Length >= threadCount);

            int constraintCount = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                constraintCount += overlapWorkers[i].PendingConstraints.CountConstraints();
            }
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].PendingConstraints.Flush(Simulation, ref PairCache);
            }
            var end = Stopwatch.GetTimestamp();
            if (constraintCount > 0)
                Console.WriteLine($"Flush time (us): {1e6 * (end - start) / Stopwatch.Frequency}, constraint count: {constraintCount}, time per constraint (ns): {1e9 * (end - start) / (constraintCount * Stopwatch.Frequency)}");
        }

    }
}
