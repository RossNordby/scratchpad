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
    public struct DeferredConstraintAdder
    {
        Simulation simulation;

        struct WorkerCache
        {
            [StructLayout(LayoutKind.Sequential, Pack = 1)]
            struct PendingConstraint<TContactImpulses, TBodyHandles, TDescription> where TDescription : IConstraintDescription<TDescription>
            {
                public PairCacheIndex ConstraintCacheIndex;
                public TBodyHandles BodyHandles;
                public TDescription ConstraintDescription;
                public TContactImpulses Impulses;
            }

            Buffer<UntypedList> pendingConstraintsByType;
            int minimumConstraintCountPerCache;

            public WorkerCache(BufferPool pool, int minimumConstraintCountPerCache = 128)
            {
                //TODO: If we add in nonconvex manifolds with up to 8 contacts, this will need to change- we preallocate enough space to hold all possible narrowphase generated types.
                pool.SpecializeFor<UntypedList>().Take(16, out pendingConstraintsByType);
                this.minimumConstraintCountPerCache = minimumConstraintCountPerCache;
            }

            public unsafe void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(BufferPool pool, int manifoldConstraintType,
                PairCacheIndex constraintCacheIndex, ref TContactImpulses impulses, ref TBodyHandles bodyHandles, ref TDescription constraintDescription)
                where TDescription : IConstraintDescription<TDescription>
            {
                ref var cache = ref pendingConstraintsByType[manifoldConstraintType];
                var index = cache.Allocate<PendingConstraint<TContactImpulses, TBodyHandles, TDescription>>(minimumConstraintCountPerCache, pool);
                ref var pendingAdd = ref Unsafe.AsRef<PendingConstraint<TContactImpulses, TBodyHandles, TDescription>>(cache.Buffer.Memory + index);
                pendingAdd.BodyHandles = bodyHandles;
                pendingAdd.ConstraintCacheIndex = constraintCacheIndex;
                pendingAdd.Impulses = impulses;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe void CompletePendingAdd<TBodyHandles, TDescription, TContactImpulses>(ref UntypedList list, int narrowPhaseConstraintTypeId, Solver solver, ref PairCache pairCache)
                where TDescription : IConstraintDescription<TDescription>
            {
                if (list.Buffer.Allocated)
                {
                    ref var start = ref Unsafe.As<byte, PendingConstraint<TContactImpulses, TBodyHandles, TDescription>>(ref *list.Buffer.Memory);
                    for (int i = 0; i < list.Count; ++i)
                    {
                        ref var add = ref Unsafe.Add(ref start, i);
                        solver.Add(ref Unsafe.As<TBodyHandles, int>(ref add.BodyHandles), typeof(TBodyHandles) == typeof(TwoBodyHandles) ? 2 : 1, ref add.ConstraintDescription, out var handle);
                        solver.GetConstraintReference(handle, out var constraintReference);
                        pairCache.ScatterNewImpulses(narrowPhaseConstraintTypeId, ref constraintReference, ref add.Impulses);
                    }
                }
            }
            public void Flush(BufferPool pool, Solver solver, ref PairCache pairCache)
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
                CompletePendingAdd<TwoBodyHandles, ContactManifold1Constraint, ContactImpulses1>(ref pendingConstraintsByType[8 + 0 + 1], 8 + 0 + 1, solver, ref pairCache);
                CompletePendingAdd<TwoBodyHandles, ContactManifold4Constraint, ContactImpulses4>(ref pendingConstraintsByType[8 + 3 + 1], 8 + 3 + 1, solver, ref pairCache);
                for (int i = 0; i < pendingConstraintsByType.Length; ++i)
                {
                    if (pendingConstraintsByType[i].Buffer.Allocated)
                    {
                        pool.Return(ref pendingConstraintsByType[i].Buffer);
                    }
                }
                pool.SpecializeFor<UntypedList>().Return(ref pendingConstraintsByType);
            }
        }
        Buffer<WorkerCache> workerCaches;

        public void Initialize(Simulation simulation)
        {
            this.simulation = simulation;
        }

        public void Prepare(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            simulation.BufferPool.SpecializeFor<WorkerCache>().Take(threadCount, out workerCaches);
            for (int i = 0; i < threadCount; ++i)
            {
                ref var workerCache = ref workerCaches[i];
                workerCache = new WorkerCache(threadDispatcher == null ? simulation.BufferPool : threadDispatcher.GetThreadMemoryPool(i));
            }

        }

        public void AddConstraint<TBodyHandles, TDescription, TContactImpulses>(int workerIndex, BufferPool workerPool, int manifoldConstraintType,
            PairCacheIndex constraintCacheIndex, ref TContactImpulses impulses, TBodyHandles bodyHandles, ref TDescription constraintDescription)
            where TDescription : IConstraintDescription<TDescription>
        {
            workerCaches[workerIndex].AddConstraint(workerPool, manifoldConstraintType, constraintCacheIndex, ref impulses, ref bodyHandles, ref constraintDescription);
        }


        public void Flush(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            Debug.Assert(workerCaches.Length >= threadCount);
            for (int i = 0; i < threadCount; ++i)
            {
                workerCaches[i].Flush(threadDispatcher.GetThreadMemoryPool(i), simulation.Solver, ref simulation.NarrowPhase.PairCache);
            }
        }

    }
}
