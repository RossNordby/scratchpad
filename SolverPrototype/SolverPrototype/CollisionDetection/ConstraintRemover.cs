using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    //generiiiiiiiiiiiics
    using Batches = QuickDictionary<
        TypeBatchIndex,
        QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>,
        Buffer<TypeBatchIndex>,
        Buffer<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>,
        Buffer<int>, TypeBatchIndexComparer>;

    struct WorkerBatchReference
    {
        public short WorkerIndex;
        public short WorkerBatchIndex;
    }
    struct TypeBatchIndex
    {
        public short TypeBatch;
        public short Batch;
    }
    struct TypeBatchIndexComparer : IEqualityComparerRef<TypeBatchIndex>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref TypeBatchIndex a, ref TypeBatchIndex b)
        {
            return Unsafe.As<TypeBatchIndex, int>(ref a) == Unsafe.As<TypeBatchIndex, int>(ref b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref TypeBatchIndex item)
        {
            return Unsafe.As<TypeBatchIndex, int>(ref item);
        }
    }
    public enum NarrowPhaseFlushJobType
    {
        RemoveConstraintsFromBodyLists,
        RemoveConstraintFromTypeBatch,
        ReturnConstraintHandlesToPool,
        FlushPairCacheChanges
    }

    public struct NarrowPhaseFlushJob
    {
        public NarrowPhaseFlushJobType Type;
        public int Start;
        public int End;
    }

    /// <summary>
    /// Accumulates constraints to remove from multiple threads, and efficiently removes them all as a batch.
    /// </summary>
    public class ConstraintRemover
    {
        Solver solver;
        BufferPool pool;


        struct WorkerCache
        {
            BufferPool pool;
            internal QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>> Batches;
            internal QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>> BatchHandles;
            int minimumCapacityPerBatch;

            public WorkerCache(BufferPool pool, int batchCapacity, int minimumCapacityPerBatch)
            {
                this.pool = pool;
                Debug.Assert(minimumCapacityPerBatch > 0);
                this.minimumCapacityPerBatch = minimumCapacityPerBatch;
                QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>>.Create(pool.SpecializeFor<TypeBatchIndex>(), batchCapacity, out Batches);
                QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>>.Create(pool.SpecializeFor<QuickList<int, Buffer<int>>>(), batchCapacity, out BatchHandles);
            }

            public void Add(int constraintHandle, TypeBatchIndex typeBatchIndex)
            {
                int index = -1;
                //Note that we just scan for an existing type batch entry that matches the new one.
                //Given the limited number of constraint types and removes happening, this brute force approach will tend to be faster than a full dictionary.
                //(TODO: That's worth testing.)
                for (int i = 0; i < Batches.Count; ++i)
                {
                    if (Unsafe.As<TypeBatchIndex, int>(ref typeBatchIndex) == Unsafe.As<TypeBatchIndex, int>(ref Batches[i]))
                    {
                        //Guarantee we can hold the new handle before we add it. Ensuring capacity beforehand avoids the need for creating pools, and shares the add
                        //with the other no-index-found path.
                        ref var handles = ref BatchHandles[i];
                        if (handles.Span.Length == handles.Count)
                        {
                            //Buffers resize by powers of 2, no worry of incremental resizes.
                            handles.EnsureCapacity(handles.Count + 1, pool.SpecializeFor<int>());
                        }
                        index = i;
                        break;
                    }
                }
                if (index == -1)
                {
                    index = Batches.Count;
                    if (Batches.Span.Length == Batches.Count)
                    {
                        var newCount = Batches.Count + 1;
                        Batches.EnsureCapacity(newCount, pool.SpecializeFor<TypeBatchIndex>());
                        BatchHandles.EnsureCapacity(newCount, pool.SpecializeFor<QuickList<int, Buffer<int>>>());
                    }
                    Batches.AllocateUnsafely() = typeBatchIndex;
                    ref var handles = ref BatchHandles.AllocateUnsafely();
                    QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), minimumCapacityPerBatch, out handles);
                }
                BatchHandles[index].AllocateUnsafely() = constraintHandle;
            }

            public void Dispose()
            {
                Batches.Dispose(pool.SpecializeFor<TypeBatchIndex>());
                var intPool = pool.SpecializeFor<int>();
                for (int i = 0; i < BatchHandles.Count; ++i)
                {
                    BatchHandles[i].Dispose(intPool);
                }
                BatchHandles.Dispose(pool.SpecializeFor<QuickList<int, Buffer<int>>>());
                this = new WorkerCache();
            }
        }

        int previousCapacityPerBatch;
        int previousBatchCapacity;
        float previousCapacityMultiplier;
        int minimumConstraintCapacity;
        int minimumTypeCapacity;
        Array<WorkerCache> workerCaches; //there is a reference within the worker cache for the pool, so this can't be a buffer.
        IThreadDispatcher dispatcher;

        public ConstraintRemover(BufferPool pool, Solver solver, int minimumTypeCapacity = 4, int minimumRemovalCapacity = 128, float previousCapacityMultiplier = 1.25f)
        {
            this.pool = pool;
            this.solver = solver;
            this.minimumConstraintCapacity = minimumRemovalCapacity;
            this.minimumTypeCapacity = minimumTypeCapacity;
            this.previousCapacityMultiplier = previousCapacityMultiplier;
        }


        public void Prepare(IThreadDispatcher dispatcher)
        {
            this.dispatcher = dispatcher;
            if (dispatcher != null)
            {
                //There aren't going to be that many workers or resizes of this array, so a managed reference is fine. Makes the storage of the buffer pool easier.
                if (workerCaches.Length < dispatcher.ThreadCount)
                {
                    new PassthroughArrayPool<WorkerCache>().Take(dispatcher.ThreadCount, out workerCaches);
                }
                var batchCapacity = (int)Math.Max(minimumTypeCapacity, previousBatchCapacity * previousCapacityMultiplier);
                var capacityPerBatch = (int)Math.Max(minimumConstraintCapacity, previousCapacityPerBatch * previousCapacityMultiplier);
                for (int i = 0; i < dispatcher.ThreadCount; ++i)
                {
                    //Note the use of per-thread pools. It is possible for the workers to resize the collections.
                    workerCaches[i] = new WorkerCache(dispatcher.GetThreadMemoryPool(i), batchCapacity, capacityPerBatch);
                }


            }
        }

        //The general idea for multithreaded constraint removal is that there are varying levels of parallelism across the process.
        //You can't call solver.RemoveConstraint from multiple threads without locks, and we don't want to pay the price of constant syncs.
        //So instead, we identify the pieces of pretty-much-sequential work, and stick them into locally sequential jobs.
        //We then run them alongside other more parallel work. The hope is that the parallel work will fill in the gaps, balancing the work across all the threads
        //and limiting the worst case.

        //It's important to note that this is massively overkill when the simulation is only dealing with <20 removals. It's likely slower than just doing regular removes
        //in sequence at that point- sequential removes would cost around 5us in that case, so any kind of multithreaded overhead can overwhelm the work being done.
        //Doubling the cost of the best case, resulting in handfuls of wasted microseconds, isn't concerning (and we could special case it if we really wanted to).
        //Cutting the cost of the worst case when thousands of constraints get removed by a factor of ~ThreadCount is worth this complexity. Frame spikes are evil!

        Batches batches;
        public void CreateFlushJobs(ref QuickList<NarrowPhaseFlushJob, Buffer<NarrowPhaseFlushJob>> jobs)
        {
            //Add the locally sequential jobs. Put them first in the hope that the usually-smaller per-typebatch jobs will balance out the remainder of the work.
            jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintsFromBodyLists, Start = batches.Count }, pool.SpecializeFor<NarrowPhaseFlushJob>());
            jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.ReturnConstraintHandlesToPool, Start = batches.Count }, pool.SpecializeFor<NarrowPhaseFlushJob>());
            //TODO: For deactivation, you don't actually want to create a body list removal request. The bodies would be getting removed, so it would be redundant.
            //Simple enough to adapt for that use case later. Probably need to get rid of the narrow phase specific reference.

            //Accumulate the set of unique type batches in a contiguous list so we can easily execute multithreaded jobs over them.
            //Note that we're not actually copying over the contents of the per-worker lists here- just storing a reference to the per-worker lists.
            Batches.Create(pool.SpecializeFor<TypeBatchIndex>(), pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>(), pool.SpecializeFor<int>(),
                128, 3, out batches);
            var typeBatchIndexPool = pool.SpecializeFor<TypeBatchIndex>();
            var quickListPool = pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>();
            var intPool = pool.SpecializeFor<int>();

            for (int i = 0; i < dispatcher.ThreadCount; ++i)
            {
                ref var cache = ref workerCaches[i];
                for (int j = 0; j < cache.Batches.Count; ++j)
                {
                    var batchIndex = batches.IndexOf(cache.Batches[j]);
                    if (batchIndex >= 0)
                    {
                        ref var slot = ref batches.Values[batchIndex].AllocateUnsafely();
                        slot.WorkerIndex = (short)i;
                        slot.WorkerBatchIndex = (short)j;
                    }
                    else
                    {
                        //This worker batch doesn't exist in the combined set. Add it now.
                        QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>.Create(pool.SpecializeFor<WorkerBatchReference>(), dispatcher.ThreadCount - i, out var references);
                        WorkerBatchReference reference;
                        reference.WorkerIndex = (short)i;
                        reference.WorkerBatchIndex = (short)j;
                        references.AddUnsafely(reference);
                        jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintFromTypeBatch, Start = batches.Count }, pool.SpecializeFor<NarrowPhaseFlushJob>());
                        batches.Add(ref cache.Batches[j], ref references, typeBatchIndexPool, quickListPool, intPool);
                    }
                }
            }
        }

        public void RemoveConstraintsFromBodyLists(Solver solver, ConstraintConnectivityGraph graph)
        {
            ConstraintGraphRemovalEnumerator enumerator;
            enumerator.graph = graph;
            //While this could technically be internally multithreaded, it would be pretty complex- you would have to do one dispatch per solver.Batches batch
            //to guarantee that no two threads hit the same body constraint list at the same time. 
            //That is more complicated and would almost certainly be slower than this locally sequential version.
            for (int workerIndex = 0; workerIndex < dispatcher.ThreadCount; ++workerIndex)
            {
                ref var workerCache = ref workerCaches[workerIndex];
                for (int batchIndex = 0; batchIndex < workerCache.BatchHandles.Count; ++batchIndex)
                {
                    ref var handles = ref workerCache.BatchHandles[batchIndex];
                    for (int handleIndex = 0; handleIndex < handles.Count; ++handleIndex)
                    {
                        var handle = handles[handleIndex];
                        enumerator.constraintHandle = handle;
                        solver.EnumerateConnectedBodyIndices(handle, ref enumerator);
                    }
                }
            }
        }

        public void ReturnConstraintHandlesToPool()
        {
            for (int workerIndex = 0; workerIndex < dispatcher.ThreadCount; ++workerIndex)
            {
                ref var workerCache = ref workerCaches[workerIndex];
                for (int batchIndex = 0; batchIndex < workerCache.BatchHandles.Count; ++batchIndex)
                {
                    ref var handles = ref workerCache.BatchHandles[batchIndex];
                    for (int handleIndex = 0; handleIndex < handles.Count; ++handleIndex)
                    {
                        solver.handlePool.Return(handles[handleIndex]);
                    }
                }
            }
        }

        QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>> removedTypeBatches;
        SpinLock removedTypeBatchLocker = new SpinLock();
        public void RemoveConstraintsFromBatch(int index, Solver solver)
        {
            ref var batchReferences = ref batches.Values[index];
            for (int i = 0; i < batchReferences.Count; ++i)
            {
                ref var reference = ref batchReferences[i];
                ref var workerCache = ref workerCaches[reference.WorkerIndex];
                ref var batch = ref workerCache.Batches[reference.WorkerBatchIndex];
                ref var handles = ref workerCache.BatchHandles[reference.WorkerBatchIndex];
                for (int j = 0; j < handles.Count; ++j)
                {
                    solver.Batches[batch.Batch].TypeBatches[batch.TypeBatch].Remove()
                }
            }
        }

        struct TypeBatchComparer : IComparerRef<TypeBatchIndex>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref TypeBatchIndex a, ref TypeBatchIndex b)
            {
                return Unsafe.As<TypeBatchIndex, int>(ref b).CompareTo(Unsafe.As<TypeBatchIndex, int>(ref a));
            }
        }

        public void Postflush()
        {
            if (removedTypeBatches.Count > 0)
            {
                //Get rid of any type batches (and constraint batches) that became empty due to removals.
                //Sort the removed batches from highest to lowest so that higher index type batches and constraint batches get removed first.
                //This allows remove-by-pulling-last-index without corrupting other indices.
                var comparer = new TypeBatchComparer();
                QuickSort.Sort(ref removedTypeBatches[0], 0, removedTypeBatches.Count - 1, ref comparer);
                for (int i = 0; i < removedTypeBatches.Count; ++i)
                {

                }
            }
            removedTypeBatches.Dispose(pool.SpecializeFor<TypeBatchIndex>());

            //Get rid of the worker batch reference collections.
            for (int i = 0; i < batches.Count; ++i)
            {
                batches.Values[i].Dispose(pool.SpecializeFor<WorkerBatchReference>());
            }
            batches.Dispose(pool.SpecializeFor<TypeBatchIndex>(), pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>(), pool.SpecializeFor<int>());

            //Get rid of the worker cache allocations and store the capacities for next frame initialization.
            previousCapacityPerBatch = 0;
            for (int i = 0; i < dispatcher.ThreadCount; ++i)
            {
                ref var workerCache = ref workerCaches[i];
                for (int j = 0; j < workerCache.BatchHandles.Count; ++j)
                {
                    if (previousCapacityPerBatch < workerCache.BatchHandles[j].Count)
                        previousCapacityPerBatch = workerCache.BatchHandles[j].Count;
                }
                if (previousBatchCapacity < workerCache.BatchHandles.Count)
                    previousBatchCapacity = workerCache.BatchHandles.Count;
                workerCache.Dispose();
            }
            dispatcher = null;
        }

        public void EnqueueRemoval(int workerIndex, int constraintHandle)
        {
            ref var constraint = ref solver.HandleToConstraint[constraintHandle];
            TypeBatchIndex typeBatchIndex;
            //Parallel removes are guaranteed to not change the constraint indices until all removes complete, so we can precache the type batch index here.
            //This allows us to collect the constraints to remove by type batch. Removes across type batches can proceed in parallel.
            typeBatchIndex.Batch = (short)constraint.BatchIndex;
            typeBatchIndex.TypeBatch = (short)solver.Batches[constraint.BatchIndex].TypeIndexToTypeBatchIndex[constraint.TypeId];
            workerCaches[workerIndex].Add(constraintHandle, typeBatchIndex);
        }
    }
}
