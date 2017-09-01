using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{
    //would you care for some generics
    using OverlapMapping = QuickDictionary<CollidablePair, CollidablePairPointers, Buffer<CollidablePair>, Buffer<CollidablePairPointers>, Buffer<int>, CollidablePairComparer>;

    public struct CollidablePairPointers
    {
        /// <summary>
        /// A narrowphase-specific type and index into the pair cache's constraint data set. Collision pairs which have no associated constraint, either 
        /// because no contacts were generated or because the constraint was filtered, will have a nonexistent ConstraintCache.
        /// </summary>
        public PairCacheIndex ConstraintCache;
        /// <summary>
        /// A narrowphase-specific type and index into a batch of custom data for the pair. Many types do not use any supplementary data, but some make use of temporal coherence
        /// to accelerate contact generation.
        /// </summary>
        public PairCacheIndex CollisionDetectionCache;
    }

    public struct ConstraintCache1
    {
        public int ConstraintHandle;
        //No need for feature ids in 1 contact constraints. Just assume the old impulse can be reused regardless. Reasonably good guess.
    }
    public struct ConstraintCache2
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
    }
    public struct ConstraintCache3
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
    }
    public struct ConstraintCache4
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
    }

    public interface IPairCacheEntry
    {
        int TypeId { get; }
    }

    /// <summary>
    /// The cached pair data created by a single worker during the last execution of narrow phase pair processing.
    /// </summary>
    public struct WorkerPairCache
    {
        struct UntypedList
        {
            public RawBuffer Buffer;
            /// <summary>
            /// The number of bytes in the list.
            /// </summary>
            public int Count;
            public int ByteCount;


            public UntypedList(int initialSize, BufferPool pool)
            {
                pool.Take(initialSize, out Buffer);
                Count = 0;
                ByteCount = 0;
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe int Add<T>(ref T data, int minimumCount, BufferPool pool)
            {
                if (!Buffer.Allocated)
                {
                    //This didn't exist at all before; create a new entry for this type.
                    pool.Take(minimumCount * Unsafe.SizeOf<T>(), out Buffer);
                    Debug.Assert(Buffer.Length > 0);
                }
                else
                {
                    var newSize = Count * Unsafe.SizeOf<T>() + Unsafe.SizeOf<T>();
                    if (newSize > Buffer.Length)
                    {
                        //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                        pool.Take(newSize, out var newBuffer);
                        Unsafe.CopyBlockUnaligned(newBuffer.Memory, Buffer.Memory, (uint)Buffer.Length);
                    }
                }
                var index = Count++;
                //If we store only byte count, we'd have to divide to get the element index.
                //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
                //So we just store both. It's pretty cheap and simple.
                ByteCount += Unsafe.SizeOf<T>();
                Unsafe.Add(ref Unsafe.As<byte, T>(ref *Buffer.Memory), index) = data;
                return index;
            }
        }
        //TODO: THIS TYPE IS STORED IN A BUFFER AT THE MOMENT, SO THIS REFERENCE TYPE CACHING IS INVALID!
        BufferPool pool;
        int minimumPerTypeCapacity;
        int workerIndex;
        //Note that the per-type batches are untyped.
        //The caller will have the necessary type knowledge to interpret the buffer.
        Buffer<UntypedList> constraintCaches;
        Buffer<UntypedList> collisionCaches;

        public struct PendingAdd
        {
            public CollidablePair Pair;
            public CollidablePairPointers Pointers;
        }

        /// <summary>
        /// The set of pair-pointer associations created by this worker that should be added to the pair mapping.
        /// </summary>
        public QuickList<PendingAdd, Buffer<PendingAdd>> PendingAdds;

        public WorkerPairCache(int workerIndex, BufferPool pool, ref QuickList<int, Buffer<int>> minimumSizesPerConstraintType, ref QuickList<int, Buffer<int>> minimumSizesPerCollisionType,
            int pendingAddCapacity, int minimumPerTypeCapacity = 128)
        {
            this.workerIndex = workerIndex;
            this.pool = pool;
            this.minimumPerTypeCapacity = minimumPerTypeCapacity;
            const float previousCountMultiplier = 1.25f;
            pool.SpecializeFor<UntypedList>().Take((int)(minimumSizesPerConstraintType.Count * previousCountMultiplier), out constraintCaches);
            pool.SpecializeFor<UntypedList>().Take((int)(minimumSizesPerCollisionType.Count * previousCountMultiplier), out collisionCaches);
            for (int i = 0; i < minimumSizesPerConstraintType.Count; ++i)
            {
                if (minimumSizesPerConstraintType[i] > 0)
                    constraintCaches[i] = new UntypedList(Math.Max(minimumPerTypeCapacity, (int)(previousCountMultiplier * minimumSizesPerConstraintType[i])), pool);
                else
                    constraintCaches[i] = new UntypedList();
            }
            for (int i = 0; i < minimumSizesPerCollisionType.Count; ++i)
            {
                if (minimumSizesPerCollisionType[i] > 0)
                    collisionCaches[i] = new UntypedList(Math.Max(minimumPerTypeCapacity, (int)(previousCountMultiplier * minimumSizesPerCollisionType[i])), pool);
                else
                    collisionCaches[i] = new UntypedList();
            }

            QuickList<PendingAdd, Buffer<PendingAdd>>.Create(pool.SpecializeFor<PendingAdd>(), pendingAddCapacity, out PendingAdds);
        }


        public void GetMaximumCacheTypeCounts(out int collision, out int constraint)
        {
            collision = 0;
            constraint = 0;
            for (int i = collisionCaches.Length - 1; i >= 0; --i)
            {
                if (collisionCaches[i].Count > 0)
                {
                    collision = i + 1;
                    break;
                }
            }
            for (int i = constraintCaches.Length - 1; i >= 0; --i)
            {
                if (constraintCaches[i].Count > 0)
                {
                    constraint = i + 1;
                    break;
                }
            }
        }

        public void AccumulateMinimumSizes(ref QuickList<int, Buffer<int>> minimumSizesPerConstraintType, ref QuickList<int, Buffer<int>> minimumSizesPerCollisionType)
        {
            for (int i = 0; i < constraintCaches.Length; ++i)
            {
                minimumSizesPerConstraintType[i] = Math.Max(minimumSizesPerConstraintType[i], constraintCaches[i].Count);
            }
            for (int i = collisionCaches.Length - 1; i >= 0; --i)
            {
                minimumSizesPerCollisionType[i] = Math.Max(minimumSizesPerCollisionType[i], collisionCaches[i].Count);
            }
        }

        //Note that we have no-collision-data overloads. The vast majority of types don't actually have any collision data cached.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void WorkerCacheAdd<TCollision, TConstraint>(ref TCollision collisionCache, ref TConstraint constraintCache, out CollidablePairPointers pointers)
            where TCollision : struct, IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = new PairCacheIndex(workerIndex, collisionCache.TypeId, collisionCaches[collisionCache.TypeId].Add(ref collisionCache, minimumPerTypeCapacity, pool)),
                ConstraintCache = new PairCacheIndex(workerIndex, constraintCache.TypeId, constraintCaches[constraintCache.TypeId].Add(ref constraintCache, minimumPerTypeCapacity, pool))
            };
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void WorkerCacheAdd<TConstraint>(ref TConstraint constraintCache, out CollidablePairPointers pointers)
            where TConstraint : IPairCacheEntry
        {
            pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = new PairCacheIndex(),
                ConstraintCache = new PairCacheIndex(workerIndex, constraintCache.TypeId, constraintCaches[constraintCache.TypeId].Add(ref constraintCache, minimumPerTypeCapacity, pool))
            };
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add<TCollision, TConstraint>(ref CollidablePair pair, ref TCollision collisionCache, ref TConstraint constraintCache)
            where TCollision : struct, IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            PendingAdd pendingAdd;
            WorkerCacheAdd(ref collisionCache, ref constraintCache, out pendingAdd.Pointers);
            pendingAdd.Pair = pair;
            PendingAdds.Add(ref pendingAdd, pool.SpecializeFor<PendingAdd>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add<TConstraint>(ref CollidablePair pair, ref TConstraint constraintCache)
            where TConstraint : IPairCacheEntry
        {
            PendingAdd pendingAdd;
            WorkerCacheAdd(ref constraintCache, out pendingAdd.Pointers);
            pendingAdd.Pair = pair;
            PendingAdds.Add(ref pendingAdd, pool.SpecializeFor<PendingAdd>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Update<TCollision, TConstraint>(ref CollidablePairPointers pointers, ref TCollision collisionCache, ref TConstraint constraintCache)
            where TCollision : struct, IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            WorkerCacheAdd(ref collisionCache, ref constraintCache, out pointers);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Update<TConstraint>(ref CollidablePairPointers pointers, ref TConstraint constraintCache)
            where TConstraint : IPairCacheEntry
        {
            WorkerCacheAdd(ref constraintCache, out pointers);
        }


        public void Dispose()
        {
            for (int i = 0; i < constraintCaches.Length; ++i)
            {
                if (constraintCaches[i].Buffer.Allocated)
                    pool.Return(ref collisionCaches[i].Buffer);
            }
            for (int i = 0; i < collisionCaches.Length; ++i)
            {
                if (collisionCaches[i].Buffer.Allocated)
                    pool.Return(ref collisionCaches[i].Buffer);
            }
            //note that the pending adds collection is not disposed here; it is disposed upon flushing immediately after the narrow phase completes.
        }
    }

    public class PairCache
    {
        OverlapMapping mapping;
        BufferPool pool;
        int minimumPendingSize;
        int minimumPerTypeCapacity;
        int previousPendingSize;

        //While the current worker caches are read from, the next caches are written to.
        //TODO: The worker pair caches contain a reference to a buffer pool, which is a reference type. The use of buffers here is invalid.
        //You need to either work around that reference caching or use an array span here. Using an array span would have very little downside; worker counts are low and mostly fixed.
        QuickList<WorkerPairCache, Buffer<WorkerPairCache>> workerCaches;
        QuickList<WorkerPairCache, Buffer<WorkerPairCache>> nextWorkerCaches;


        public PairCache(BufferPool pool, int minimumMappingSize = 2048, int minimumPendingSize = 128, int minimumPerTypeCapacity = 128)
        {
            this.minimumPendingSize = minimumPendingSize;
            this.minimumPerTypeCapacity = minimumPerTypeCapacity;
            this.pool = pool;
            OverlapMapping.Create(
                pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>(),
                SpanHelper.GetContainingPowerOf2(minimumMappingSize), 3, out mapping);


        }

        public void Prepare(IThreadDispatcher threadDispatcher = null)
        {
            int maximumConstraintTypeCount = 0, maximumCollisionTypeCount = 0;
            for (int i = 0; i < workerCaches.Count; ++i)
            {
                workerCaches[i].GetMaximumCacheTypeCounts(out var collision, out var constraint);
                if (collision > maximumCollisionTypeCount)
                    maximumCollisionTypeCount = collision;
                if (constraint > maximumConstraintTypeCount)
                    maximumConstraintTypeCount = constraint;
            }
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), maximumConstraintTypeCount, out var minimumSizesPerConstraintType);
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), maximumCollisionTypeCount, out var minimumSizesPerCollisionType);
            for (int i = 0; i < workerCaches.Count; ++i)
            {
                workerCaches[i].AccumulateMinimumSizes(ref minimumSizesPerConstraintType, ref minimumSizesPerCollisionType);
            }
            QuickList<WorkerPairCache, Buffer<WorkerPairCache>>.Create(pool.SpecializeFor<WorkerPairCache>(), threadDispatcher.ThreadCount, out nextWorkerCaches);

            //TODO: Single threaded.
            var pendingSize = Math.Max(minimumPendingSize, previousPendingSize);
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                nextWorkerCaches[i] = new WorkerPairCache(i, threadDispatcher.GetThreadMemoryPool(i), ref minimumSizesPerConstraintType, ref minimumSizesPerCollisionType,
                    pendingSize, minimumPerTypeCapacity);
            }

            minimumSizesPerConstraintType.Dispose(pool.SpecializeFor<int>());
            minimumSizesPerCollisionType.Dispose(pool.SpecializeFor<int>());
        }

        /// <summary>
        /// Flush all deferred changes from the last narrow phase execution.
        /// </summary>
        public void Flush()
        {
            //Get rid of the now-unused worker caches.
            for (int i = 0; i < workerCaches.Count; ++i)
            {
                workerCaches[i].Dispose();
            }

            //Flush all pending adds from the new set.
            var pairPool = pool.SpecializeFor<CollidablePair>();
            var pendingPool = pool.SpecializeFor<WorkerPairCache.PendingAdd>();
            var pointerPool = pool.SpecializeFor<CollidablePairPointers>();
            var intPool = pool.SpecializeFor<int>();
            int largestPendingSize = 0;
            for (int i = 0; i < nextWorkerCaches.Count; ++i)
            {
                ref var cache = ref nextWorkerCaches[i];
                if (cache.PendingAdds.Count > largestPendingSize)
                {
                    largestPendingSize = cache.PendingAdds.Count;
                }
                for (int j = 0; j < cache.PendingAdds.Count; ++j)
                {
                    ref var pending = ref cache.PendingAdds[j];
                    mapping.Add(ref pending.Pair, ref pending.Pointers, pairPool, pointerPool, intPool);
                }
                cache.PendingAdds.Dispose(pendingPool);
            }
            //Swap pointers.
            var temp = nextWorkerCaches;
            workerCaches = nextWorkerCaches;
            nextWorkerCaches = temp;
        }


        public void Dispose()
        {
            for (int i = 0; i < workerCaches.Count; ++i)
            {
                workerCaches[i].Dispose();
            }
            mapping.Dispose(pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref CollidablePair pair)
        {
            return mapping.IndexOf(ref pair);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollidablePairPointers GetPointers(int index)
        {
            return ref mapping.Values[index];
        }
        /// <summary>
        /// Marks a pair in the narrow phase as fresh. It is not a candidate for removal from the narrow phase during this frame.
        /// </summary>
        /// <param name="index">Index to mark.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void MarkFresh(int index)
        {
            //TODO: Should have a pair-parallel array of bytes. Every updated pair will have its byte set to a value that avoids the removal postprocess.
            //Note that this is not stored in the pointers array- the removal postprocess doesn't consider any information in its analysis pass except for the freshness bytes,
            //so to keep memory bandwidth at a minimum, 
            //(You could use a single bit, but then you have to manage the atomicity of changes manually.)
        }

        public void Update()

        public void Add(int workerIndex, ref CollidablePair pair, )


        internal void ScatterNewImpulses(int constraintType, ref ConstraintReference constraintReference, ref ContactImpulses contactImpulses)
        {
            //Constraints cover 16 possible cases:
            //1-4 contacts: 0x3
            //convex vs nonconvex: 0x4
            //1 body versus 2 body: 0x8
            //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.

            //TODO: Note that we do not modify the friction accumulated impulses. This is just for simplicity- the impact of accumulated impulses on friction *should* be relatively
            //hard to notice compared to penetration impulses. We should, however, test this assumption.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var inner);
            switch (constraintType)
            {
                //1 body
                //Convex
                case 0:
                    {
                        //1 contact
                    }
                    break;
                case 1:
                    {
                        //2 contacts
                    }
                    break;
                case 2:
                    {
                        //3 contacts
                    }
                    break;
                case 3:
                    {
                        //4 contacts
                    }
                    break;
                //Nonconvex
                case 4 + 0:
                    {
                        //1 contact
                    }
                    break;
                case 4 + 1:
                    {
                        //2 contacts
                    }
                    break;
                case 4 + 2:
                    {
                        //3 contacts
                    }
                    break;
                case 4 + 3:
                    {
                        //4 contacts
                    }
                    break;
                //2 body
                //Convex
                case 8 + 0:
                    {
                        //1 contact
                    }
                    break;
                case 8 + 1:
                    {
                        //2 contacts
                    }
                    break;
                case 8 + 2:
                    {
                        //3 contacts
                    }
                    break;
                case 8 + 3:
                    {
                        //4 contacts
                        var batch = Unsafe.As<TypeBatch, ContactManifold4TypeBatch>(ref constraintReference.TypeBatch);
                        ref var bundle = ref batch.AccumulatedImpulses[bundleIndex];
                        GatherScatter.SetLane(ref bundle.Penetration0, inner, ref contactImpulses.Impulse0, 4);
                    }
                    break;
                //Nonconvex
                case 8 + 4 + 0:
                    {
                        //1 contact
                    }
                    break;
                case 8 + 4 + 1:
                    {
                        //2 contacts
                    }
                    break;
                case 8 + 4 + 2:
                    {
                        //3 contacts
                    }
                    break;
                case 8 + 4 + 3:
                    {
                        //4 contacts
                    }
                    break;
            }

        }


        /// <summary>
        /// Completes the addition of a constraint by filling in the narrowphase's pointer to the constraint and by distributing accumulated impulses.
        /// </summary>
        /// <param name="solver">Solver containing the constraint to set the impulses of.</param>
        /// <param name="impulses"></param>
        /// <param name="constraintCacheIndex"></param>
        /// <param name="constraintHandle"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CompleteConstraintAdd(Solver solver, ref ContactImpulses impulses, TypedIndex constraintCacheIndex, int constraintHandle)
        {
            var constraintContactCount = constraintCacheIndex.Type & 3;
            var constraintIndex = constraintCacheIndex.Index;
            switch (constraintContactCount)
            {
                case 0:
                    Debug.Assert(constraintCache1.Span.Allocated && constraintContactCount < constraintCache1.Count);
                    constraintCache1[constraintIndex].ConstraintHandle = constraintHandle;
                    return;
                case 1:
                    Debug.Assert(constraintCache2.Span.Allocated && constraintContactCount < constraintCache2.Count);
                    constraintCache2[constraintIndex].ConstraintHandle = constraintHandle;
                    return;
                case 2:
                    Debug.Assert(constraintCache3.Span.Allocated && constraintContactCount < constraintCache3.Count);
                    constraintCache3[constraintIndex].ConstraintHandle = constraintHandle;
                    return;
                case 3:
                    Debug.Assert(constraintCache4.Span.Allocated && constraintContactCount < constraintCache4.Count);
                    constraintCache4[constraintIndex].ConstraintHandle = constraintHandle;
                    return;
            }

            solver.GetConstraintReference(constraintHandle, out var reference);
            ScatterNewImpulses(constraintCacheIndex.Type, ref reference, ref impulses);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TConstraintCache GetConstraintCache<TConstraintCache>(int constraintCacheIndex)
        {
            //Note that the casts are nops. They're just there so that the compiler is okay with it.
            //Also, given that the constraint cache types are all structs, this will type specialize.
            if (typeof(TConstraintCache) == typeof(ConstraintCache1))
            {
                Debug.Assert(constraintCache1.Span.Allocated && constraintCacheIndex < constraintCache1.Count);
                return ref Unsafe.As<ConstraintCache1, TConstraintCache>(ref constraintCache1[constraintCacheIndex]);
            }
            if (typeof(TConstraintCache) == typeof(ConstraintCache2))
            {
                Debug.Assert(constraintCache2.Span.Allocated && constraintCacheIndex < constraintCache2.Count);
                return ref Unsafe.As<ConstraintCache2, TConstraintCache>(ref constraintCache2[constraintCacheIndex]);
            }
            if (typeof(TConstraintCache) == typeof(ConstraintCache3))
            {
                Debug.Assert(constraintCache3.Span.Allocated && constraintCacheIndex < constraintCache3.Count);
                return ref Unsafe.As<ConstraintCache3, TConstraintCache>(ref constraintCache3[constraintCacheIndex]);
            }
            if (typeof(TConstraintCache) == typeof(ConstraintCache4))
            {
                Debug.Assert(constraintCache4.Span.Allocated && constraintCacheIndex < constraintCache4.Count);
                return ref Unsafe.As<ConstraintCache4, TConstraintCache>(ref constraintCache4[constraintCacheIndex]);
            }
            Debug.Fail("The type of the constraint cache must actually be one of the hard coded constraint caches.");
            return ref Unsafe.As<ConstraintCache1, TConstraintCache>(ref constraintCache1[0]);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TCollisionData GetCollisionData<TCollisionData>(int index) where TCollisionData : struct, IPairCacheEntry
        {
            return ref Unsafe.As<byte, TCollisionData>(ref *collisionDataCache[default(TCollisionData).TypeId].Buffer.Memory);
        }


    }
}
