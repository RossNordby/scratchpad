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



    public class PairCache
    {
        OverlapMapping mapping;
        /// <summary>
        /// Per-pair 'freshness' flags set when a pair is added or updated by the narrow phase execution.
        /// </summary>
        RawBuffer pairFreshness;
        BufferPool pool;
        int minimumPendingSize;
        int minimumPerTypeCapacity;
        int previousPendingSize;

        //While the current worker caches are read from, the next caches are written to.
        //The worker pair caches contain a reference to a buffer pool, which is a reference type. That makes WorkerPairCache non-blittable, so in the interest of not being
        //super duper gross, we don't use the untyped buffer pools to store it. 
        //Given that the size of the arrays here will be small and almost never change, this isn't a significant issue.
        QuickList<WorkerPairCache, Array<WorkerPairCache>> workerCaches;
        QuickList<WorkerPairCache, Array<WorkerPairCache>> nextWorkerCaches;


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

            var threadCount = threadDispatcher != null ? threadDispatcher.ThreadCount : 1;
            //Ensure that the new worker pair caches can hold all workers.
            if (nextWorkerCaches.Span.Length < threadCount)
            {
                //The next worker caches should never need to be disposed here. The flush should have taken care of it.
#if DEBUG
                for (int i = 0; i < nextWorkerCaches.Count; ++i)
                    Debug.Assert(nextWorkerCaches[i].Equals(default(WorkerPairCache)));
#endif
                QuickList<WorkerPairCache, Array<WorkerPairCache>>.Create(new PassthroughArrayPool<WorkerPairCache>(), threadCount, out nextWorkerCaches);
            }
            //Note that we have not initialized the workerCaches from the previous frame. In the event that this is the first frame and there are no previous worker caches,
            //there will be no pointers into the caches, and removal analysis loops over the count which defaults to zero- so it's safe.
            nextWorkerCaches.Count = threadCount;

            var pendingSize = Math.Max(minimumPendingSize, previousPendingSize);
            if (threadDispatcher != null)
            {
                for (int i = 0; i < threadCount; ++i)
                {
                    nextWorkerCaches[i] = new WorkerPairCache(i, threadDispatcher.GetThreadMemoryPool(i), ref minimumSizesPerConstraintType, ref minimumSizesPerCollisionType,
                        pendingSize, minimumPerTypeCapacity);
                }
            }
            else
            {
                nextWorkerCaches[0] = new WorkerPairCache(0, pool, ref minimumSizesPerConstraintType, ref minimumSizesPerCollisionType, pendingSize, minimumPerTypeCapacity);
            }
            pool.Take(mapping.Count, out pairFreshness);
            //This clears 1 byte per pair. 32768 pairs with 10GBps assumed single core bandwidth means about 3 microseconds.
            //There is a small chance that multithreading this would be useful in larger simulations- but it would be very, very close.
            pairFreshness.Clear(0, mapping.Count);

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
            previousPendingSize = largestPendingSize;
            //Swap references.
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
            //Note that we do not need to dispose the worker cache arrays themselves- they were just arrays pulled out of a passthrough pool.
#if DEBUG
            for (int i =0; i < nextWorkerCaches.Count; ++i)
            {
                Debug.Assert(nextWorkerCaches[i].Equals(default(WorkerPairCache)), "Outside of the execution of the narrow phase, the 'next' caches should not be allocated.");
            }
#endif
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void BuildNewConstraintCache<TConstraintCache>(int* featureIds, out TConstraintCache cache)
        {
            cache = default(TConstraintCache);
            //1 contact constraint caches do not store a feature id; it's pointless.
            if (typeof(TConstraintCache) == typeof(ConstraintCache2))
            {
                ref var typedCache = ref Unsafe.As<TConstraintCache, ConstraintCache2>(ref cache);
                typedCache.FeatureId0 = featureIds[0];
                typedCache.FeatureId1 = featureIds[1];
            }
            else if (typeof(TConstraintCache) == typeof(ConstraintCache3))
            {
                ref var typedCache = ref Unsafe.As<TConstraintCache, ConstraintCache3>(ref cache);
                typedCache.FeatureId0 = featureIds[0];
                typedCache.FeatureId1 = featureIds[1];
                typedCache.FeatureId2 = featureIds[2];
            }
            else if (typeof(TConstraintCache) == typeof(ConstraintCache4))
            {
                ref var typedCache = ref Unsafe.As<TConstraintCache, ConstraintCache4>(ref cache);
                typedCache.FeatureId0 = featureIds[0];
                typedCache.FeatureId1 = featureIds[1];
                typedCache.FeatureId2 = featureIds[2];
                typedCache.FeatureId3 = featureIds[3];
            }
            //TODO: In the event that higher contact count manifolds exist for the purposes of nonconvexes, this will need to be expanded.
        }

        internal unsafe PairCacheIndex Add<TConstraintCache, TCollisionCache>(int workerIndex, ref CollidablePair pair, ref TCollisionCache collisionCache, int* featureIds)
            where TConstraintCache : IPairCacheEntry
            where TCollisionCache : IPairCacheEntry
        {
            BuildNewConstraintCache(featureIds, out TConstraintCache constraintCache);
            return nextWorkerCaches[workerIndex].Add(ref pair, ref collisionCache, ref constraintCache);
        }

        internal unsafe void Update<TConstraintCache, TCollisionCache>(int workerIndex, ref CollidablePairPointers pointers, ref TCollisionCache collisionCache, int* featureIds)
            where TConstraintCache : IPairCacheEntry
            where TCollisionCache : IPairCacheEntry
        {
            BuildNewConstraintCache(featureIds, out TConstraintCache constraintCache);
            nextWorkerCaches[workerIndex].Update(ref pointers, ref collisionCache, ref constraintCache);
        }


        internal unsafe void GatherOldImpulses(int constraintType, ref ConstraintReference constraintReference, float* impulses)
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
                        GatherScatter.GetLane(ref bundle.Penetration0, inner, ref *impulses, 4);
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
        public unsafe void CompleteConstraintAdd(Solver solver, ref ContactImpulses impulses, PairCacheIndex constraintCacheIndex, int constraintHandle)
        {
            //Note that the update is being directed to the *next* worker caches. We have not yet performed the flush that swaps references.
            //Note that this assumes that the constraint handle is stored in the first 4 bytes of the constraint cache.
            *(int*)nextWorkerCaches[constraintCacheIndex.Worker].GetConstraintCachePointer(constraintCacheIndex) = constraintHandle;
            solver.GetConstraintReference(constraintHandle, out var reference);
            ScatterNewImpulses(constraintCacheIndex.Type, ref reference, ref impulses);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TConstraintCache GetConstraintCache<TConstraintCache>(PairCacheIndex constraintCacheIndex)
        {
            //Note that these refer to the previous workerCaches, not the nextWorkerCaches. We read from these caches during the narrowphase to redistribute impulses.
            return ref Unsafe.AsRef<TConstraintCache>(workerCaches[constraintCacheIndex.Worker].GetConstraintCachePointer(constraintCacheIndex));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TCollisionData GetCollisionData<TCollisionData>(PairCacheIndex index) where TCollisionData : struct, IPairCacheEntry
        {
            return ref Unsafe.AsRef<TCollisionData>(workerCaches[index.Worker].GetCollisionCachePointer(index));
        }

    }
}
