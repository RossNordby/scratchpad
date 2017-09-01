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
        public TypedIndex ConstraintCache;
        /// <summary>
        /// A narrowphase-specific type and index into a batch of custom data for the pair. Many types do not use any supplementary data, but some make use of temporal coherence
        /// to accelerate contact generation.
        /// </summary>
        public TypedIndex CollisionDetectionCache;
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
        BufferPool pool;
        int minimumTypeCapacity;
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

        public WorkerPairCache(BufferPool pool, ref QuickList<int, Buffer<int>> minimumSizesPerConstraintType, ref QuickList<int, Buffer<int>> minimumSizesPerCollisionType,
            int pendingAddCapacity, int minimumTypeCapacity = 128)
        {
            this.pool = pool;
            this.minimumTypeCapacity = minimumTypeCapacity;
            const float previousTypeCountMultiplier = 1.25f;
            pool.SpecializeFor<UntypedList>().Take((int)(minimumSizesPerConstraintType.Count * previousTypeCountMultiplier), out constraintCaches);
            pool.SpecializeFor<UntypedList>().Take((int)(minimumSizesPerCollisionType.Count * previousTypeCountMultiplier), out collisionCaches);
            for (int i = 0; i < minimumSizesPerConstraintType.Count; ++i)
            {
                if (minimumSizesPerConstraintType[i] > 0)
                    constraintCaches[i] = new UntypedList(Math.Max(minimumTypeCapacity, minimumSizesPerConstraintType[i]), pool);
                else
                    constraintCaches[i] = new UntypedList();
            }
            for (int i = 0; i < minimumSizesPerCollisionType.Count; ++i)
            {
                if (minimumSizesPerCollisionType[i] > 0)
                    collisionCaches[i] = new UntypedList(Math.Max(minimumTypeCapacity, minimumSizesPerCollisionType[i]), pool);
                else
                    collisionCaches[i] = new UntypedList();
            }

            QuickList<PendingAdd, Buffer<PendingAdd>>.Create(pool.SpecializeFor<PendingAdd>(), pendingAddCapacity, out PendingAdds);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void WorkerCacheAdd<TCollision, TConstraint>(ref TCollision collisionCache, ref TConstraint constraintCache, out CollidablePairPointers pointers)
            where TCollision : struct, IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = new TypedIndex(collisionCache.TypeId, collisionCaches[collisionCache.TypeId].Add(ref collisionCache, minimumTypeCapacity, pool)),
                ConstraintCache = new TypedIndex(constraintCache.TypeId, constraintCaches[constraintCache.TypeId].Add(ref constraintCache, minimumTypeCapacity, pool))
            };
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void WorkerCacheAdd<TConstraint>(ref TConstraint constraintCache, out CollidablePairPointers pointers)
            where TConstraint : IPairCacheEntry
        {
            pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = new TypedIndex(),
                ConstraintCache = new TypedIndex(constraintCache.TypeId, constraintCaches[constraintCache.TypeId].Add(ref constraintCache, minimumTypeCapacity, pool))
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
        }
    }

    public struct PairCache
    {
        OverlapMapping cache;
        BufferPool pool;



        int minimumCollisionDataBatchSize, minimumConstraintBatchSize;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InitializeConstraintCache<T>(ref QuickList<T, Buffer<T>> previous, BufferPool pool, float multiplier, out QuickList<T, Buffer<T>> current)
        {
            if (previous.Count > 0)
                QuickList<T, Buffer<T>>.Create(pool.SpecializeFor<T>(), (int)(previous.Count * multiplier), out current);
            else
                current = new QuickList<T, Buffer<T>>();
        }

        //The previous cache always persists while building a new one. It has to be available for searching old cached data.
        public PairCache(BufferPool pool, ref PairCache previousPairCache, int minimumMappingSize = 2048,
            int minimumConstraintBatchSize = 128, int minimumCollisionDataBatchSize = 128, float previousBatchMultiplier = 1.25f)
        {
            this.pool = pool;
            OverlapMapping.Create(
                pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>(),
                SpanHelper.GetContainingPowerOf2((int)(previousPairCache.cache.Count * previousBatchMultiplier)), 3, out cache);

            const int customDataTypeCount = 4;
            pool.SpecializeFor<CollisionDataList>().Take(customDataTypeCount, out collisionDataCache);
            //Preallocate any caches that were used in the previous frame. No reason to go through all the resize effort again.
            for (int i = 0; i < previousPairCache.collisionDataCache.Length; ++i)
            {
                var previousSize = previousPairCache.collisionDataCache[i].ByteCount;
                ref var typeCache = ref collisionDataCache[i];
                if (previousSize > 0)
                {
                    pool.Take((int)(previousPairCache.collisionDataCache[i].ByteCount * previousBatchMultiplier), out typeCache.Buffer);
                    typeCache.ByteCount = 0;
                    typeCache.Count = 0;
                }
                else
                {
                    //We check the allocated state to know if a given cache exists, so we have to clear. Old data might otherwise corrupt.
                    collisionDataCache[i] = new CollisionDataList();
                }
            }
            InitializeConstraintCache(ref previousPairCache.constraintCache1, pool, previousBatchMultiplier, out constraintCache1);
            InitializeConstraintCache(ref previousPairCache.constraintCache2, pool, previousBatchMultiplier, out constraintCache2);
            InitializeConstraintCache(ref previousPairCache.constraintCache3, pool, previousBatchMultiplier, out constraintCache3);
            InitializeConstraintCache(ref previousPairCache.constraintCache4, pool, previousBatchMultiplier, out constraintCache4);


            this.minimumCollisionDataBatchSize = minimumCollisionDataBatchSize;
            this.minimumConstraintBatchSize = minimumConstraintBatchSize;
        }


        public void Dispose()
        {
            cache.Dispose(pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref CollidablePair pair)
        {
            return cache.IndexOf(ref pair);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollidablePairPointers GetPointers(int index)
        {
            return ref cache.Values[index];
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
