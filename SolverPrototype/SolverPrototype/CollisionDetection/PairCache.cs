using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
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

    public struct PairCache
    {
        OverlapMapping cache;
        struct CollisionDataList
        {
            public RawBuffer Buffer;
            /// <summary>
            /// The number of bytes in the list.
            /// </summary>
            public int Count;
            public int ByteCount;

        }

        //Since there is a low and fixed number of required constraint data caches, we just list them inline. Avoids a single (cheap) indirection.
        QuickList<ConstraintCache1, Buffer<ConstraintCache1>> constraintCache1;
        QuickList<ConstraintCache2, Buffer<ConstraintCache2>> constraintCache2;
        QuickList<ConstraintCache3, Buffer<ConstraintCache3>> constraintCache3;
        QuickList<ConstraintCache4, Buffer<ConstraintCache4>> constraintCache4;
        //Note that the per-type batches are untyped.
        //The caller will have the necessary type knowledge to interpret the buffer.
        Buffer<CollisionDataList> collisionDataCache;

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
            if (constraintCache1.Span.Allocated)
                constraintCache1.Dispose(pool.SpecializeFor<ConstraintCache1>());
            if (constraintCache2.Span.Allocated)
                constraintCache2.Dispose(pool.SpecializeFor<ConstraintCache2>());
            if (constraintCache3.Span.Allocated)
                constraintCache3.Dispose(pool.SpecializeFor<ConstraintCache3>());
            if (constraintCache4.Span.Allocated)
                constraintCache4.Dispose(pool.SpecializeFor<ConstraintCache4>());
            for (int i = 0; i < collisionDataCache.Length; ++i)
            {
                if (collisionDataCache[i].Buffer.Allocated)
                    pool.Return(ref collisionDataCache[i].Buffer);
            }
            cache.Dispose(pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetPointers(ref CollidablePair pair, out CollidablePairPointers pointers)
        {
            return cache.TryGetValue(ref pair, out pointers);
        }

        /// <summary>
        /// When the narrow phase reports a desired constraint, it hands the description and associated bodies off to the adder implementation. That adder is responsible
        /// for putting a constraint into the solver. It may do so in a deferred fashion; the narrow phase implementation is unaware.
        /// Since a constraint handle doesn't exist until it's added to the solver, the adder has to call back into the pair cache and update the constraint handle.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void FillConstraintHandle(TypedIndex constraintCacheIndex, int constraintHandle)
        {
            Debug.Assert(constraintCacheIndex.Exists && constraintCacheIndex.Type >= 0 && constraintCacheIndex.Type < 4);
            var index = constraintCacheIndex.Type;
            switch (constraintCacheIndex.Type)
            {
                case 0:
                    Debug.Assert(constraintCache1.Span.Allocated && index < constraintCache1.Count);
                    constraintCache1[index].ConstraintHandle = constraintHandle;
                    return;
                case 1:
                    Debug.Assert(constraintCache2.Span.Allocated && index < constraintCache2.Count);
                    constraintCache2[index].ConstraintHandle = constraintHandle;
                    return;
                case 2:
                    Debug.Assert(constraintCache3.Span.Allocated && index < constraintCache3.Count);
                    constraintCache3[index].ConstraintHandle = constraintHandle;
                    return;
                case 3:
                    Debug.Assert(constraintCache4.Span.Allocated && index < constraintCache4.Count);
                    constraintCache4[index].ConstraintHandle = constraintHandle;
                    return;
            }

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
        private unsafe int Add<TCollisionData>(ref TCollisionData data) where TCollisionData : struct, IPairCacheEntry
        {
            ref var cache = ref collisionDataCache[data.TypeId];
            if (!cache.Buffer.Allocated)
            {
                //This didn't exist at all before; create a new entry for this type.
                pool.Take(minimumCollisionDataBatchSize * Unsafe.SizeOf<TCollisionData>(), out cache.Buffer);
                Debug.Assert(cache.Buffer.Length > 0);
            }
            else
            {
                var newSize = cache.Count * Unsafe.SizeOf<TCollisionData>() + Unsafe.SizeOf<TCollisionData>();
                if (newSize > cache.Buffer.Length)
                {
                    //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                    pool.Take(newSize, out var newBuffer);
                    Unsafe.CopyBlockUnaligned(newBuffer.Memory, cache.Buffer.Memory, (uint)cache.Buffer.Length);
                }
            }
            var index = cache.Count++;
            //If we store only byte count, we'd have to divide to get the element index.
            //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
            //So we just store both. It's pretty cheap and simple.
            cache.ByteCount += Unsafe.SizeOf<TCollisionData>();
            Unsafe.Add(ref Unsafe.As<byte, TCollisionData>(ref *cache.Buffer.Memory), index) = data;
            return index;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void Add(ref CollidablePair pair, TypedIndex collisionDataIndex, int featureId0, int featureId1, int featureId2, int featureId3)
        {
            ConstraintCache4 constraintCache;
            constraintCache.ConstraintHandle = 0; //This is filled in later.
            constraintCache.FeatureId0 = featureId0;
            constraintCache.FeatureId1 = featureId1;
            constraintCache.FeatureId2 = featureId2;
            constraintCache.FeatureId3 = featureId3;
            var constraintCacheIndex = constraintCache4.Count;
            constraintCache4.Add(ref constraintCache, pool.SpecializeFor<ConstraintCache4>());
            var pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = collisionDataIndex,
                ConstraintCache = new TypedIndex(3, constraintCacheIndex)
            };
            cache.Add(ref pair, ref pointers, pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void Add(ref CollidablePair pair, TypedIndex collisionDataIndex, int featureId0, int featureId1, int featureId2)
        {
            ConstraintCache3 constraintCache;
            constraintCache.ConstraintHandle = 0; //This is filled in later.
            constraintCache.FeatureId0 = featureId0;
            constraintCache.FeatureId1 = featureId1;
            constraintCache.FeatureId2 = featureId2;
            var constraintCacheIndex = constraintCache3.Count;
            constraintCache3.Add(ref constraintCache, pool.SpecializeFor<ConstraintCache3>());
            var pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = collisionDataIndex,
                ConstraintCache = new TypedIndex(2, constraintCacheIndex)
            };
            cache.Add(ref pair, ref pointers, pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void Add(ref CollidablePair pair, TypedIndex collisionDataIndex, int featureId0, int featureId1)
        {
            ConstraintCache2 constraintCache;
            constraintCache.ConstraintHandle = 0; //This is filled in later.
            constraintCache.FeatureId0 = featureId0;
            constraintCache.FeatureId1 = featureId1;
            var constraintCacheIndex = constraintCache2.Count;
            constraintCache2.Add(ref constraintCache, pool.SpecializeFor<ConstraintCache2>());
            var pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = collisionDataIndex,
                ConstraintCache = new TypedIndex(1, constraintCacheIndex)
            };
            cache.Add(ref pair, ref pointers, pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void Add(ref CollidablePair pair, TypedIndex collisionDataIndex)
        {
            ConstraintCache1 constraintCache;
            constraintCache.ConstraintHandle = 0; //This is filled in later.
            var constraintCacheIndex = constraintCache2.Count;
            constraintCache1.Add(ref constraintCache, pool.SpecializeFor<ConstraintCache1>());
            var pointers = new CollidablePairPointers
            {
                CollisionDetectionCache = collisionDataIndex,
                ConstraintCache = new TypedIndex(0, constraintCacheIndex)
            };
            cache.Add(ref pair, ref pointers, pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<CollidablePairPointers>(), pool.SpecializeFor<int>());
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, ref TCollisionData collisionData, int featureId0, int featureId1, int featureId2, int featureId3)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(collisionData.TypeId, Add(ref collisionData)), featureId0, featureId1, featureId2, featureId3);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, ref TCollisionData collisionData, int featureId0, int featureId1, int featureId2)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(collisionData.TypeId, Add(ref collisionData)), featureId0, featureId1, featureId2);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, ref TCollisionData collisionData, int featureId0, int featureId1)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(collisionData.TypeId, Add(ref collisionData)), featureId0, featureId1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, ref TCollisionData collisionData)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(collisionData.TypeId, Add(ref collisionData)));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, int featureId0, int featureId1, int featureId2, int featureId3)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(), featureId0, featureId1, featureId2, featureId3);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, int featureId0, int featureId1, int featureId2)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(), featureId0, featureId1, featureId2);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair, int featureId0, int featureId1)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex(), featureId0, featureId1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TCollisionData>(ref CollidablePair pair)
            where TCollisionData : struct, IPairCacheEntry
        {
            Add(ref pair, new TypedIndex());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TCollisionData GetCollisionData<TCollisionData>(int index) where TCollisionData : struct, IPairCacheEntry
        {
            return ref Unsafe.As<byte, TCollisionData>(ref *collisionDataCache[default(TCollisionData).TypeId].Buffer.Memory);
        }


    }
}
