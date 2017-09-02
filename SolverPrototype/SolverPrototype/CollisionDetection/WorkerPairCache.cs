﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
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
                var newSize = ByteCount + Unsafe.SizeOf<T>();
                if (!Buffer.Allocated)
                {
                    //This didn't exist at all before; create a new entry for this type.
                    pool.Take(Math.Max(newSize, minimumCount * Unsafe.SizeOf<T>()), out Buffer);
                    Debug.Assert(Buffer.Length > 0);
                }
                else
                {
                    if (newSize > Buffer.Length)
                    {
                        //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                        pool.Take(newSize, out var newBuffer);
                        Unsafe.CopyBlockUnaligned(newBuffer.Memory, Buffer.Memory, (uint)Buffer.Length);
                    }
                }
                //If we store only byte count, we'd have to divide to get the element index.
                //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
                //So we just store both. It's pretty cheap and simple.
                Count++;
                var byteIndex = ByteCount;
                ByteCount = newSize;
                Unsafe.As<byte, T>(ref Buffer.Memory[byteIndex]) = data;
                return byteIndex;
            }
        }
        BufferPool pool; //note that this reference makes the entire worker pair cache nonblittable. That's why the pair cache uses managed arrays to store the worker caches.
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
            where TCollision : IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            pointers.ConstraintCache = new PairCacheIndex(workerIndex, constraintCache.TypeId, constraintCaches[constraintCache.TypeId].Add(ref constraintCache, minimumPerTypeCapacity, pool));

            if (typeof(TCollision) == typeof(EmptyCollisionCache))
                pointers.CollisionDetectionCache = new PairCacheIndex();
            else
                pointers.CollisionDetectionCache = new PairCacheIndex(workerIndex, collisionCache.TypeId, collisionCaches[collisionCache.TypeId].Add(ref collisionCache, minimumPerTypeCapacity, pool));

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairCacheIndex Add<TCollision, TConstraint>(ref CollidablePair pair, ref TCollision collisionCache, ref TConstraint constraintCache)
            where TCollision : IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            PendingAdd pendingAdd;
            WorkerCacheAdd(ref collisionCache, ref constraintCache, out pendingAdd.Pointers);
            pendingAdd.Pair = pair;
            PendingAdds.Add(ref pendingAdd, pool.SpecializeFor<PendingAdd>());
            return pendingAdd.Pointers.ConstraintCache;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Update<TCollision, TConstraint>(ref CollidablePairPointers pointers, ref TCollision collisionCache, ref TConstraint constraintCache)
            where TCollision : IPairCacheEntry where TConstraint : IPairCacheEntry
        {
            WorkerCacheAdd(ref collisionCache, ref constraintCache, out pointers);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void* GetConstraintCachePointer(PairCacheIndex constraintCacheIndex)
        {
            //Note that only the count is used to index into the constraint caches.
            //TODO: If you expand the number of contacts that can exist in a single entry, this will have to be updated.
            return constraintCaches[constraintCacheIndex.Type & 3].Buffer.Memory + constraintCacheIndex.Index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void* GetCollisionCachePointer(PairCacheIndex collisionCacheIndex)
        {
            return constraintCaches[collisionCacheIndex.Type].Buffer.Memory + collisionCacheIndex.Index;
        }

        public void Dispose()
        {
            for (int i = 0; i < constraintCaches.Length; ++i)
            {
                if (constraintCaches[i].Buffer.Allocated)
                    pool.Return(ref collisionCaches[i].Buffer);
            }
            pool.SpecializeFor<UntypedList>().Return(ref constraintCaches);
            for (int i = 0; i < collisionCaches.Length; ++i)
            {
                if (collisionCaches[i].Buffer.Allocated)
                    pool.Return(ref collisionCaches[i].Buffer);
            }
            pool.SpecializeFor<UntypedList>().Return(ref collisionCaches);
            this = new WorkerPairCache();
            //note that the pending adds collection is not disposed here; it is disposed upon flushing immediately after the narrow phase completes.
        }
    }
}
