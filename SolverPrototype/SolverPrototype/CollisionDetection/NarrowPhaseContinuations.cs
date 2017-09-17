using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{

    public enum ConstraintGeneratorType
    {
        /// <summary>
        /// Pair which will directly produce constraints.
        /// </summary>
        Direct = 0,
        /// <summary>
        /// Pair expecting both a discrete and inner sphere manifolds.
        /// </summary>
        Linear = 1,
        /// <summary>
        /// Pair expecting multiple discrete manifolds.
        /// </summary>
        Substep = 2,
        /// <summary>
        /// Pair expecting both inner sphere manifolds and multiple discrete manifolds.
        /// </summary>
        SubstepWithLinear = 3
    }

    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        public struct ConstraintGenerators : IContinuations
        {
            int workerIndex;
            BufferPool pool;
            NarrowPhase<TCallbacks> narrowPhase;

            struct LinearManifolds
            {
                public ContactManifold A;
                public ContactManifold B;
            }

            struct SubstepManifolds
            {
                public QuickList<ContactManifold, Buffer<ContactManifold>> Manifolds;
                public SubstepManifolds(BufferPool pool, int capacity)
                {
                    QuickList<ContactManifold, Buffer<ContactManifold>>.Create(pool.SpecializeFor<ContactManifold>(), capacity, out Manifolds);
                }
            }

            //Discrete pairs are simply stored as collidable pairs; there is no additional cached data.
            struct LinearPair
            {
                public ContactManifold DiscreteManifold;
                public LinearManifolds LinearManifolds;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(CollidablePair pair)
                {
                    Pair = pair;
                    ManifoldsReported = 0;
                }
            }

            struct SubstepPair
            {
                public SubstepManifolds Manifolds;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, CollidablePair pair)
                {
                    Manifolds = new SubstepManifolds(pool, substepCapacity);
                    Pair = pair;
                    ManifoldsReported = 0;
                }
            }

            struct SubstepWithLinearPair
            {
                public SubstepManifolds SubstepManifolds;
                public LinearManifolds LinearManifolds;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, CollidablePair pair)
                {
                    SubstepManifolds = new SubstepManifolds(pool, substepCapacity);
                    Pair = pair;
                    ManifoldsReported = 0;
                }
            }

            struct ContinuationCache<T>
            {
                public IdPool<Buffer<int>> Ids;
                public Buffer<T> Caches;

                public ContinuationCache(BufferPool pool)
                {
                    IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 32, out Ids);
                    pool.SpecializeFor<T>().Take(128, out Caches);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public ref T Allocate(BufferPool pool, out int index)
                {
                    index = Ids.Take();
                    if (Caches.Length < index)
                    {
                        pool.SpecializeFor<T>().Resize(ref Caches, index, Caches.Length);
                    }
                    return ref Caches[index];
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Dispose(BufferPool pool)
                {
                    Ids.Dispose(pool.SpecializeFor<int>());
                    pool.SpecializeFor<T>().Return(ref Caches);
                }
            }

            ContinuationCache<CollidablePair> discrete;
            ContinuationCache<LinearPair> linear;
            ContinuationCache<SubstepPair> substep;
            ContinuationCache<SubstepWithLinearPair> substepWithLinear;

            public ConstraintGenerators(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.pool = pool;
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
                discrete = new ContinuationCache<CollidablePair>(pool);
                linear = new ContinuationCache<LinearPair>(pool);
                substep = new ContinuationCache<SubstepPair>(pool);
                substepWithLinear = new ContinuationCache<SubstepWithLinearPair>(pool);
            }

            public unsafe void Notify(TypedIndex continuationId, ContactManifold* manifold)
            {
                var todoTestCollisionCache = default(EmptyCollisionCache);
                switch ((ConstraintGeneratorType)continuationId.Type)
                {
                    case ConstraintGeneratorType.Direct:
                        //Direct has no need for accumulating multiple reports; we can immediately dispatch.
                        narrowPhase.UpdateConstraintsForPair(workerIndex, ref discrete.Caches[continuationId.Index], manifold, ref todoTestCollisionCache);
                        break;
                    case ConstraintGeneratorType.Linear:
                        break;
                    case ConstraintGeneratorType.Substep:
                        break;
                    case ConstraintGeneratorType.SubstepWithLinear:
                        break;
                }
                
            }
        }

        ConstraintGenerators[] constraintGenerators;

        private void PrepareConstraintGenerators(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            //Resizes should be very rare, and having a single extra very small array isn't concerning.
            //(It's not an unmanaged type because it contains nonblittable references.)
            if (constraintGenerators == null || constraintGenerators.Length < threadCount)
                Array.Resize(ref constraintGenerators, threadCount);
            for (int i = 0; i < threadCount; ++i)
            {
                constraintGenerators[i] = new ConstraintGenerators(i, threadDispatcher != null ? threadDispatcher.GetThreadMemoryPool(i) : Pool, this);
            }
        }


    }


}
