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

            struct Linear
            {
                public ContactManifold Discrete;
                public ContactManifold LinearA;
                public ContactManifold LinearB;
                public int ManifoldsReported;
            }

            struct Substep
            {
                public QuickList<ContactManifold, Buffer<ContactManifold>> Manifolds;
                public int ManifoldsReported;
                public Substep(BufferPool pool, int capacity)
                {
                    QuickList<ContactManifold, Buffer<ContactManifold>>.Create(pool.SpecializeFor<ContactManifold>(), capacity, out Manifolds);
                    ManifoldsReported = 0;
                }
            }

            struct SubstepWithLinear
            {
                public ContactManifold LinearA;
                public ContactManifold LinearB;
                public Substep Substep;

                public SubstepWithLinear(BufferPool pool, int capacity)
                {
                    Substep = new Substep(pool, capacity);
                    LinearA = new ContactManifold();
                    LinearB = new ContactManifold();
                }
            }



            Buffer<CollidablePair> direct;
            Buffer<Linear> linear;
            Buffer<Substep> substep;
            Buffer<SubstepWithLinear> substepWithLinear;


            public ConstraintGenerators(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
            }
            public unsafe void Notify(TypedIndex continuationId, ContactManifold* manifold)
            {
                var todoTestCollisionCache = default(EmptyCollisionCache);
                narrowPhase.UpdateConstraintsForPair(workerIndex, ref pair, manifold, ref todoTestCollisionCache);
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
