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
        /// One of potentially multiple substeps produced by a collidable pair using substepped continuous collision detection.
        /// </summary>
        Substep = 1,
        /// <summary>
        /// Inner sphere test associated with a collidable pair using inner sphere continuous collision detection.
        /// </summary>
        InnerSphere = 2
    }



    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        public struct ConstraintGenerators : IContinuations
        {
            int workerIndex;
            NarrowPhase<TCallbacks> narrowPhase;
            public ConstraintGenerators(int workerIndex, NarrowPhase<TCallbacks> narrowPhase)
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
                constraintGenerators[i] = new ConstraintGenerators(i, this);
            }
        }


    }


}
