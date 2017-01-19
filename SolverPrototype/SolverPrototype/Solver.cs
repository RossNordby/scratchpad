using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Reference to a particular constraint in a type batch.
    /// </summary>
    /// <typeparam name="T">Type of the batch referenced.</typeparam>
    public struct ConstraintReference<T>
    {
        public T TypeBatch;
        public int IndexInTypeBatch;
    }


    public class Solver
    {
        QuickList<ConstraintBatch> batches;

        int iterationCount;
        /// <summary>
        /// Gets or sets the number of solver iterations to compute per call to Update.
        /// </summary>
        public int IterationCount
        {
            get { return iterationCount; }
            set
            {
                if (value < 1)
                {
                    throw new ArgumentException("Iteration count must be positive.");
                }
                iterationCount = value;
            }
        }

        Bodies bodies;

        struct ConstraintLocation
        {
            //Note that the type id is omitted. To be useful, any access of a constraint pointer is expected to supply a generic type parameter.
            //The type id can be retrieved from that efficiently. We may change this to store it later if typeless access patterns become common.
            public int BatchIndex;
            public int IndexInTypeBatch;
        }
        IdPool handlePool = new IdPool();
        ConstraintLocation[] HandlesToConstraints;

        public Solver(Bodies bodies, int iterationCount = 5, int initialCapacity = 128)
        {
            this.iterationCount = iterationCount;
            this.bodies = bodies;
            batches = new QuickList<ConstraintBatch>(new PassthroughBufferPool<ConstraintBatch>());
            HandlesToConstraints = new ConstraintLocation[initialCapacity];
        }


        public void GetConstraintPointer<T>(int handleIndex, out ConstraintReference<T> constraintPointer) where T : TypeBatch
        {
            ref var constraintLocation = ref HandlesToConstraints[handleIndex];
            constraintPointer.TypeBatch = batches.Elements[constraintLocation.BatchIndex].GetTypeBatch<T>();
            constraintPointer.IndexInTypeBatch = constraintLocation.IndexInTypeBatch;
        }

        public void Allocate<T>(int bodyHandleA, int bodyHandleB, out ConstraintReference<T> constraintReference, out int handleIndex) where T : TypeBatch, new()
        {
            int targetBatchIndex = -1;
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches[i];
                if (!batch.Handles.Contains(bodyHandleA) &&
                    !batch.Handles.Contains(bodyHandleB))
                {
                    targetBatchIndex = i;
                }
            }
            ConstraintBatch targetBatch;
            if (targetBatchIndex == -1)
            {
                //No batch available. Have to create a new one.
                targetBatch = new ConstraintBatch();
                targetBatchIndex = batches.Count;
                batches.Add(targetBatch);
            }
            else
            {
                targetBatch = batches.Elements[targetBatchIndex];
            }
            targetBatch.Handles.Add(bodyHandleA);
            targetBatch.Handles.Add(bodyHandleB);
            targetBatch.Allocate(out constraintReference);

            handleIndex = handlePool.Take();
            if (handleIndex >= HandlesToConstraints.Length)
            {
                Array.Resize(ref HandlesToConstraints, HandlesToConstraints.Length << 1);
                Debug.Assert(handleIndex < HandlesToConstraints.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            HandlesToConstraints[handleIndex].IndexInTypeBatch = constraintReference.IndexInTypeBatch;
            HandlesToConstraints[handleIndex].BatchIndex = targetBatchIndex;
        }

        //TODO: Note that removals are a little tricky. In order to reduce the number of batches which persist, every removal
        //should attempt to fill the entity reference gap left by the removal with a constraint from another batch if possible.
        //Unfortunately, this requires a way to look up the constraint that is holding a given reference. That makes the
        //bitfield approach insufficient- have to store indices...
        //An alternative is to keep remove as simple and direct as possible, possibly leaving more batches than there would ideally be.
        //Then, later, a dedicated batch analysis process tries to find any potential compressions.
        //The dedicated batch analysis has some pretty nice advantages:
        //0) Removes stay (relatively) fast- no O(n) searching or complex logic.
        //1) High churn adds/removes are extremely common during chaotic collisions, which is exactly when you need as little overhead as possible. 
        //1.5) High churn situations will tend to rapidly invalidate the 'optimization' effort of extremely aggressive on-remove swaps.
        //2) On-removal will often fail to make any change due to other reference blockages.
        //3) Dedicated batch analysis can be deferred over multiple frames because the intermediate results are all fine from a correctness standpoint. 
        //3.5) Deferred costs can be kept consistently low no matter what kind of add/remove churn is happening.
        //4) Dedicated batch analysis can be performed asynchronously and hidden behind other stally stages which aren't the solver and don't modify the solver (e.g. broadphase, midphase).
        //5) Even if we are in a 'suboptimal' constraint configuration (i.e. some pulldowns exist), it will rarely have an effect on performance unless it actually results in extra batches.
        //6) Dedicated analysis could afford to perform more complex heuristics to optimize batches.

        public void Update(float dt, float inverseDt)
        {
            //TODO: Note that the prestep phase is completely parallel. When multithreading, there is no need to stop at the boundaries of bodybatches.
            //You could technically build a separate list that ignores bodybatch boundaries. Value is unclear.
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].Prestep(bodies.LocalInertiaBundles, dt, inverseDt);
                }
            }
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].WarmStart(bodies.VelocityBundles);
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < batches.Count; ++i)
                {
                    var batch = batches.Elements[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        batch.TypeBatches.Elements[j].SolveIteration(bodies.VelocityBundles);
                    }
                }
            }
        }

        //This is a pure debug thing. Remember, it modifies the state of the simulation...
        public float GetVelocityChangeHeuristic()
        {
            float accumulatedChanges = 0;
            foreach (var batch in batches)
            {
                for (int i = 0; i < batch.TypeBatches.Count; ++i)
                {
                    //Replace unsafe cast with virtual call once we have other types that handle.
                    accumulatedChanges += Unsafe.As<TypeBatch, ContactPenetrationTypeBatch>(ref batch.TypeBatches.Elements[i]).GetVelocityChangeHeuristic(bodies.VelocityBundles);
                }
            }
            return accumulatedChanges;
        }
    }
}
