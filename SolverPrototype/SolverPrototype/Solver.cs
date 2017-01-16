using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;

namespace SolverPrototype
{
    public class Solver
    {
        QuickList<ConstraintBatch> batches;
        QuickList<TypeBatch> solveBatches;

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

        public Solver(int iterationCount = 5)
        {
            this.iterationCount = iterationCount;
            batches = new QuickList<ConstraintBatch>(new PassthroughBufferPool<ConstraintBatch>());
        }

        public void Add<T>(ref T constraint) where T : ITwoBodyConstraintDescription
        {
            ConstraintBatch targetBatch = null;
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches[i];
                if (!batch.Handles.Contains(constraint.BodyHandleA) &&
                    !batch.Handles.Contains(constraint.BodyHandleB))
                {
                    targetBatch = batch;
                }
            }
            if (targetBatch == null)
            {
                //No batch available. Have to create a new one.
                targetBatch = new ConstraintBatch();
                batches.Add(targetBatch);
            }
            targetBatch.Handles.Add(constraint.BodyHandleA);
            targetBatch.Handles.Add(constraint.BodyHandleB);
            targetBatch.Add(ref constraint);
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

        public void Update()
        {
            //TODO: Note that the prestep phase is completely parallel. When multithreading, there is no need to stop at the boundaries of bodybatches.
            //You could technically build a separate list that ignores bodybatch boundaries. Value is unclear.
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches.Elements[i];
                batch.TypeBatches.Elements[j].Prestep();

            }
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].WarmStart();
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < batches.Count; ++i)
                {
                    var batch = batches.Elements[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        batch.TypeBatches.Elements[j].SolveIteration();
                    }
                }
            }
        }
    }
}
