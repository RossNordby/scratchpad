using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;

namespace SolverPrototype
{
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

        public void Update()
        {
            //TODO: Note that the prestep phase is completely parallel. When multithreading, there is no need to stop at the boundaries of bodybatches.
            //You could technically build a separate list that ignores bodybatch boundaries. Value is unclear.
            for (int i = 0; i < batches.Count; ++i)
            {
                var batch = batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].Prestep();
                }
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
