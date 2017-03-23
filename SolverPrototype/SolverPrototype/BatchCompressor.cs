using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SolverPrototype
{
    /// <summary>
    /// Handles the movement of constraints from higher indexed batches into lower indexed batches to avoid accumulating a bunch of unnecessary ConstraintBatches.
    /// </summary>
    public class BatchCompressor
    {
        public Solver Solver { get; private set; }
        public Bodies Bodies { get; private set; }
        /// <summary>
        /// Gets or sets the desired number of candidates to analyze.
        /// </summary>
        public int TargetCandidateCount;

        struct Compression
        {
            //Since we have an ordered list of compression targets all in the same batch, and because we're going to do the compressions sequentially from high to low,
            //we can directly cache the source indices.
            public int TypeBatchIndex;
            public int IndexInTypeBatch;
            //We don't cache the batch since it's the same for all compressions in a given pass.
            //Because we have to do CanFits ahead of time, the target is reliable. We don't know where they're going inside of the batch, but that's relatively cheap to figure out.
            //We don't even have to do the usual handle->index conversion, since we're just copying a lane from its old position.
            public int TargetBatch;
        }

        //We could allocate fixed length stuff and whatnot, but working on the assumption that we'll have a higher performance instance-level pool for value types,
        //we'll just use a quicklist array for now. When the instance level pool exists, we can swap out the quick lists.
        //Note that we don't expect to pool the actual array of lists- it is limited in size to the number of worker threads, which is pretty darn low for the foreseeable future
        //(less than 1024 entries even for a machine that costs six figures). .. ... .... On the other hand, since a pointer-backed list won't be reference based, we COULD just
        //allocate it all on demand. That would save space, technically. We'll see!
        struct WorkerContext
        {
            public QuickList<Compression> Compressions;
            public AnalysisRegion Region;
        }
        WorkerContext[] workerContexts;
        PassthroughBufferPool<Compression> compressionPool = new PassthroughBufferPool<Compression>();
        /// <summary>
        /// Gets or sets the maximum number of constraint moves that can occur in a single execution of Compress.
        /// </summary>
        int MaximumCompressionCount;


        public BatchCompressor(Solver solver, Bodies bodies, int targetCandidateCount = 1024, int maximumCompressionCount = 128, int expectedWorkerThreadCount = 1)
        {
            this.Solver = solver;
            this.Bodies = bodies;
            TargetCandidateCount = targetCandidateCount;
            this.MaximumCompressionCount = maximumCompressionCount;

            workerContexts = new WorkerContext[expectedWorkerThreadCount];

            for (int i = 0; i < expectedWorkerThreadCount; ++i)
            {
                workerContexts[i].Compressions = new QuickList<Compression>(compressionPool);
            }
        }
        struct AnalysisStart
        {
            /// <summary>
            /// Index of the target type batch.
            /// </summary>
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the target constraint to analyze in a type batch.
            /// </summary>
            public int StartIndexInTypeBatch;
        }

        struct AnalysisRegion
        {
            public AnalysisStart Start;
            /// <summary>
            /// Number of constraints in the region.
            /// </summary>
            public int ConstraintCount;
            //Note that no ConstraintBatch index is stored; all targets share the same batch in a given pass.

        }
        AnalysisStart nextTarget;
        /// <summary>
        /// Index of the constraint batch to optimize.
        /// </summary>
        int nextBatchIndex;
        
        /// <summary>
        /// Incrementally finds and applies a set of compressions to apply to the constraints in the solver's batches.
        /// Constraints in higher index batches try to move to lower index batches whenever possible.
        /// </summary>
        public void Compress()
        {
            //TODO: Worker thread count needs to conditionally resize the compression sets.
            FindCompressions(1);
            ApplyCompressions();
        }

        //Note that we split the find and apply stages conceptually just because there is a decent chance that they'll be scheduled at different times.
        //That is, while you can run the analysis phase in parallel, you can't run the application in parallel. So, if you can hide the sequential application in an unrelated
        //phase, it could be a net win. However, oversubscribing a parallel analysis phase on top of another parallel phase might not be quite as wise.
        //It'll just require some testing.
        //(The broad phase is a pretty likely candidate for this overlay- it both causes no changes in constraints and is very stally compared to most other phases.)

        unsafe struct BodyIndexAccumulator : IForEach<int>
        {
            public Bodies Bodies;
            public int* Handles;
            public int Index;

            public void LoopBody(int handleIndex)
            {
                Handles[Index++] = Bodies.IndexToHandle[handleIndex];
            }
        }

        int foundCompressionsCount;
        unsafe void AnalysisWorker(int workerId)
        {
            ref var context = ref workerContexts[workerId];
            context.Compressions.Count = 0;
            var batch = Solver.Batches[nextBatchIndex];
            var typeBatchIndex = context.Region.Start.TypeBatchIndex;
            var typeBatch = batch.TypeBatches[typeBatchIndex];
            var indexInTypeBatch = context.Region.Start.StartIndexInTypeBatch;

            //This region may cover multiple type batches. Each iteration goes up to either the constraint count, or the end of the current type batch.
            var remainder = context.Region.ConstraintCount;
            while (remainder > 0)
            {
                var availableConstraints = Math.Min(remainder, typeBatch.ConstraintCount - indexInTypeBatch);

                var end = indexInTypeBatch + availableConstraints;
                var bodiesPerConstraint = typeBatch.BodiesPerConstraint;
                var bodyHandles = stackalloc int[bodiesPerConstraint];
                BodyIndexAccumulator indexAccumulator;
                indexAccumulator.Bodies = Bodies;
                indexAccumulator.Handles = bodyHandles;
                for (int i = indexInTypeBatch; i < end; ++i)
                {
                    //Check if this constraint can be removed.
                    //Note that we go in order from lowest index to highest; this is important. The results from the workers will be traversed as if they were one long sorted list.

                    //Note that it is possible to early out if the totalCompressionCount has already reached the max, but doing that every single iteration would create a lot of
                    //multithreaded cache contention. Generally, we expect compressions to be quite rare relative to examined elements, so aggressively performing early outs
                    //would actually worsen the worst case for analysis (where there aren't any compressions!).

                    indexAccumulator.Index = 0;
                    typeBatch.EnumerateConnectedBodyIndices(i, ref indexAccumulator);
                    for (int batchIndex = 0; batchIndex < nextBatchIndex; ++batchIndex)
                    {
                        if (Solver.Batches[i].CanFit(ref bodyHandles[0], bodiesPerConstraint))
                        {
                            //This constraint can move down!
                            //Note that we don't add it if the new index would put the total number of compression targets above the maximum for this pass.
                            //We might have wasted some time getting to this point, but in practice, that doesn't really matter so long as compressions are rare relative
                            //to analysis.
                            if (Interlocked.Increment(ref foundCompressionsCount) > MaximumCompressionCount)
                                return;
                            context.Compressions.Add(new Compression { IndexInTypeBatch = i, TargetBatch = batchIndex, TypeBatchIndex = typeBatchIndex });
                            break;
                        }
                    }
                }

                remainder -= availableConstraints;
                indexInTypeBatch = end;
                if (indexInTypeBatch == typeBatch.ConstraintCount)
                {
                    //If it wasn't large enough, then we need to push the type batch to the next slot.
                    //The region creation phase should guarantee that this is a valid thing to do.
                    Debug.Assert(typeBatchIndex + 1 < batch.TypeBatches.Count, "The worker region generator should guarantee that the regions cover valid areas.");
                    ++typeBatchIndex;
                    typeBatch = batch.TypeBatches[typeBatchIndex];
                }
            }


        }

        /// <summary>
        /// Identifies a set of constraints that can be moved into lower batches. Only handles a subset of one constraint batch at a time.
        /// </summary>
        void FindCompressions(int workerCount)
        {
            //In any given compression attempt, we only optimize over one ConstraintBatch.
            //This provides a guarantee that every optimization that occurs over the course of the compression
            //does not affect any other optimization, because a ConstraintBatch guarantees that bodies are only referenced by a single constraint.
            //That's useful when multithreading- we don't have to worry about what candidates other threads have found.

            //Note that the application of compression is sequential. Solver.Add and Solver.Remove can't be called from multiple threads. So when multithreading,
            //only the candidate analysis is actually multithreaded. That's fine- actual compressions are actually pretty rare in nonpathological cases!


            //Note that this is similar to the ConstraintLayoutOptimizer in terms of batch walking. The differences are:
            //1) We don't have to worry about nonzero offsets, and
            //2) there is only one progression index set, and
            //3) we operate on constraint indices rather than bundle indices.
            //4) The batch compressor cannot walk to the next batch during a single pass, since that would invalidate the guarantees required for safe multithreaded analysis.
            bool WrapBatch(ref AnalysisRegion target)
            {
                Debug.Assert(Solver.Batches.Count >= 0);
                if (target.BatchIndex >= Solver.Batches.Count)
                {
                    //Wrap around.
                    target = new AnalysisRegion();
                    return true;
                }
                return false;
            }
            bool WrapTypeBatch(ref AnalysisRegion o)
            {
                Debug.Assert(o.BatchIndex <= Solver.Batches.Count, "Should only attempt to wrap type batch indices if the batch index is known to be valid.");
                if (o.TypeBatchIndex >= Solver.Batches[o.BatchIndex].TypeBatches.Count)
                {
                    ++o.BatchIndex;
                    if (!WrapBatch(ref o))
                    {
                        o.TypeBatchIndex = 0;
                        o.StartIndexInTypeBatch = 0;
                    }
                    return true;
                }
                return false;
            }
            void WrapBundle(ref AnalysisRegion o, ref int batchIndex)
            {
                Debug.Assert(o.BatchIndex <= Solver.Batches.Count && o.TypeBatchIndex <= Solver.Batches[o.BatchIndex].TypeBatches.Count,
                    "Should only attempt to wrap constraint index if the type batch and batch indices are known to be valid.");
                if (o.StartIndexInTypeBatch >= Solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].ConstraintCount)
                {
                    ++o.TypeBatchIndex;
                    if (!WrapTypeBatch(ref o))
                    {
                        o.StartIndexInTypeBatch = 0;
                    }
                }
            }
            //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
            if (!WrapBatch(ref nextTarget))
            {
                if (!WrapTypeBatch(ref nextTarget))
                {
                    WrapBundle(ref nextTarget);
                }
            }

            //Build the analysis regions.
            var batch = Solver.Batches[nextBatchIndex];

            var remainingConstraintCount = -nextTarget.StartIndexInTypeBatch;
            for (int i = nextTarget.TypeBatchIndex; i < batch.TypeBatches.Count; ++i)
            {
                remainingConstraintCount += batch.TypeBatches[i].ConstraintCount;
            }
            var constraintsPerWorkerBase = remainingConstraintCount / workerCount;
            var constraintsRemainder = remainingConstraintCount - workerCount * constraintsPerWorkerBase;
            for (int i = 0; i < workerCount; ++i)
            {
                ref var context = ref workerContexts[i];
                context.Region.ConstraintCount = constraintsPerWorkerBase + (--constraintsRemainder > 0 ? 1 : 0);
                context.Region.Start = nextTarget;
                nextTarget.StartIndexInTypeBatch += context.Region.ConstraintCount;
                WrapBundle(ref nextTarget);
            }
            do
            {
                //Add the initial target for optimization. It's already been validated- either by the initial test, or by the previous wrap.
                targets.Add(ref nextTarget);
                nextTarget.BundleIndex += maximumRegionSizeInBundles;
                WrapBundle(ref nextTarget);

                //If the next target overlaps with the first target, the collection has wrapped around all constraints. Apparently more regions were requested
                //than are available. Stop collection.
                ref var firstTarget = ref targets.Elements[0];
                if (nextTarget.BatchIndex == firstTarget.BatchIndex &&
                    nextTarget.TypeBatchIndex == firstTarget.TypeBatchIndex &&
                    nextTarget.BundleIndex < firstTarget.BundleIndex + maximumRegionSizeInBundles)
                {
                    break;
                }
            }
            while (targets.Count < regionCount);

            //For now, we only have one worker.
            foundCompressionsCount = 0;
            AnalysisWorker(0);

            //Note that it is possible for this to skip 
            //Now we've got the set of compressions. Apply them.

        }

        /// <summary>
        /// Applies the set of compressions found in the previous call to FindCompressions.
        /// </summary>
        void ApplyCompressions()
        {
            Debug.Assert(foundCompressionsCount < compressions.Length);
        }
    }
}
