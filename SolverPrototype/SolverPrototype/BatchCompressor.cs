using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace SolverPrototype
{
    /// <summary>
    /// Handles the movement of constraints from higher indexed batches into lower indexed batches to avoid accumulating a bunch of unnecessary ConstraintBatches.
    /// </summary>
    public class BatchCompressor<TBodies> where 
    {
        //We want to keep removes as fast as possible. So, when removing constraints, no attempt is made to pull constraints from higher constraint batches into the revealed slot.
        //Over time, this could result in lots of extra constraint batches that ruin multithreading performance.
        //This batch compressor solves this problem over multiple frames.
        //The dedicated batch analysis has some pretty nice advantages:
        //0) Removes stay (relatively) fast- no O(n) searching or complex logic.
        //1) High churn adds/removes are extremely common during chaotic collisions, which is exactly when you need as little overhead as possible. 
        //1.5) High churn situations will tend to rapidly invalidate the 'optimization' effort of extremely aggressive on-remove swaps.
        //2) On-removal will often fail to make any change due to other reference blockages.
        //3) Dedicated batch analysis can be deferred over multiple frames because the intermediate results are all fine from a correctness standpoint. 
        //3.5) Deferred costs can be kept consistently low no matter what kind of add/remove churn is happening.
        //4) Dedicated batch analysis can be performed asynchronously and hidden behind other stally stages which aren't the solver and don't modify the solver (e.g. broadphase, midphase).
        //5) Even if we are in a 'suboptimal' constraint configuration (i.e. some pulldowns exist), it will rarely have an effect on performance unless it actually results in extra batches.
        //6) Dedicated analysis could afford to perform more complex heuristics to optimize batches. This doesn't do anything clever, but in theory, it could.

        //Note that this is trying to solve an NP hard problem (graph edge coloring) with a strictly greedy approach, so it won't produce optimal results.
        //(In fact, it could use almost twice as many batches as would be optimal!)
        //There's probably an argument to be made for a slightly more clever approach later on- consider that the BVH SAH optimization is NP hard too, but the 
        //broadphase's incremental refinement does pretty well (and quite a bit better than pure greedy approaches). It is likely we can create something 
        //similar here. Some possibilities:
        //1) Grab a random set of N constraints from each batch, speculatively remove all of them and 'rebatch' them with a more expensive local
        //optimizer. The rebatching would have to take into account all the other bodies represented in the batches at each level. There is no guarantee this
        //would make progress unless the 'rebatching' algorithm was provably better than greedy and you had a sufficiently large rebatching region.
        //2) Claim a region of the constraint graph by traversing it. Execute misra & gries or similar not-exponential offline operations on it.
        //3) Maybe claim a smaller region and use an optimal (exponential) algorithm. I suspect covering a large region would work better.
        //This would be similar to treelet rotations in BVH refinement.

        //(Keep in mind that the difference between an optimal coloring and the greedy coloring is pretty small as far as solve time goes.
        //Expect less than 5% impact. In other words, it's not worth spending a month on a research project when there's so many other options to pursue.)

        public Solver Solver { get; private set; }
        public Bodies Bodies { get; private set; }
        float targetCandidateFraction;
        float maximumCompressionFraction;
        /// <summary>
        /// Gets or sets the desired number of candidates to analyze as a fraction of the total number of constraints.
        /// </summary>
        public float TargetCandidateFraction
        {
            get { return targetCandidateFraction; }
            set
            {
                if (value < 0 || value > 1) throw new ArgumentException("Fraction must be from 0 to 1.");
                targetCandidateFraction = value;
            }
        }
        /// <summary>
        /// Gets or sets the maximum number of constraint moves that can occur in a single execution of Compress as a fraction of the total number of constraints.
        /// </summary>       
        public float MaximumCompressionFraction
        {
            get { return maximumCompressionFraction; }
            set
            {
                if (value < 0 || value > 1) throw new ArgumentException("Fraction must be from 0 to 1.");
                maximumCompressionFraction = value;
            }
        }
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

        //Note that these lists do not contain any valid information between frames- they only exist for the duration of the optimization.
        struct WorkerContext
        {
            public QuickList<Compression, Buffer<Compression>> Compressions;
            public AnalysisRegion Region;
        }
        //Note that we allocate all of the contexts and their compressions on demand. It's all pointer backed, so there's no worries about GC reference tracing.
        Buffer<WorkerContext> workerContexts;


        Action<int> analysisWorkerDelegate;
        public BatchCompressor(Solver solver, Bodies bodies, float targetCandidateFraction = 0.01f, float maximumCompressionFraction = 0.0005f)
        {
            this.Solver = solver;
            this.Bodies = bodies;
            TargetCandidateFraction = targetCandidateFraction;
            this.MaximumCompressionFraction = maximumCompressionFraction;
            analysisWorkerDelegate = AnalysisWorker;
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


        //Note that we split the find and apply stages conceptually just because there is a decent chance that they'll be scheduled at different times.
        //That is, while you can run the analysis phase in parallel, you can't run the application in parallel. So, if you can hide the sequential application in an unrelated
        //phase, it could be a net win. However, oversubscribing a parallel analysis phase on top of another parallel phase might not be quite as wise.
        //It'll just require some testing.
        //(The broad phase is a pretty likely candidate for this overlay- it both causes no changes in constraints and is very stally compared to most other phases.)

        int compressionsCount;
        unsafe void AnalysisWorker(int workerId)
        {
            ref var context = ref workerContexts[workerId];
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
                ConstraintBodyHandleCollector indexAccumulator;
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
                        if (Solver.Batches[batchIndex].CanFit(ref bodyHandles[0], bodiesPerConstraint))
                        {
                            //This constraint can move down!
                            //Note that we don't add it if the new index would put the total number of compression targets above the maximum for this pass.
                            //We might have wasted some time getting to this point, but in practice, that doesn't really matter so long as compressions are rare relative
                            //to analysis.
                            if (Interlocked.Increment(ref compressionsCount) > maximumCompressionCount)
                                return;
                            //Note that we build the compressions list with a maximum sized backing array. Don't have to worry about resizing.
                            //IMPORTANT: The compression lists must be created per worker. The order of elements in the compressions lists are important.
                            //The compression-application phase will iterate over the workers in order. Since the analysis regions were created in order,
                            //and because the workers generate compressions in order, the application phase has a fully ordered list of compressions.
                            //This is required to avoid early compressions invalidating later compression targets.
                            context.Compressions.AddUnsafely(new Compression { IndexInTypeBatch = i, TargetBatch = batchIndex, TypeBatchIndex = typeBatchIndex });
                            break;
                        }
                    }
                }

                remainder -= availableConstraints;
                indexInTypeBatch = end;
                if (indexInTypeBatch == typeBatch.ConstraintCount && remainder > 0)
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
        void ScheduleAnalysisRegions(int workerCount, BufferPool rawPool)
        {
            rawPool.SpecializeFor<WorkerContext>().Take(workerCount, out workerContexts);
            for (int i = 0; i < workerCount; ++i)
            {
                QuickList<Compression, Buffer<Compression>>.Create(rawPool.SpecializeFor<Compression>(), maximumCompressionCount, out workerContexts[i].Compressions);
            }

            //In any given compression attempt, we only optimize over one ConstraintBatch.
            //This provides a guarantee that every optimization that occurs over the course of the compression
            //does not affect any other optimization, because a ConstraintBatch guarantees that bodies are only referenced by a single constraint.
            //That's useful when multithreading- we don't have to worry about what candidates other threads have found.

            //Note that the application of compression is sequential. Solver.Add and Solver.Remove can't be called from multiple threads. So when multithreading,
            //only the candidate analysis is actually multithreaded. That's fine- actual compressions are actually pretty rare in nonpathological cases!


            //Note that this is similar to the ConstraintLayoutOptimizer in terms of batch walking. The differences are:
            //1) We don't have to worry about nonzero offsets, and
            //2) there is only one progression index set, and
            //3) we operate on constraint indices rather than bundle indices, and
            //4) the batch compressor cannot walk to the next batch during a single pass, since that would invalidate the guarantees required for safe multithreaded analysis.


            //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
            Debug.Assert(Solver.Batches.Count > 0);
            if (nextBatchIndex >= Solver.Batches.Count)
            {
                //Invalid batch; wrap to the first one.
                nextBatchIndex = 0;
                nextTarget = new AnalysisStart();
            }
            else if (nextTarget.TypeBatchIndex >= Solver.Batches[nextBatchIndex].TypeBatches.Count)
            {
                //Invalid type batch; move to the next batch.
                ++nextBatchIndex;
                if (nextBatchIndex >= Solver.Batches.Count)
                    nextBatchIndex = 0;
                nextTarget = new AnalysisStart();
            }
            else if (nextTarget.StartIndexInTypeBatch >= Solver.Batches[nextBatchIndex].TypeBatches[nextTarget.TypeBatchIndex].ConstraintCount)
            {
                //Invalid constraint index; move to the next type batch.
                nextTarget.StartIndexInTypeBatch = 0;
                ++nextTarget.TypeBatchIndex;
                //Check if we have to move to the next batch.
                if (nextTarget.TypeBatchIndex >= Solver.Batches[nextBatchIndex].TypeBatches.Count)
                {
                    nextTarget.TypeBatchIndex = 0;
                    ++nextBatchIndex;
                    if (nextBatchIndex >= Solver.Batches.Count)
                        nextBatchIndex = 0;
                }
            }

            //Console.WriteLine($"start: batch {nextBatchIndex}, type batch {nextTarget.TypeBatchIndex}, index {nextTarget.StartIndexInTypeBatch}");

            //Build the analysis regions.
            var batch = Solver.Batches[nextBatchIndex];

            var remainingConstraintCount = -nextTarget.StartIndexInTypeBatch;
            for (int i = nextTarget.TypeBatchIndex; i < batch.TypeBatches.Count; ++i)
            {
                remainingConstraintCount += batch.TypeBatches[i].ConstraintCount;
                if (remainingConstraintCount > targetCandidateCount)
                {
                    remainingConstraintCount = targetCandidateCount;
                    break;
                }
            }
            var constraintsPerWorkerBase = remainingConstraintCount / workerCount;
            var constraintsRemainder = remainingConstraintCount - workerCount * constraintsPerWorkerBase;
            //var DEBUGTotalConstraintsAnalyzed = 0;
            for (int i = 0; i < workerCount; ++i)
            {
                ref var context = ref workerContexts[i];
                context.Region.ConstraintCount = constraintsPerWorkerBase + (--constraintsRemainder > 0 ? 1 : 0);
                context.Region.Start = nextTarget;
                nextTarget.StartIndexInTypeBatch += context.Region.ConstraintCount;
                //DEBUGTotalConstraintsAnalyzed += region.ConstraintCount;
            }

            //if (workerContexts[0].Compressions.Count > 0)
            //    Console.WriteLine($"Analyzed {DEBUGTotalConstraintsAnalyzed} constraints, compressing {workerContexts[0].Compressions.Count}");

            //Note that it is possible for this to skip some compressions if the maximum number of compressions is hit before all candidates are visited. That's fine;
            //compressions should be rare under normal circumstances and having to wait some frames for the analysis to swing back through shouldn't be a problem.

        }

        /// <summary>
        /// Applies the set of compressions found in the previous call to FindCompressions.
        /// </summary>
        void ApplyCompressions(int workerCount, BufferPool rawPool)
        {
            //Walk the list of workers in reverse order. We assume here that each worker has generated its results in low to high order as well.
            //So, by walking backwards sequentially, we ensure that early removals will not interfere with later removals 
            //because removals take the last element and pull it into the removed slot, leaving the earlier section of the arrays untouched.
            var sourceBatch = Solver.Batches[nextBatchIndex];
            for (int i = workerCount - 1; i >= 0; --i)
            {
                ref var context = ref workerContexts[i];
                for (int j = context.Compressions.Count - 1; j >= 0; --j)
                {
                    ref var compression = ref context.Compressions[j];
                    //Note that we do not simply remove and re-add the constraint; while that would work, it would redo a lot of work that isn't necessary.
                    //Instead, since we already know exactly where the constraint is and what constraint batch it should go to, we can avoid a lot of abstractions
                    //and do more direct copies.
                    sourceBatch.TypeBatches[compression.TypeBatchIndex].TransferConstraint(
                        nextBatchIndex, compression.IndexInTypeBatch, Solver, Bodies, compression.TargetBatch);
                }
            }

            for (int i = 0; i < workerCount; ++i)
            {
                workerContexts[i].Compressions.Dispose(rawPool.SpecializeFor<Compression>());
            }
            rawPool.SpecializeFor<WorkerContext>().Return(ref workerContexts);
        }

        int maximumCompressionCount;
        int targetCandidateCount;
        /// <summary>
        /// Incrementally finds and applies a set of compressions to apply to the constraints in the solver's batches.
        /// Constraints in higher index batches try to move to lower index batches whenever possible.
        /// </summary>
        public void Compress(BufferPool rawPool, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher != null ? threadDispatcher.ThreadCount : 1;
            var constraintCount = Solver.ConstraintCount;
            //Early out if there are no constraints to compress. The existence of constraints is assumed in some of the subsequent stages, so this is not merely an optimization.
            if (constraintCount == 0)
                return;
            maximumCompressionCount = (int)Math.Max(1, Math.Round(MaximumCompressionFraction * constraintCount));
            targetCandidateCount = (int)Math.Max(1, Math.Round(TargetCandidateFraction * constraintCount));
            ScheduleAnalysisRegions(workerCount, rawPool);
            if (threadDispatcher != null)
            {
                threadDispatcher.DispatchWorkers(analysisWorkerDelegate);
            }
            else
            {
                AnalysisWorker(0);
            }
            //It's possible for multiple workers to hit the interlocked increment and bump it above the maximum. Clamp it.
            if (compressionsCount > maximumCompressionCount)
                compressionsCount = maximumCompressionCount;

            ApplyCompressions(workerCount, rawPool);
        }
    }
}
