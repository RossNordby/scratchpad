using System;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Constraints;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using System.Diagnostics;

namespace DemoRenderer.Constraints
{
    interface IConstraintLineExtractor<TBodyReferences, TPrestep>
    {
        int LinesPerConstraint { get; }

        void ExtractLines(ref TPrestep prestepBundle, ref TBodyReferences referencesBundle, int innerIndex, Bodies bodies, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }
    abstract class TypeLineExtractor
    {
        public abstract int LinesPerConstraint { get; }
        public abstract void ExtractLines(Simulation simulation, TypeBatch typeBatch, int constraintStart, int constraintCount, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }

    class TypeLineExtractor<T, TTypeBatch, TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses> : TypeLineExtractor
        where T : struct, IConstraintLineExtractor<TBodyReferences, TPrestep>
        where TTypeBatch : TypeBatch<TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses>
    {
        public override int LinesPerConstraint => default(T).LinesPerConstraint;
        public override void ExtractLines(Simulation simulation, TypeBatch typeBatch, int constraintStart, int constraintCount,
            ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            var batch = (TTypeBatch)typeBatch;
            ref var prestepStart = ref batch.PrestepData[0];
            ref var referencesStart = ref batch.BodyReferences[0];
            var extractor = default(T);

            var constraintEnd = constraintStart + constraintCount;
            for (int i = constraintStart; i < constraintEnd; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                extractor.ExtractLines(ref prestepBundle, ref referencesBundle, innerIndex, simulation.Bodies, ref lines);
            }
        }
    }

    public class LineExtractor
    {
        TypeLineExtractor[] lineExtractors;
        internal QuickList<LineInstance, Array<LineInstance>> lines;
        const int jobsPerThread = 4;
        QuickList<ThreadJob, Array<ThreadJob>> jobs;

        struct ThreadJob
        {
            public int BatchIndex;
            public int TypeBatchIndex;
            public int ConstraintStart;
            public int ConstraintCount;
            public int LineStart;
            public int LinesPerConstraint;
        }


        public LineExtractor(int initialLineCapacity = 8192)
        {
            QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), initialLineCapacity, out lines);
            lineExtractors = new TypeLineExtractor[TypeIds<TypeBatch>.RegisteredTypeCount];
            lineExtractors[TypeIds<TypeBatch>.GetId<BallSocketTypeBatch>()] =
                new TypeLineExtractor<BallSocketLineExtractor, BallSocketTypeBatch, TwoBodyReferences, BallSocketPrestepData, BallSocketProjection, Vector3Wide>();
            QuickList<ThreadJob, Array<ThreadJob>>.Create(new PassthroughArrayPool<ThreadJob>(), Environment.ProcessorCount * (jobsPerThread + 1), out jobs);
        }

        public void ClearInstances()
        {
            lines.Count = 0;
        }

        Simulation simulation;
        private void ExecuteJob(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            var typeBatch = simulation.Solver.Batches[job.BatchIndex].TypeBatches[job.TypeBatchIndex];
            Debug.Assert(lineExtractors[typeBatch.TypeId] != null, "Jobs should only be created for types which are available and active.");
            //Creating a local copy of the list reference and count allows additions to proceed in parallel. 
            var jobLineList = new QuickList<LineInstance, Array<LineInstance>>(ref lines.Span);
            //By setting the count, we work around the fact that Array<T> doesn't support slicing.
            jobLineList.Count = job.LineStart;
            lineExtractors[typeBatch.TypeId].ExtractLines(simulation, typeBatch, job.ConstraintStart, job.ConstraintCount, ref jobLineList);
        }

        public void AddInstances(Simulation simulation)
        {
            int neededLineCapacity = 0;
            jobs.Count = 0;
            var jobPool = new PassthroughArrayPool<ThreadJob>();
            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    var extractor = lineExtractors[typeBatch.TypeId];
                    if (extractor != null)
                    {
                        jobs.Add(new ThreadJob
                        {
                            BatchIndex = batchIndex,
                            TypeBatchIndex = typeBatchIndex,
                            ConstraintStart = 0,
                            ConstraintCount = typeBatch.ConstraintCount,
                            LineStart = neededLineCapacity,
                            LinesPerConstraint = extractor.LinesPerConstraint
                        }, jobPool);
                        neededLineCapacity += extractor.LinesPerConstraint * typeBatch.ConstraintCount;
                    }
                }
            }
            var maximumJobSize = Math.Max(1, neededLineCapacity / (jobsPerThread * Environment.ProcessorCount));
            var originalJobCount = jobs.Count;
            //Split jobs if they're larger than desired to help load balancing a little bit. This isn't terribly important, but it's pretty easy.
            for (int i = 0; i < originalJobCount; ++i)
            {
                ref var job = ref jobs[i];
                if (job.ConstraintCount > maximumJobSize)
                {
                    var subjobCount = (int)Math.Round(0.5 + job.ConstraintCount / (double)maximumJobSize);
                    var constraintsPerSubjob = job.ConstraintCount / subjobCount;
                    var remainder = job.ConstraintCount - constraintsPerSubjob * subjobCount;
                    //Modify the first job in place.
                    job.ConstraintCount = constraintsPerSubjob;
                    if (remainder > 0)
                        ++job.ConstraintCount;
                    //Append the remaining jobs.
                    var previousJob = job;
                    for (int j = 1; j < subjobCount; ++j)
                    {
                        var newJob = previousJob;
                        newJob.LineStart += previousJob.ConstraintCount * newJob.LinesPerConstraint;
                        newJob.ConstraintStart += previousJob.ConstraintCount;
                        newJob.ConstraintCount = constraintsPerSubjob;
                        if (remainder > j)
                            ++newJob.ConstraintCount;
                        jobs.Add(newJob, jobPool);
                        previousJob = newJob;
                    }
                }
            }
            lines.EnsureCapacity(neededLineCapacity, new PassthroughArrayPool<LineInstance>());
            lines.Count = neededLineCapacity; //Line additions will be performed on suballocated lists. This count will be used by the renderer when reading line data.
            this.simulation = simulation;
            Parallel.For(0, jobs.Count, ExecuteJob);
            this.simulation = null;
        }

    }
}
