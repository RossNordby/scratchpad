using System;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Constraints;
using System.Runtime.CompilerServices;
using System.Numerics;
using Quaternion = BEPUutilities2.Quaternion;

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
        public abstract void ExtractLines(Simulation simulation, TypeBatch typeBatch, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }

    class TypeLineExtractor<T, TTypeBatch, TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses> : TypeLineExtractor
        where T : struct, IConstraintLineExtractor<TBodyReferences, TPrestep>
        where TTypeBatch : TypeBatch<TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses>
    {
        public override int LinesPerConstraint => default(T).LinesPerConstraint;
        public override void ExtractLines(Simulation simulation, TypeBatch typeBatch, ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            var batch = (TTypeBatch)typeBatch;
            ref var prestepStart = ref batch.PrestepData[0];
            ref var referencesStart = ref batch.BodyReferences[0];
            var extractor = default(T);
            for (int i = 0; i < batch.ConstraintCount; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                extractor.ExtractLines(ref prestepBundle, ref referencesBundle, innerIndex, simulation.Bodies, ref lines);
            }
        }
    }

    struct BallSocketLineExtractor : IConstraintLineExtractor<TwoBodyReferences, BallSocketPrestepData>
    {
        public int LinesPerConstraint => 3;

        public void ExtractLines(ref BallSocketPrestepData prestepBundle, ref TwoBodyReferences referencesBundle, int innerIndex,
            Bodies bodies, ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            var indexA = GatherScatter.Get(ref referencesBundle.IndexA, innerIndex);
            var indexB = GatherScatter.Get(ref referencesBundle.IndexB, innerIndex);
            bodies.GetPoseByIndex(indexA, out var poseA);
            bodies.GetPoseByIndex(indexB, out var poseB);
            Vector3Wide.GetLane(ref prestepBundle.LocalOffsetA, innerIndex, out var localOffsetA);
            Vector3Wide.GetLane(ref prestepBundle.LocalOffsetB, innerIndex, out var localOffsetB);
            Quaternion.Transform(ref localOffsetA, ref poseA.Orientation, out var worldOffsetA);
            Quaternion.Transform(ref localOffsetB, ref poseB.Orientation, out var worldOffsetB);
            var endA = poseA.Position + worldOffsetA;
            var endB = poseB.Position + worldOffsetB;
            var color = new Vector3(0.2f, 0.2f, 1f);
            var lineA = new LineInstance(ref poseA.Position, ref endA, ref color);
            var lineB = new LineInstance(ref poseB.Position, ref endB, ref color);
            lines.AddUnsafely(ref lineA);
            lines.AddUnsafely(ref lineB);
            var errorColor = new Vector3(1, 0, 0);
            var errorLine = new LineInstance(ref endA, ref endB, ref errorColor);
            lines.AddUnsafely(ref errorLine);
        }
    }


    public class LineExtractor
    {
        TypeLineExtractor[] lineExtractors;
        internal QuickList<LineInstance, Array<LineInstance>> lines;

        public LineExtractor(int initialLineCapacity = 8192)
        {
            QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), initialLineCapacity, out lines);
            lineExtractors = new TypeLineExtractor[ConstraintTypeIds.RegisteredTypeCount];
            lineExtractors[ConstraintTypeIds.GetId<BallSocketTypeBatch>()] =
                new TypeLineExtractor<BallSocketLineExtractor, BallSocketTypeBatch, TwoBodyReferences, BallSocketPrestepData, BallSocketProjection, Vector3Wide>();
        }

        public void ClearInstances()
        {
            lines.Count = 0;
        }

        public void AddInstances(Simulation simulation)
        {
            int neededLineCapacity = 0;
            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    var extractor = lineExtractors[typeBatch.TypeId];
                    if (extractor != null)
                    {
                        neededLineCapacity += extractor.LinesPerConstraint * typeBatch.ConstraintCount;
                    }
                }
            }
            lines.EnsureCapacity(neededLineCapacity, new PassthroughArrayPool<LineInstance>());
            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    lineExtractors[typeBatch.TypeId]?.ExtractLines(simulation, typeBatch, ref lines);
                }
            }
        }
    }
}
