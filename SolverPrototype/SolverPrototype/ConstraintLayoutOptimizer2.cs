using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace SolverPrototype
{
    public class ConstraintLayoutOptimizer
    {
        Bodies bodies;
        Solver solver;
        struct Optimization
        {
            /// <summary>
            /// Index of the target constraint bundle to optimize.
            /// </summary>
            public int BundleIndex;
            /// <summary>
            /// Index of the last optimized type batch.
            /// </summary>
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the last optimized batch.
            /// </summary>
            public int BatchIndex;

        }

        Optimization nextTarget;

        /// <summary>
        /// If true, regions are offset by a half region width. Toggled each frame. Offsets allow the sorted regions to intermix, eventually converging to a full sort.
        /// </summary>
        bool shouldOffset;

        public ConstraintLayoutOptimizer(Bodies bodies, Solver solver)
        {
            this.bodies = bodies;
            this.solver = solver;
        }

        bool WrapBatch(ref Optimization o)
        {
            Debug.Assert(solver.Batches.Count >= 0);
            if (o.BatchIndex >= solver.Batches.Count)
            {
                //Wrap around.
                o = new Optimization();
                return true;
            }
            return false;
        }
        bool WrapTypeBatch(ref Optimization o)
        {
            Debug.Assert(o.BatchIndex <= solver.Batches.Count, "Should only attempt to wrap type batch indices if the batch index is known to be valid.");
            if (o.TypeBatchIndex >= solver.Batches[o.BatchIndex].TypeBatches.Count)
            {
                ++o.BatchIndex;
                if (!WrapBatch(ref o))
                {
                    o.TypeBatchIndex = 0;
                    o.BundleIndex = 0;
                }
                return true;
            }
            return false;
        }

        bool WrapBundle(ref Optimization o)
        {
            Debug.Assert(o.BatchIndex <= solver.Batches.Count && o.TypeBatchIndex <= solver.Batches[o.BatchIndex].TypeBatches.Count,
                "Should only attempt to wrap constraint index if the type batch and batch indices are known to be valid.");
            if (o.BundleIndex >= solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount)
            {
                ++o.TypeBatchIndex;
                if (!WrapTypeBatch(ref o))
                {
                    o.BundleIndex = 0;
                }
                return true;
            }
            return false;
        }
        void BoundsCheckOldTarget(ref Optimization o)
        {
            if (!WrapBatch(ref o))
            {
                if (!WrapTypeBatch(ref o))
                {
                    WrapBundle(ref o);
                }
            }
        }
        Optimization FindOffsetFrameStart(Optimization o, int maximumRegionSizeInBundles)
        {
            BoundsCheckOldTarget(ref o);

            var typeBatch = solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex];
            var spaceRemaining = typeBatch.BundleCount - o.BundleIndex;
            if (spaceRemaining <= maximumRegionSizeInBundles)
            {
                ++o.TypeBatchIndex;
                WrapTypeBatch(ref o);
            }
            o.BundleIndex = Math.Max(0, Math.Min(o.BundleIndex + maximumRegionSizeInBundles / 2, typeBatch.BundleCount - maximumRegionSizeInBundles));
            
            return o;
        }


        public void Update(int maximumRegionSizeInBundles, int throwaway, BufferPool rawPool, IThreadDispatcher threadDispatcher = null)
        {
            //Note that we require that all regions are bundle aligned. This is important for the typebatch sorting process, which tends to use bulk copies from bundle arrays to cache.
            //If not bundle aligned, those bulk copies would become complex due to the constraint AOSOA layout.
            Debug.Assert((maximumRegionSizeInBundles & 1) == 0, "Region size in bundles should be divisible by two to allow offsets.");
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            if (solver.Batches.Count == 0)
                return;

            Optimization target;
            if (shouldOffset)
            {
                //Use the previous frame's start to create the new target.
                target = FindOffsetFrameStart(nextTarget, maximumRegionSizeInBundles);
                Debug.Assert(solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex].BundleCount <= maximumRegionSizeInBundles || target.BundleIndex != 0,
                    "On offset frames, the only time a target bundle can be 0 is if the batch is too small for it to be anything else.");
                //Console.WriteLine($"Offset frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
            }
            else
            {
                //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
                target = nextTarget;
                BoundsCheckOldTarget(ref target);
                nextTarget = target;
                nextTarget.BundleIndex += maximumRegionSizeInBundles;
                //Console.WriteLine($"Normal frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
            }
            //Note that we have two separate parallel optimizations over multiple frames. Alternating between them on a per frame basis is a fairly simple way to guarantee
            //eventual convergence in the sort. We only ever push forward the non-offset version; the offset position is based on the nonoffset version's last start position.
            shouldOffset = !shouldOffset;


            var maximumRegionSizeInConstraints = maximumRegionSizeInBundles * Vector<int>.Count;

            var typeBatch = solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex];
            typeBatch.SortByBodyLocation(target.BundleIndex, Math.Min(typeBatch.ConstraintCount - target.BundleIndex * Vector<int>.Count, maximumRegionSizeInConstraints),
                solver.HandlesToConstraints, bodies.BodyCount, rawPool, threadDispatcher);

        }
    }
}
