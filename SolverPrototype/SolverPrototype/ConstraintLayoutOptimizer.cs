﻿using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

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

        Optimization nextTargetWithoutOffset;
        Optimization nextTargetWithOffset;

        //Note that we could easily use a stackalloc to handle the targets, but it would actually cost more- stackallocs force a zeroing (as of this writing).
        //Basically irrelevant in terms of total cost, though- this is just mainly to make use of the QuickList's functionality.
        //Might also be nice to have it stored on the heap when we later use multithreaded optimizations.
        QuickList<Optimization> targets = new QuickList<Optimization>(new PassthroughBufferPool<Optimization>());

        /// <summary>
        /// If true, regions are offset by a half region width. Toggled each frame. Offsets allow the sorted regions to intermix, eventually converging to a full sort.
        /// </summary>
        bool shouldOffset;

        public ConstraintLayoutOptimizer(Bodies bodies, Solver solver)
        {
            this.bodies = bodies;
            this.solver = solver;
        }

        public void Update(int maximumRegionSizeInBundles, int regionCount)
        {
            //Note that we require that all regions are bundle aligned. This is important for the typebatch sorting process, which tends to use bulk copies from bundle arrays to cache.
            //If not bundle aligned, those bulk copies would become complex due to the constraint AOSOA layout.
            Debug.Assert((maximumRegionSizeInBundles & 1) == 0, "Region size in bundles should be divisible by two to allow offsets.");
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            if (solver.Batches.Count == 0)
                return;
            targets.Count = 0;


            var nextTarget = shouldOffset ? nextTargetWithOffset : nextTargetWithoutOffset;

            int GetStartBundleIndex(int batchIndex, int typeBatchIndex)
            {
                return shouldOffset ? Math.Min(maximumRegionSizeInBundles / 2,
                    Math.Max(0, solver.Batches[batchIndex].TypeBatches[typeBatchIndex].BundleCount - maximumRegionSizeInBundles)) : 0;
            }
            bool WrapBatch(ref Optimization o)
            {
                Debug.Assert(solver.Batches.Count >= 0);
                if (o.BatchIndex >= solver.Batches.Count)
                {
                    //Wrap around.
                    o = new Optimization();
                    o.BundleIndex = GetStartBundleIndex(0, 0);
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
                        o.BundleIndex = GetStartBundleIndex(o.BatchIndex, 0);
                    }
                    return true;
                }
                return false;
            }
            void WrapBundle(ref Optimization o)
            {
                Debug.Assert(o.BatchIndex <= solver.Batches.Count && o.TypeBatchIndex <= solver.Batches[o.BatchIndex].TypeBatches.Count,
                    "Should only attempt to wrap constraint index if the type batch and batch indices are known to be valid.");
                if (o.BundleIndex >= solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount)
                {
                    ++o.TypeBatchIndex;
                    if (!WrapTypeBatch(ref o))
                    {
                        o.BundleIndex = GetStartBundleIndex(o.BatchIndex, o.TypeBatchIndex);
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

            //Note that we have two separate parallel optimizations over multiple frames. Alternating between them on a per frame basis is a fairly simple way to guarantee
            //eventual convergence in the sort.
            //If conditionally assignable ref locals existed, this reassignment could go away. You might be able to do some hack to get around it, but it's superdupernotworthit.
            if (shouldOffset)
            {
                nextTargetWithOffset = nextTarget;
            }
            else
            {
                nextTargetWithoutOffset = nextTarget;
            }
            shouldOffset = !shouldOffset;


            var maximumRegionSizeInConstraints = maximumRegionSizeInBundles * Vector<int>.Count;
            //Now that we have a set of scheduled optimizations, execute them. 
            //Note that having the set of optimizations computed up front makes multithreading the loop trivial.
            for (int i = 0; i < targets.Count; ++i)
            {
                ref var target = ref targets.Elements[i];
                var typeBatch = solver.Batches[target.BatchIndex].TypeBatches.Elements[target.TypeBatchIndex];
                typeBatch.SortByBodyLocation(target.BundleIndex, Math.Min(typeBatch.ConstraintCount - target.BundleIndex * Vector<int>.Count, maximumRegionSizeInConstraints), solver.HandlesToConstraints, bodies.BodyCount);
            }
        }
    }
}