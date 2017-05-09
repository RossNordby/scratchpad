using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace SolverPrototype
{
    public class ConstraintLayoutOptimizer2
    {
        Bodies bodies;
        ConstraintConnectivityGraph graph;
        Solver solver;
        struct Optimization
        {
            /// <summary>
            /// Index of the target constraint to optimize.
            /// </summary>
            public int ConstraintIndex;
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

        public ConstraintLayoutOptimizer2(Bodies bodies, ConstraintConnectivityGraph graph, Solver solver)
        {
            this.bodies = bodies;
            this.graph = graph;
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
                    o.ConstraintIndex = 0;
                }
                return true;
            }
            return false;
        }

        bool WrapConstraint(ref Optimization o)
        {
            Debug.Assert(o.BatchIndex <= solver.Batches.Count && o.TypeBatchIndex <= solver.Batches[o.BatchIndex].TypeBatches.Count,
                "Should only attempt to wrap constraint index if the type batch and batch indices are known to be valid.");
            //Note that we only bother considering any constraint which is far enough from the end of the current type batch to permit a swap.
            //If the traversal origin is at count - 2, then there's only one slot following it- nothing could swap with it.
            //(This could change if we decide to permit sorting- then the last and second to last indices could swap. Unlikely to be useful, though.)
            if (o.ConstraintIndex >= solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].ConstraintCount - 2)
            {
                ++o.TypeBatchIndex;
                if (!WrapTypeBatch(ref o))
                {
                    o.ConstraintIndex = 0;
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
                    WrapConstraint(ref o);
                }
            }
        }

        struct Worker
        {
            /// <summary>
            /// The set of constraints which have already been enqueued to the worker local traversal stack at some point.
            /// </summary>
            public QuickSet<int, Buffer<int>, Buffer<int>, PrimitiveComparer<int>> AlreadyTraversedConstraintHandles;
            /// <summary>
            /// List used as the stack of constraints to visit.
            /// </summary>
            public QuickList<int, Buffer<int>> TraversalStack;
        }

        struct BodyEnumerator : IForEach<int>
        {
            public Worker Worker;
            public ConstraintConnectivityGraph Graph;
            public Solver Solver;
            public void LoopBody(int bodyIndex)
            {
                //TODO: May want to see if an extra precondition here is a net win.
                ref var constraints = ref Graph.GetConstraintList(bodyIndex);
                for (int i = 0; i < constraints.Count; ++i)
                {
                    if (Worker.AlreadyTraversedConstraintHandles.Count == Worker.AlreadyTraversedConstraintHandles.Span.Length)
                        break;
                    //More constraints can fit.
                    var constraint = constraints[i];
                    //Note that we do not filter by the type batch or the position of the constraint in its type batch here- 
                    //traversal needs to traverse any batch's constraints, otherwise we would never find any other constraints within our type batch.
                    //(Recall that constraint batches contain no constraints that share bodies!)
                    if (!Worker.AlreadyTraversedConstraintHandles.AddUnsafely(constraint.ConnectingConstraintHandle))
                    {
                        //Already added this constraint handle. Can't revisit it.
                        continue;
                    }
                    //We haven't already visited this constraint and it can fit in the traversal stack.
                    //Push it onto the stack.
                    Worker.TraversalStack.AddUnsafely(constraint.ConnectingConstraintHandle);


                }
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Optimize(ref Optimization target, ref Worker worker)
        {
            var typeBatch = solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex];
            Debug.Assert(worker.TraversalStack.Count == 0 && worker.AlreadyTraversedConstraintHandles.Count == 0, "The original worker reference counts should be zero.");
            //Initial add is guaranteed to succeed.
            BodyEnumerator bodyEnumerator;
            bodyEnumerator.Worker = worker;
            bodyEnumerator.Graph = graph;
            bodyEnumerator.Solver = solver;
            bodyEnumerator.Worker.AlreadyTraversedConstraintHandles.AddUnsafely(typeBatch.IndexToHandle[target.ConstraintIndex]);
            typeBatch.EnumerateConnectedBodyIndices(target.ConstraintIndex, ref bodyEnumerator);

            int moveSlotIndex = target.ConstraintIndex + 1;


            //Note that the loop condition alone would permit the search of an entire island.
            //It does not because of the limitation on adding new elements to the traversal stack- it stops once the already traversed set is full,
            //which is also when the traversal stack would cap out.
            while (bodyEnumerator.Worker.TraversalStack.Count > 0)
            {
                bodyEnumerator.Worker.TraversalStack.Pop(out var constraintToVisit);
                //Note that this must be a ref local; the later enumeration over the constraint's bodies uses it, but that enumeration occurs after a potential swap.
                ref var handleToConstraint = ref solver.HandlesToConstraints[constraintToVisit];
                Debug.Assert(
                    handleToConstraint.BatchIndex != target.BatchIndex ||
                    handleToConstraint.TypeId != typeBatch.TypeId || 
                    handleToConstraint.IndexInTypeBatch != target.ConstraintIndex,
                    "It should not be possible for the original constraint to appear on the traversal stack; " +
                    "its bodies were directly enumerated earlier, and its handle was added to the already-traversed set.");

                
                if (handleToConstraint.BatchIndex == target.BatchIndex && handleToConstraint.TypeId == typeBatch.TypeId &&
                    handleToConstraint.IndexInTypeBatch > moveSlotIndex)
                {
                    //This constraint is in the same type batch and is a candidate for pulling.
                    //Note that our traversal stack is based on constraint handles, so we don't have to worry about the possibility of invalidating later 
                    //entries in the traversal stack.
                    //Swap the constraint in the moveSlotIndex with the discovered constraint.
                    //Note that, when multithreading, we do not need any form of extra lock for thes olver.HandlesToConstraints, even though this swap will mutate them.
                    //That's because we will have already claimed the constraints, and the only handle-constraint mappings that will change are those which have been claimed.
                    //No race conditions are possible.
                    typeBatch.SwapConstraints(handleToConstraint.IndexInTypeBatch, moveSlotIndex++, solver.HandlesToConstraints);
                }
                //Note that we use the handleToConstraint here. It must be a ref local, because any constraint swap triggered above could change the index.
                typeBatch.EnumerateConnectedBodyIndices(handleToConstraint.IndexInTypeBatch, ref bodyEnumerator);

            }
            //Clear the worker sets. Note that we don't reassign the worker to the original worker reference. Contents of the buffers other than the table don't matter.
            bodyEnumerator.Worker.AlreadyTraversedConstraintHandles.FastClear();
            Debug.Assert(worker.TraversalStack.Count == 0 && worker.AlreadyTraversedConstraintHandles.Count == 0, "The original worker reference counts should be zero and unchanged.");
        }

        public void Update(int constraintsToOptimize, BufferPool rawPool)
        {
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            if (solver.Batches.Count == 0)
                return;

            //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
            BoundsCheckOldTarget(ref nextTarget);

            Worker worker;
            const int traversalCapacity = 64;
            QuickList<int, Buffer<int>>.Create(rawPool.SpecializeFor<int>(), 64, out worker.TraversalStack);
            QuickSet<int, Buffer<int>, Buffer<int>, PrimitiveComparer<int>>.Create(rawPool.SpecializeFor<int>(), rawPool.SpecializeFor<int>(),
                SpanHelper.GetContainingPowerOf2(traversalCapacity), 3, out worker.AlreadyTraversedConstraintHandles);

            for (int i = 0; i < constraintsToOptimize; ++i)
            {
                Optimize(ref nextTarget, ref worker);
                ++nextTarget.ConstraintIndex;
                WrapConstraint(ref nextTarget);
            }

            worker.AlreadyTraversedConstraintHandles.Dispose(rawPool.SpecializeFor<int>(), rawPool.SpecializeFor<int>());
            worker.TraversalStack.Dispose(rawPool.SpecializeFor<int>());
        }

    }
}
