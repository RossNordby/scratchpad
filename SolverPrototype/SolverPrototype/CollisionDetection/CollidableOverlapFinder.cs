using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    public abstract class CollidableOverlapFinder
    {
        public abstract void DispatchOverlaps(IThreadDispatcher threadDispatcher = null);
    }

    //The overlap finder requires type knowledge about the narrow phase that the broad phase lacks. Don't really want to infect the broad phase with a bunch of narrow phase dependent 
    //generic parameters, so instead we just explicitly create a type-aware overlap finder to help the broad phase.
    public class CollidableOverlapFinder<TCallbacks> : CollidableOverlapFinder where TCallbacks : struct, INarrowPhaseCallbacks
    {
        struct OverlapHandler : IOverlapHandler
        {
            //TODO: Once we have a second tree, we'll need to have a second handler type which can pull from two different leaf sources.
            //No reason to try to make one type do both- that would just result in a bunch of last second branches that could be avoided by using the proper
            //handler upon dispatch. 
            public NarrowPhase<TCallbacks> NarrowPhase;
            public Buffer<CollidableReference> Leaves;
            public int WorkerIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public OverlapHandler(Buffer<CollidableReference> leaves, NarrowPhase<TCallbacks> narrowPhase, int workerIndex)
            {
                Leaves = leaves;
                NarrowPhase = narrowPhase;
                WorkerIndex = workerIndex;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                NarrowPhase.HandleOverlap(WorkerIndex, Leaves[indexA], Leaves[indexB]);
            }
        }
        Tree.MultithreadedSelfTest<OverlapHandler> selfTestContext;
        NarrowPhase<TCallbacks> narrowPhase;
        BroadPhase broadPhase;
        OverlapHandler[] threadHandlers;
        Action<int> workerAction;
        public CollidableOverlapFinder(NarrowPhase<TCallbacks> narrowPhase, BroadPhase broadPhase)
        {
            selfTestContext = new Tree.MultithreadedSelfTest<OverlapHandler>();
            this.narrowPhase = narrowPhase;
            this.broadPhase = broadPhase;
            workerAction = Worker;
        }

        void Worker(int workerIndex)
        {
            selfTestContext.PairTest(workerIndex);
            ref var worker = ref narrowPhase.overlapWorkers[workerIndex];
            worker.Batcher.Flush(ref worker.ConstraintGenerators, ref worker.Filters);
        }

        public override void DispatchOverlaps(IThreadDispatcher threadDispatcher = null)
        {
            narrowPhase.Prepare(threadDispatcher);
            if (threadDispatcher != null)
            {
                if (threadHandlers == null || threadHandlers.Length < threadDispatcher.ThreadCount)
                {
                    //This initialization/resize should occur extremely rarely.
                    threadHandlers = new OverlapHandler[threadDispatcher.ThreadCount];
                    for (int i = 0; i < threadHandlers.Length; ++i)
                    {
                        threadHandlers[i] = new OverlapHandler(broadPhase.activeLeaves, narrowPhase, i);
                    }
                }
                Debug.Assert(threadHandlers.Length >= threadDispatcher.ThreadCount);
                selfTestContext.PrepareSelfTestJobs(broadPhase.ActiveTree, threadHandlers, threadDispatcher.ThreadCount);
                threadDispatcher.DispatchWorkers(workerAction);
                selfTestContext.CompleteSelfTest(broadPhase.ActiveTree);
            }
            else
            {
                var overlapHandler = new OverlapHandler { NarrowPhase = narrowPhase, Leaves = broadPhase.activeLeaves, WorkerIndex = 0 };
                broadPhase.ActiveTree.GetSelfOverlaps(ref overlapHandler);
                ref var worker = ref narrowPhase.overlapWorkers[0];
                worker.Batcher.Flush(ref worker.ConstraintGenerators, ref worker.Filters);

            }

        }

    }
}
