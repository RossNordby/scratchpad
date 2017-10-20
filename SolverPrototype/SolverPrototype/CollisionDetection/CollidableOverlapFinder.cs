﻿using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

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
        struct SelfOverlapHandler : IOverlapHandler
        {
            public NarrowPhase<TCallbacks> NarrowPhase;
            public Buffer<CollidableReference> Leaves;
            public int WorkerIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public SelfOverlapHandler(Buffer<CollidableReference> leaves, NarrowPhase<TCallbacks> narrowPhase, int workerIndex)
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
        struct IntertreeOverlapHandler : IOverlapHandler
        {
            public NarrowPhase<TCallbacks> NarrowPhase;
            public Buffer<CollidableReference> LeavesA;
            public Buffer<CollidableReference> LeavesB;
            public int WorkerIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public IntertreeOverlapHandler(Buffer<CollidableReference> leavesA, Buffer<CollidableReference> leavesB, NarrowPhase<TCallbacks> narrowPhase, int workerIndex)
            {
                LeavesA = leavesA;
                LeavesB = leavesB;
                NarrowPhase = narrowPhase;
                WorkerIndex = workerIndex;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                NarrowPhase.HandleOverlap(WorkerIndex, LeavesA[indexA], LeavesB[indexB]);
            }
        }
        Tree.MultithreadedSelfTest<SelfOverlapHandler> selfTestContext;
        Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler> intertreeTestContext;
        NarrowPhase<TCallbacks> narrowPhase;
        BroadPhase broadPhase;
        SelfOverlapHandler[] selfHandlers;
        IntertreeOverlapHandler[] intertreeHandlers;
        Action<int> workerAction;
        int nextJobIndex;
        public CollidableOverlapFinder(NarrowPhase<TCallbacks> narrowPhase, BroadPhase broadPhase)
        {
            selfTestContext = new Tree.MultithreadedSelfTest<SelfOverlapHandler>(narrowPhase.Pool);
            intertreeTestContext = new Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler>(narrowPhase.Pool);
            this.narrowPhase = narrowPhase;
            this.broadPhase = broadPhase;
            workerAction = Worker;
        }

        void Worker(int workerIndex)
        {
            Debug.Assert(workerIndex >= 0 && workerIndex < intertreeHandlers.Length && workerIndex < selfHandlers.Length);

            var totalJobCount = selfTestContext.JobCount + intertreeTestContext.JobCount;
            while (true)
            {
                var jobIndex = Interlocked.Increment(ref nextJobIndex);
                if (jobIndex < selfTestContext.JobCount)
                {
                    //This is a self test job.
                    selfTestContext.ExecuteJob(jobIndex, workerIndex);
                }
                else if (jobIndex < totalJobCount)
                {
                    //This is an intertree test job.
                    intertreeTestContext.ExecuteJob(jobIndex - selfTestContext.JobCount, workerIndex);
                }
                else
                {
                    //No more jobs remain;
                    break;
                }
            }
            ref var worker = ref narrowPhase.overlapWorkers[workerIndex];
            worker.Batcher.Flush(ref worker.ConstraintGenerators, ref worker.Filters);
        }

        public override void DispatchOverlaps(IThreadDispatcher threadDispatcher = null)
        {
            narrowPhase.Prepare(threadDispatcher);
            if (threadDispatcher != null)
            {
                if (intertreeHandlers == null || intertreeHandlers.Length < threadDispatcher.ThreadCount)
                {
                    //This initialization/resize should occur extremely rarely.
                    selfHandlers = new SelfOverlapHandler[threadDispatcher.ThreadCount];
                    intertreeHandlers = new IntertreeOverlapHandler[threadDispatcher.ThreadCount];
                    for (int i = 0; i < intertreeHandlers.Length; ++i)
                    {
                        selfHandlers[i] = new SelfOverlapHandler(broadPhase.activeLeaves, narrowPhase, i);
                    }
                    for (int i = 0; i < intertreeHandlers.Length; ++i)
                    {
                        intertreeHandlers[i] = new IntertreeOverlapHandler(broadPhase.activeLeaves, broadPhase.staticLeaves, narrowPhase, i);
                    }
                }
                Debug.Assert(intertreeHandlers.Length >= threadDispatcher.ThreadCount);
                selfTestContext.PrepareJobs(broadPhase.ActiveTree, selfHandlers, threadDispatcher.ThreadCount);
                intertreeTestContext.PrepareJobs(broadPhase.ActiveTree, broadPhase.StaticTree, intertreeHandlers, threadDispatcher.ThreadCount);
                nextJobIndex = -1;
                //threadDispatcher.DispatchWorkers(workerAction);
                for (int i = 0; i < selfTestContext.JobCount; ++i)
                {
                    selfTestContext.ExecuteJob(i, 0);
                }
                for (int i = 0; i < intertreeTestContext.JobCount; ++i)
                {
                    intertreeTestContext.ExecuteJob(i, 0);
                }
                selfTestContext.CompleteSelfTest();
                intertreeTestContext.CompleteTest();
            }
            else
            {
                var selfTestHandler = new SelfOverlapHandler(broadPhase.activeLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetSelfOverlaps(ref selfTestHandler);
                var intertreeHandler =  new IntertreeOverlapHandler(broadPhase.activeLeaves, broadPhase.staticLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetOverlaps(broadPhase.StaticTree, ref intertreeHandler);
                ref var worker = ref narrowPhase.overlapWorkers[0];
                worker.Batcher.Flush(ref worker.ConstraintGenerators, ref worker.Filters);

            }

        }

    }
}

