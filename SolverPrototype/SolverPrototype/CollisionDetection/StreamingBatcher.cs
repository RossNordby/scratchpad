using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{
    public interface IContinuations
    {
        void Notify(TypedIndex continuationId);
    }

    /// <summary>
    /// Defines filters that some collision tasks may call when child tasks need to be spawned.
    /// </summary>
    public interface ICollisionSubtaskFilters
    {
        /// <summary>
        /// Checks whether further collision testing should be performed for a given subtask.
        /// </summary>
        /// <typeparam name="TInput">Type of the input used to decide whether to allow contact generation.</typeparam>
        /// <param name="input">Input data to be used to decide whether to allow testing.</param>
        /// <returns>True if testing should proceed, false otherwise.</returns>
        bool AllowCollisionTesting<TInput>(ref TInput input) where TInput : struct;
        /// <summary>
        /// Provides control over subtask generated results before they are reported to the parent task.
        /// </summary>
        /// <typeparam name="TInput">Type of the input used for configuration.</typeparam>
        /// <param name="input">Input used by configuration.</param>
        void Configure<TInput>(ref TInput input) where TInput : struct;
    }

    public abstract class CollisionTask
    {
        /// <summary>
        /// Gets the number of tasks to batch together before executing this task.
        /// </summary>
        public int BatchSize { get; protected set; }
        /// <summary>
        /// Gets the first shape type index associated with the task. Shape pairs provided to the task for execution should be in the order defined by these type two indices.
        /// If a collision task isn't a top level shape pair task, this should be -1.
        /// </summary>
        public int ShapeTypeIndexA { get; protected set; }
        /// <summary>
        /// Gets the second shape type index associated with the task. Shape pairs provided to the task for execution should be in the order defined by these type two indices.
        /// If a collision task isn't a top level shape pair task, this should be -1.
        /// </summary>
        public int ShapeTypeIndexB { get; protected set; }
        /// <summary>
        /// Gets the type ids of the specialized subtasks registered by this task.
        /// </summary>
        public int[] SubtaskIndices { get; protected set; }
        /// <summary>
        /// Gets the set of collision tasks that this task may produce as a part of execution.
        /// </summary>
        public CollisionTask[] Subtasks { get; protected set; }

        //Note that we leave the details of input and output of a task's execution to be undefined.
        //A task can reach into the batcher and create new entries or trigger continuations as required.
        /// <summary>
        /// Executes the task on the given input.
        /// </summary>
        /// <typeparam name="TFilters">Type of the filters used to influence execution of collision tasks.</typeparam>
        /// <typeparam name="TContinuations">Type of the continuations that can be triggered by the this execution.</typeparam>
        /// <param name="batcher">Batcher responsible for the invocation.</param>
        /// <param name="batch">Batch of pairs to test.</param>
        /// <param name="continuations">Continuations to invoke upon completion of a top level pair.</param>
        /// <param name="filters">Filters to use to influence execution of the collision tasks.</param>
        public abstract void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters;

    }

    struct TopLevelIndices
    {
        public int TaskIndex;
    }

    public class CollisionTaskRegistry
    {
        int[][] topLevelMatrix;
        internal CollisionTask[] tasks;
        int count;

        public CollisionTaskRegistry(int initialShapeCount = 9)
        {
            ResizeMatrix(initialShapeCount);
        }

        void ResizeMatrix(int newSize)
        {
            Array.Resize(ref topLevelMatrix, newSize);
            for (int i = 0; i < newSize; ++i)
            {
                Array.Resize(ref topLevelMatrix[i], newSize);
            }
        }

        void InsertTask(CollisionTask task, int index)
        {
            //This allocates a lot of garbage due to frequently resizing, but it does not matter- task registration a one time thing at program initialization.
            //Having tight bounds is more useful for performance in the end (by virtue of having a marginally simpler heap).
            int newCount = count + 1;
            if (newCount > tasks.Length)
                Array.Resize(ref tasks, newCount);
            if (index < count)
            {
                Array.Copy(tasks, index, tasks, index + 1, count - index);
                //Every task in the array that got moved needs to have its top level index bumped. 
                for (int i = 0; i < topLevelMatrix.Length; ++i)
                {
                    for (int j = 0; j < topLevelMatrix.Length; ++j)
                    {
                        if (topLevelMatrix[i][j] >= index)
                        {
                            ++topLevelMatrix[i][j];
                        }
                    }
                }
                for (int i = index + 1; i < newCount; ++i)
                {
                    var t = tasks[i];
                    for (int j = 0; j < t.SubtaskIndices.Length; ++j)
                    {
                        ref var subtaskIndex = ref t.SubtaskIndices[j];
                        if (subtaskIndex >= index)
                        {
                            ++subtaskIndex;
                        }
                    }
                }
            }

            tasks[index] = task;
            count = newCount;
        }


        public int Register(CollisionTask task)
        {
            var index = count;
            //This task may have some dependencies that are already present. In order for the batcher's flush to work with a single pass,
            //the tasks must be stored in dependency order- any task that can create more subwork has to appear earlier in the list than the subwork's task.
            //Where is the earliest one?
            for (int i = 0; i < task.Subtasks.Length; ++i)
            {
                var subtaskIndex = Array.IndexOf(tasks, task.Subtasks[i], 0, count);
                if (subtaskIndex >= 0 && subtaskIndex < index)
                    index = subtaskIndex;
            }

            InsertTask(task, index);
            var a = task.ShapeTypeIndexA;
            var b = task.ShapeTypeIndexB;
            var highestShapeIndex = a > b ? a : b;
            if (highestShapeIndex >= 0)
            {
                //This only handles top level pairs (those associated with shape type pairs).
                //Some tasks are not directly associated with a top level entrypoint- instead, they're follow ups on top level tasks. Since they're not an entry point,
                //there is no need for them to appear in the top level matrix.
                if (highestShapeIndex >= topLevelMatrix.Length)
                    ResizeMatrix(highestShapeIndex + 1);
                topLevelMatrix[a][b] = index;
                topLevelMatrix[b][a] = index;
            }

#if DEBUG
            //Ensure that no task dependency cycles exist.
            for (int i = 0; i < task.Subtasks.Length; ++i)
            {
                for (int j = i + 1; j < task.Subtasks.Length; ++j)
                {
                    Debug.Assert(Array.IndexOf(tasks[j].Subtasks, tasks[i]) == -1,
                        "Tasks must be stored in a strict order of work generation- if a task generates work for another task, the receiving task must appear later in the list. " +
                        "No cycles can exist.");
                }
            }
#endif
            //Register any unregistered subtasks.
            for (int i = 0; i < task.Subtasks.Length; ++i)
            {
                var subtaskIndex = Array.IndexOf(tasks, task.Subtasks[i], 0, count);
                if (subtaskIndex < 0)
                {
                    subtaskIndex = Register(task);
                }
                task.SubtaskIndices[i] = subtaskIndex;
            }
            Debug.Assert(tasks[index] == task, "No subtask registrations should move the original task; that would imply a cycle in the dependency graph.");
            return index;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetTask(int topLevelTypeA, int topLevelTypeB, out CollisionTask task, out int taskIndex)
        {
            taskIndex = topLevelMatrix[topLevelTypeA][topLevelTypeB];
            task = tasks[taskIndex];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetTask<TShapeA, TShapeB>(out CollisionTask task, out int taskIndex)
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            GetTask(TypeIds<IShape>.GetId<TShapeA>(), TypeIds<IShape>.GetId<TShapeB>(), out task, out taskIndex);
        }
    }

    struct RigidPair<TShapeA, TShapeB>
            where TShapeA : struct, IShape where TShapeB : struct, IShape
    {
        public TShapeA A;
        public TShapeB B;
        public BodyPose RelativePose;
        public TypedIndex Continuation;
    }


    public struct StreamingBatcher
    {
        //The streaming batcher contains batches for pending work submitted by the user.
        //This pending work can be top level pairs like sphere versus sphere, but it may also be subtasks of submitted work.
        //Consider two compound bodies colliding. The pair will decompose into a set of potentially many convex subpairs.
        //Similarly, a hull-hull collision test could spawn many subtasks, but those subtasks may not be of the same type as any top level pair.

        CollisionTaskRegistry typeMatrix;
        BufferPool pool;


        int minimumBatchIndex, maximumBatchIndex;
        Buffer<UntypedList> batches;
        //A subset of collision tasks require a place to return information.
        Buffer<UntypedList> localContinuations;


        public unsafe StreamingBatcher(BufferPool pool, CollisionTaskRegistry collisionTypeMatrix)
        {
            this.pool = pool;
            typeMatrix = collisionTypeMatrix;
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out batches);
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out localContinuations);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, collisionTypeMatrix.tasks.Length);
            localContinuations.Clear(0, collisionTypeMatrix.tasks.Length);
            minimumBatchIndex = collisionTypeMatrix.tasks.Length;
            maximumBatchIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add<TShapeA, TShapeB, TContinuations, TFilters>(CollisionTask task, int taskIndex,
            ref TShapeA shapeA, ref TShapeB shapeB, ref BodyPose relativePose,
            TypedIndex continuationId, ref TContinuations continuations, ref TFilters filters)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            ref var batch = ref batches[taskIndex];
            ref var pairData = ref batch.AllocateUnsafely<RigidPair<TShapeA, TShapeB>>();
            pairData.A = shapeA;
            pairData.B = shapeB;
            pairData.RelativePose = relativePose;
            if (batch.Count == task.BatchSize)
            {
                task.ExecuteBatch(ref batch, ref this, ref continuations, ref filters);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add<TShapeA, TShapeB, TContinuations, TFilters>(ref TShapeA shapeA, ref TShapeB shapeB, ref BodyPose relativePose,
             TypedIndex continuationId, ref TContinuations continuations, ref TFilters filters)
             where TShapeA : struct, IShape where TShapeB : struct, IShape
             where TContinuations : struct, IContinuations
             where TFilters : struct, ICollisionSubtaskFilters
        {
            //TODO: It's possible that only retrieving the actual task in the event that it's time to dispatch could save some cycles.
            //That would imply caching the batch sizes and expected first ids, likely alongside the task indices.
            //Value of that is questionable, since all the task-associated indirections are pretty much guaranteed to be cached.
            typeMatrix.GetTask<TShapeA, TShapeB>(out var task, out var taskIndex);
            ref var batch = ref batches[taskIndex];
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(task.BatchSize * Unsafe.SizeOf<RigidPair<TShapeA, TShapeB>>(), pool);
                if (minimumBatchIndex > taskIndex)
                    minimumBatchIndex = taskIndex;
                if (maximumBatchIndex < taskIndex)
                    maximumBatchIndex = taskIndex;
            }
            //The type comparison should be a compilation constant.
            if (typeof(TShapeA) != typeof(TShapeB) && TypeIds<IShape>.GetId<TShapeA>() != task.ShapeTypeIndexA)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(task, taskIndex, ref shapeA, ref shapeB, ref relativePose, continuationId, ref continuations, ref filters);
            }
            else
            {
                Add(task, taskIndex, ref shapeA, ref shapeB, ref relativePose, continuationId, ref continuations, ref filters);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush<TContinuations, TFilters>(ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            //The collision task registry guarantees that tasks which create work for other tasks always appear sooner in the task array than their child tasks.
            //Since there are no cycles, only one flush pass is required.
            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
            {
                ref var batch = ref batches[i];
                if (batch.Count > 0)
                {
                    typeMatrix.tasks[i].ExecuteBatch(ref batch, ref this, ref continuations, ref filters);
                }
            }
            //TODO: dispose everything here too. 
        }
    }
}
