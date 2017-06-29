using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.Collidables
{
    public enum ContinuousDetectionMode : byte
    {
        //These modes use bit flags to enable specific features.
        //Bit 0: If set, allows velocity expansion beyond speculative margin. If unset, expansion is clamped to the margin.
        //Bit 1: If set, collision tests use an extra inner sphere contact generation test.
        //Bit 2: If set, collision tests use substepping. 
        //Not all combinations are valid. If substepping is enabled, bounding box velocity expansion must be enabled, otherwise the substepping won't do much.
        //Note that it's possible to have another mode- the bounding box expansion clamped to speculative margin, but then potentially expanded by an inner sphere swept by linear velocity.
        //The extra complexity doesn't really seem worth it, considering that the user would need to be aware of the unique corner cases involved. (Some collisions would be missed.)

        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This is the cheapest mode, but it may miss collisions. Note that if a Discrete mode collidable is moving quickly, the fact that its bounding box is not expanded
        /// may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// </summary>
        Discrete = 0b000,
        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will be expanded by velocity beyond the speculative margin if necessary.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
        /// that should avoid missing collisions.</para>
        /// </summary>
        Passive = 0b001,
        /// <summary>
        /// <para>In addition to the default speculative contact generation, a sphere embedded in the shape will be used as an additional speculative contact source when the 
        /// collidable is moving quickly enough relative to a collidable neighbor. The extra contact will only be used if room in the contact manifold is available.</para>
        /// <para>This is a very cheap form of continuous collision detection, and it tends to avoid ghost collisions better than simply increasing the speculative margin.
        /// On the other hand, the extra contact cannot capture angular motion, and it will tend to allow more penetration than substepping or a large speculative margin.</para>
        /// </summary>
        Linear = 0b011,
        /// <summary>
        /// <para>Collision detection will use multiple poses that span the time between frames. For any collidable neighbor, the earliest detected contact manifold is used.</para>
        /// <para>The number of substeps depends on the configured target step size and maximum step count. When moving slowly, substepping may be skipped entirely, and when moving quickly,
        /// many substeps may be used.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. Carefully choosing a target substep length and a speculative margin together can catch
        /// virtually all primary impacts in a very natural way.</para> 
        /// <para>Because it performs what amounts to multiple collision tests, this mode is more expensive for fast moving objects. Further, because part of its goal is to avoid ghost 
        /// collisions, it can miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        Substep = 0b101,
        /// <summary>
        /// <para>Uses both Linear and Substep modes together. This is the most expensive collision detection mode, but it has the benefits of both of the continuous modes.</para>
        /// <para>The inner sphere contact generation helps avoid tunneling through secondary collisions that the substepping explicitly filtered out, while substepping captures 
        /// the difficult angular motion. However, the inner sphere contact may reintroduce ghost collisions at extremely high velocities.</para>
        /// </summary>
        LinearAndSubstep = 0b111

        //TODO: Not really happy with these names. "Linear" does an okay job at describing the goal of the mode, but it really doesn't tell you much about what it's doing
        //or what corner cases to expect intuitively. And while something like "InnerSphere" would describe the underlying mechanism well, it doesn't provide much insight at a glance.
        //And "LinearWithSubstep" is just unwieldy. I decided against something like "Full" there because it isn't some universal per-event conservative advancement- it still has
        //definite holes and compromises. Even if it is the most complete of the bunch.
    }
    [StructLayout(LayoutKind.Explicit)]
    public struct ContinuousDetectionSettings
    {
        /// <summary>
        /// The continuous collision detection mode.
        /// </summary>
        [FieldOffset(0)]
        public ContinuousDetectionMode Mode;
        /// <summary>
        /// If using a substepping mode, this is the maximum number of substeps allowed. Fewer substeps than the maximum may be used for slower motion.
        /// </summary>
        [FieldOffset(1)]
        private byte MaximumSubstepCount;
        /// <summary>
        /// <para>The length of a motion path allowed before continuous collision detection will be used for this collidable.</para>
        /// <para>For modes using substepping, this is the maximum distance any part of the shape can move (due to linear or angular motion) before another substep is introduced.
        /// For modes using an inner sphere, this is the displacement in a single frame necessary to trigger the use of the extra inner sphere contact generator.</para>
        /// </summary>
        [FieldOffset(4)]
        public float MaximumStepLength;
        //Technically, there are situations where you would want to configure the step length for substepping and inner sphere separately. However, those are not terribly common,
        //and we really don't want to bloat the size of this structure- L1 cache isn't big.

        //Substepping CCD and AABB calculations also depend on the maximum radius, but that is handled by the Shape.
        //There isn't a strong reason to include per-collidable information for that.

        //Note that we disallow cases where expansion beyond the margin is off, yet continuous detection is on. So, there's no need for masking.
        internal bool AllowExpansionBeyondSpeculativeMargin { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (uint)Mode > 0; } }
        internal bool UseInnerSphere { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ((uint)Mode & 2) > 0; } }
        internal bool UseSubstepping { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ((uint)Mode & 4) > 0; } }
    }

    /// <summary>
    /// Collidable representing a body.
    /// </summary>
    /// <remarks>Body collidables are distinguished from non-body collidables by the need to gather pose and velocity from a body to update the bounding box.
    /// Isolated collidables can exist and use the same shapes as body collidables, but since their bounding boxes don't need to be updated every frame, they aren't included 
    /// in the body collidable batches (whose main purpose is to update bounding boxes efficiently).</remarks>
    public struct BodyCollidable
    {
        /// <summary>
        /// Index of the shape used by the body. The type is implicit; the body collidable batch holding this body collidable knows which type to use.
        /// </summary>
        public int ShapeIndex;
        /// <summary>
        /// Index of the body that owns this collidable.
        /// </summary>
        /// <remarks>This is needed for the narrow phase to know what bodies are involved in newly generated contact constraints. It will be updated whenever a body moves in memory.</remarks> 
        public int BodyIndex;
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters.
        /// </summary>
        public int BroadPhaseIndex;
        /// <summary>
        /// Size of the margin around the surface of the shape in which contacts can be generated. These contacts will have negative depth and only contribute if the frame's velocity
        /// would push the shape into overlap. This should be positive to avoid jittering. It can also be used as a form of continuous collision detection, but excessively high values
        /// combined with fast motion may result in visible 'ghost collision' artifacts. 
        /// <para>For continuous collision detection with less chance of ghost collisions, use the dedicated continuous collision detection modes.</para>
        /// </summary>
        public float SpeculativeMargin;

        /// <summary>
        /// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
        /// </summary>
        public ContinuousDetectionSettings Continuity;
        //These CCD settings are bundled together away from the rest of the collidable data for a few reasons:
        //1) They do a little packing to avoid pointless memory overhead,
        //2) It's possible that we'll want to split them out later if data access patterns suggest that it's a good idea,
        //3) Don't really want to pollute this structure's members with CCD-conditional tuning variables.
    }



    public abstract class BodyCollidableBatch
    {
        protected QuickList<BodyCollidable, Buffer<BodyCollidable>> collidables;

        public ref BodyCollidable this[int collidableIndex] { get { return ref collidables[collidableIndex]; } }

        protected BodyCollidableBatch(BufferPool pool, int initialCollidableCount)
        {
            QuickList<BodyCollidable, Buffer<BodyCollidable>>.Create(pool.SpecializeFor<BodyCollidable>(), initialCollidableCount, out collidables);

        }

        public int Allocate(BufferPool pool)
        {
            //Note that we do not actually fill the slot here. That's because of a dependency issue- we have to add the collidable's index to the broad phase to get a target index,
            //but we need a target index to build the data for the body collidable...
            int index = collidables.Count;
            var newCount = collidables.Count + 1;
            collidables.EnsureCapacity(newCount, pool.SpecializeFor<BodyCollidable>());
            collidables.Count = newCount;
            //The bodies set will store an index that points back here, so we need to report it.
            return index;
        }

        /// <summary>
        /// Removes a body collidable from the batch and which body owns the collidable that moved to fill its slot, if any. 
        /// </summary>
        /// <param name="index">Index to remove from the batch.</param>
        /// <param name="bodyHandleOfMovedCollidable">If the index was not the last slot in the collidables (and so this function returned true),
        /// this is the handle of the body owning the body collidable that was moved to fill its slot.
        /// If no collidable was moved (and so this function returned false), the value is undefined.</param>
        /// <returns>True if a body collidable was moved and reported, false otherwise.</returns>
        public bool RemoveAt(int index, out int bodyHandleOfMovedCollidable)
        {
            collidables.FastRemoveAt(index);
            if (index < collidables.Count)
            {
                bodyHandleOfMovedCollidable = collidables[index].BodyIndex;
                return true;
            }
            bodyHandleOfMovedCollidable = -1;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void GatherCollidableData(ref QuickList<int, Buffer<int>> bundledCollidables, int startIndex,
            out Vector<int> shapeIndices, out Vector<int> bodyIndices, out Vector<float> maximumExpansion)
        {
            ref var firstCollidable = ref bundledCollidables[startIndex];
            Debug.Assert(bundledCollidables.Count > startIndex + Vector<float>.Count);

            ref var firstShapeIndex = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var firstBodyIndex = ref Unsafe.As<Vector<int>, int>(ref bodyIndices);
            ref var firstExpansion = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                ref var collidable = ref collidables[Unsafe.Add(ref firstCollidable, i)];
                Unsafe.Add(ref firstShapeIndex, i) = collidable.ShapeIndex;
                Unsafe.Add(ref firstBodyIndex, i) = collidable.BodyIndex;
                Unsafe.Add(ref firstExpansion, i) = collidable.DetectionMode == DetectionMode.Discrete ? collidable.SpeculativeMargin : float.MaxValue;
            }
        }



        public abstract void FlushUpdates(Bodies bodies, ref QuickList<int, Buffer<int>> collidablesToUpdate);

        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    public class BodyCollidableBatch<TShape> : BodyCollidableBatch where TShape : struct, IShape
    {
        ShapeBatch<TShape> shapes;

        public BodyCollidableBatch(ShapeBatch shapeBatch, BufferPool pool, int initialCollidableCount)
            : base(pool, initialCollidableCount)
        {
            shapes = (ShapeBatch<TShape>)shapeBatch;
        }

        public interface IBoundingBoxBundleCalculator
        {

        }

        public void UpdatePrimitiveBoundingBoxes<TBoundingBoxBundleCalculator>(ref TBoundingBoxBundleCalculator calculator,
            Bodies bodies, float dt, ref QuickList<int, Buffer<int>> collidablesToUpdate)
        {
            for (int i = 0; i < collidablesToUpdate.Count; i += Vector<float>.Count)
            {
                //Create a bundle and execute it.
                GatherCollidableData(ref collidablesToUpdate, i, out var shapeIndices, out var bodyIndices);

                int count = collidablesToUpdate.Count - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;
                bodies.GatherPoseAndVelocity(ref bodyIndices, count, out var poses, out var velocities);
                calculator.GetBounds(shapes, ref shapeIndices, ref poses, out var min, out var max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumExpansion);
            }

        public override void FlushUpdates(Bodies bodies, ref QuickList<int, Buffer<int>> collidablesToUpdate)
        {
            Debug.Assert((collidablesToUpdate.Count & (collidablesToUpdate.Count - 1)) == 0 && collidablesToUpdate.Count >= Vector<float>.Count,
                "Primitive batches expect actual vector multiples.");

            for (int i = 0; i < collidablesToUpdate.Count; i += Vector<float>.Count)
            {
                //Create a bundle and execute it.
                ToShapeIndicesBundle(ref collidablesToUpdate, i, out var shapeIndices);
                SphereBundle bundle = new SphereBundle(shapes, shapeIndices);
                BodyPoses gatheredPoses;
                bundle.ComputeBoundingBoxes(ref gatheredPoses, out var min, out var max);
                BoundingBoxUpdater.exp
            }
        }
    }

    public interface IBroadPhase
    {
        int AllocateForCollidable(CollidableReference collidableReference);
    }

    public struct BroadPhaseWrapper<TBroadPhase> : IBroadPhase where TBroadPhase : IBroadPhase
    {
        public int AllocateForCollidable(CollidableReference collidableReference)
        {
            throw new NotImplementedException();
        }
    }


    public class BodyCollidables//<TBroadPhase> where TBroadPhase : struct, IBroadPhase
    {
        BodyCollidableBatch[] batches;
        public Shapes Shapes;
        IBroadPhase broadPhase;

        public BodyCollidableBatch this[int typeIndex] => batches[typeIndex];

        public int InitialCapacityPerCollidableBatch { get; set; }

        public BodyCollidables(Shapes shapes, int initialCapacityPerCollidableBatch)
        {
            //This set pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            batches = new BodyCollidableBatch[16];
        }

        public void Add(int bodyIndex, TypedIndex shapeIndex, BufferPool pool)
        {
            var typeIndex = shapeIndex.Type;
            var index = shapeIndex.Index;
            Debug.Assert(typeIndex >= 0 && typeIndex < Shapes.RegisteredTypeCount);
            if (batches.Length < Shapes.RegisteredTypeCount)
            {
                Array.Resize(ref batches, Shapes.RegisteredTypeCount);
            }
            ref var batch = ref batches[typeIndex];
            if (batch == null)
            {
                batch = Shapes[typeIndex].CreateBodyCollidableBatchForType(InitialCapacityPerCollidableBatch);
            }
            var collidableIndex = batch.Allocate(pool);
            ref var collidable = ref batch[collidableIndex];
            collidable.BodyIndex = bodyIndex;
            collidable.ShapeIndex = index;
            collidable.BroadPhaseIndex = broadPhase.AllocateForCollidable(new CollidableReference(true, typeIndex, collidableIndex));
        }

    }
}
