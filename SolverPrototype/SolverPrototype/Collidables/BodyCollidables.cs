using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Colldiables;
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


    public interface IBundleBounder
    {
        void Initialize(ShapeBatch batch);
        //Note that this requires the bundle bounder to have access to the type-specific shape data; a typed shape collection is not passed in.
        //This helps limit type exposure and generics explosion.
        void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max);
    }

    public interface IBoundsScatterer
    {
        void Scatter(ref Vector3Wide min, ref Vector3Wide max, int index);
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
        /// <param name="bodyIndexOfMovedCollidable">If the index was not the last slot in the collidables (and so this function returned true),
        /// this is the handle of the body owning the body collidable that was moved to fill its slot.
        /// If no collidable was moved (and so this function returned false), the value is undefined.</param>
        /// <returns>True if a body collidable was moved and reported, false otherwise.</returns>
        public bool RemoveAt(int index, out int bodyIndexOfMovedCollidable)
        {
            collidables.FastRemoveAt(index);
            if (index < collidables.Count)
            {
                bodyIndexOfMovedCollidable = collidables[index].BodyIndex;
                return true;
            }
            bodyIndexOfMovedCollidable = -1;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherCollidableData(ref QuickList<int, Buffer<int>> bundledCollidables, int startIndex,
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
                Unsafe.Add(ref firstExpansion, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }

        /// <summary>
        /// Defines a type that acts as a source of data needed for bounding box calculations.
        /// </summary>
        /// <remarks>
        /// Collidables may be pulled from objects directly in the world or from compound children. Compound children have to pull and compute information from the parent compound,
        /// and the result of the calculation has to be pushed back to the compound parent for further processing. In contrast, body collidables that live natively in the space simply
        /// gather data directly from the bodies set and scatter bounds directly into the broad phase.
        /// </remarks>
        public interface ICollidableBundleSource
        {
            /// <summary>
            /// Gets the number of collidables in this set of bundles.
            /// </summary>
            int Count { get; }
            /// <summary>
            /// Gathers collidable data required to calculate the bounding boxes for a bundle.
            /// </summary>
            /// <param name="collidablesStartIndex">Start index of the bundle in the collidables set to gather bounding box relevant data for.</param>
            void GatherCollidableBundle(int collidablesStartIndex, out Vector<int> shapeIndices, out Vector<float> maximumExpansion,
                out BodyPoses poses, out BodyVelocities velocities);
            /// <summary>
            /// Scatters the calculated bounds into the target memory locations.
            /// </summary>
            void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int collidablesStartIndex);
        }

        public struct NativePrimitiveBundleSource : ICollidableBundleSource
        { }

        public static void UpdatePrimitiveBoundingBoxes<TBundleBounder, TBundleSource>(
            ref TBundleSource bundleSource, ref TBundleBounder bundleBounder, Bodies bodies, float dt)
            where TBundleBounder : IBundleBounder where TBundleSource : ICollidableBundleSource
        {
            for (int i = 0; i < bundleSource.Count; i += Vector<float>.Count)
            {
                bundleSource.GatherCollidableBundle(i, out var shapeIndices, out var maximumExpansion, out var poses, out var velocities);

                //The bundle bounder is responsible for gathering shapes from type specific sources. Since it has type knowledge, it is able to both
                //gather the necessary shape information and call the appropriate bounding box calculator.
                bundleBounder.GetBounds(ref shapeIndices, ref poses, out var maximumRadius, out var min, out var max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumExpansion);

                //The bounding boxes are now fully expanded. Scatter them to the target location. For raw primitives in the broad phase, this means sticking them
                //in the broad phase's acceleration structure. For compound children, we stick them in the waiting compound temporary slots.
                //To avoid branching or virtual indirections, we once again abuse generics to supply the scattering logic.
                bundleSource.ScatterBounds(ref min, ref max, i);

            }
        }

        public abstract void FlushUpdates(Bodies bodies, float dt, ref QuickList<int, Buffer<int>> collidablesToUpdate);

        //TODO: Clear/EnsureCapacity/Resize/Compact/Dispose
    }

    struct SphereBundleBounder : IBundleBounder
    {
        ShapeBatch<Sphere> shapes;

        //Using an initialize function gets around generic issues with constructors.
        public void Initialize(ShapeBatch shapes)
        {
            this.shapes = (ShapeBatch<Sphere>)shapes;
        }

        public void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max)
        {
        }
    }


    public class PrimitiveCollidableBatch<TBundleBounder> : BodyCollidableBatch where TBundleBounder : struct, IBundleBounder
    {
        TBundleBounder bundleBounder;

        public PrimitiveCollidableBatch(ShapeBatch shapeBatch, BufferPool pool, int initialCollidableCount)
            : base(pool, initialCollidableCount)
        {
            bundleBounder = default(TBundleBounder);
            bundleBounder.Initialize(shapeBatch);
        }

        public sealed override void FlushUpdates(Bodies bodies, float dt, ref QuickList<int, Buffer<int>> collidablesToUpdate)
        {
            UpdatePrimitiveBoundingBoxes(this, ref bundleBounder, bodies, dt, ref collidablesToUpdate);
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

        public void Add(Bodies bodies, int bodyIndex, ref CollidableDescription collidableDescription, BufferPool pool)
        {
            var typeIndex = collidableDescription.ShapeIndex.Type;
            var index = collidableDescription.ShapeIndex.Index;
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
            //Note that this is responsible for setting the body's collidable reference back to the collidable. A little less gross in practice than forcing the simulation to handle
            //that mapping, and it's consistent with the way the Remove function works.
            bodies.Collidables[bodyIndex] = new TypedIndex(typeIndex, collidableIndex);
            ref var collidable = ref batch[collidableIndex];
            collidable.BodyIndex = bodyIndex;
            collidable.ShapeIndex = index;
            collidable.BroadPhaseIndex = broadPhase.AllocateForCollidable(new CollidableReference(true, typeIndex, collidableIndex));
            collidable.SpeculativeMargin = collidableDescription.SpeculativeMargin;
            collidable.Continuity = collidableDescription.Continuity;
        }

        public void Remove(Bodies bodies, TypedIndex collidableIndex)
        {
            if (batches[collidableIndex.Type].RemoveAt(collidableIndex.Index, out var bodyIndexOfMovedCollidable))
            {
                //When another collidable is moved into the removed slot, we must notify the body of the moved collidable of the new position.
                bodies.Collidables[bodyIndexOfMovedCollidable] = collidableIndex;
            }
        }
    }
}
