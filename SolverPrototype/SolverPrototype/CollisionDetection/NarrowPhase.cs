﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype.CollisionDetection
{

    /*
     * The narrow phase operates on overlaps generated by the broad phase. 
     * Its job is to compute contact manifolds for overlapping collidables and to manage the constraints produced by those manifolds. 
     * 
     * The scheduling of collision detection jobs is conceptually asynchronous. There is no guarantee that a broad phase overlap provided to the narrow phase
     * will result in an immediate calculation of the manifold. This is useful for batching together many collidable pairs of the same time for simultaneous SIMD-friendly execution.
     * (Not all pairs are ideal fits for SIMD, but many common and simple ones are.)
     * 
     * The interface to the broad phase makes no guarantees about the nature of this batching. The narrow phase could immediately execute, or it could batch up Vector<float>.Count,
     * or maybe 32 in a row, or it could wait until all overlaps have been submitted before actually beginning work.
     * 
     * This deferred execution requires that the pending work be stored somehow. This is complicated by the fact that there are a variety of different top level pairs that handle
     * incoming contact manifold data and the resulting constraints in different ways. There are two main distinctions:
     * 1) Continuous collision detection mode. For the purposes of the narrow phase, each collidable can be thought of as discrete, inner sphere, substepping, or inner sphere + substepping.
     * -Discrete pairs take the result of the underlying manifold and directly manipulate regular contact constraints. 
     * -Inner sphere pairs, with sufficient relative linear velocity, can create one or two additional sphere-convex pairs per convex pair.
     * -Substepping pairs potentially generate a bunch of child pairs, depending on the collidable velocities, and then choose from the resulting manifolds. 
     * Once the best manifold is selected, constraint management is similar to the discrete case.
     * -Inner sphere + substepping pairs just do both of the above.
     * 2) Individual versus compound types. Compound pairs will tend to create child convex pairs and wait for their completion. This ensures the greatest number of simultaneous
     * SIMD-friendly manifold calculations. For example, four compound-compound pairs could result in 60 sphere-capsule subpairs which can then all be executed in a SIMD fashion.
     * 
     * These two build on each other- a compound-compound pair with inner sphere enabled will want to generate both the inner sphere pairs and the regular pairs simultaneously to avoid 
     * traversing any acceleration structures multiple times.
     * 
     * Note that its possible for the evaluation of a pair to generate more pairs. This is most easily seen in compound pairs or substep pairs, but we do permit less obvious cases.
     * For example, a potential optimization for substepping is only do as many substeps as are needed to find the first manifold with approaching contacts (or some other heuristic).
     * In order for such an optimization to be used, we must be willing to spawn more pairs if the first set of substeps we did didn't find any heuristically accepted manifolds.
     * In the limit, that would mean doing one substep at a time. (In practice, we'd probably just try to fill up the remainder of a SIMD batch.)
     * 
     * Another example: imagine a high-complexity convex-convex test that has highly divergent execution, but with smaller pieces which are not as divergent.
     * SIMD operations don't map well to divergent execution, so if the individual jobs are large enough, it could be worth it to spawn new pairs for the nondivergent pieces.
     * Most convexes aren't complicated enough to warrant this (often it's faster to simply execute all paths), but it may be relevant in the convex hull versus convex hull case.
     * 
     * In any case where more pairs are generated, evaluating just the current set of pairs is insufficient to guarantee completion. Instead, execution can be thought of like traversing a graph.
     * Each work-creating pair may create an entry on the execution stack if its 'execution threshold' is reached (the arbitrary size which, when reached, results in the execution of the 
     * stored pairs). When no jobs remain on the stack, take any available stored pair set and try to execute it- even if it hasn't yet reached its execution threshold. In this situation,
     * without further action it won't ever fill up, so there's no reason to wait. That execution may then spawn more work, which could create an element on the execution stack, and so on. 
     * Ideally, job sets are consumed in order of their probability of creating new work. That maximizes the number of SIMD-friendly executions.
     * 
     * In practice, there are two phases. The first phase takes in the broad phase-generated top level pairs. At this stage, we do not need to resort to executing incomplete bundles. 
     * Instead, we just continue to work on the top level pairs until none remain. The second phase kicks in here. Since no further top-level work is being generated, we start trying to 
     * flush all the remaining pairs, even if they are not at the execution threshold, as in the above traverse-and-reset approach.
     * 
     * All of the above works within the context of a single thread. There may be many threads in flight, but each one is guaranteed to be handling different top level pairs.
     * That means all of the pair storage is thread local and requires no synchronization. It is also mostly ephemeral- once the thread finishes, only a small amount of information needs
     * to be persisted to globally accessed memory. (Overlap->ConstraintHandle is one common piece of data, but some pairs may also persist other data like separating axes for early outs.
     * Such extra data is fairly rare, since it implies divergence in execution- which is something you don't want in a SIMD-friendly implementation. Likely only in things like hull-hull.)
     * 
     * Every narrow phase pair is responsible for managing the constraints that its computed manifolds require. 
     * This requires the ability to look up existing overlap->constraint relationships for three reasons:
     * 1) Any existing constraint, if it has the same number of contacts as the new manifold, should have its contact data updated.
     * 2) Any accumulated impulse from the previous frame's contact solve should be distributed over the new set of contacts for warm starting this frame's solve.
     * 3) Any change in contact count should result in the removal of the previous constraint (if present) and the addition of the new constraint (if above zero contacts).
     * This mapping is stored in a single dictionary. The previous frame's mapping is treated as read-only by the new frame, so no synchronization is required to read it. The current frame builds
     * a new dictionary incrementally. It starts from scratch, so only actually-needed overlaps will exist in the new dictionary.
     * 
     * Constraints associated with 'stale' overlaps (those which were not updated during the current frame) are removed in a postpass.
     * 
     */

    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public struct CollidablePair
    {
        [FieldOffset(0)]
        public CollidableReference A;
        [FieldOffset(4)]
        public CollidableReference B;
    }

    struct CollidablePairComparer : IEqualityComparerRef<CollidablePair>
    {
        //The order of collidables in the pair should not affect equality or hashing. The broad phase is not guaranteed to provide a reliable order.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref CollidablePair a, ref CollidablePair b)
        {
            return Unsafe.As<CollidablePair, ulong>(ref a) == Unsafe.As<CollidablePair, ulong>(ref b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref CollidablePair item)
        {
            return (int)(item.A.packed ^ item.B.packed);
        }
    }

    /// <summary>
    /// Performs a collision test on a set of collidable pairs as a batch.
    /// </summary>
    public interface ICollidablePairTester
    {
        int BatchSize { get; }
        void Test(ref QuickList<CollidablePair, Buffer<CollidablePair>> pairs);
    }

    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester
    {
        public int BatchSize { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Vector<float>.Count; } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref Vector<float> radiiA, ref Vector<float> radiiB,
            ref Vector<float> minimumDepth,
            ref Vector3Wide relativePositionB,
            out Vector3Wide relativeContactPosition, out Vector3Wide contactNormal, out Vector<float> depth, out Vector<int> contactCount)
        {
            Vector3Wide.Length(ref relativePositionB, out var centerDistance);
            var inverseDistance = Vector<float>.One / centerDistance;
            Vector3Wide.Scale(ref relativePositionB, ref inverseDistance, out contactNormal);
            var normalIsValid = Vector.GreaterThan(centerDistance, Vector<float>.Zero);
            //Arbitrarily choose the (0,1,0) if the two spheres are in the same position. Any unit length vector is equally valid.
            contactNormal.X = Vector.ConditionalSelect(normalIsValid, contactNormal.X, Vector<float>.Zero);
            contactNormal.Y = Vector.ConditionalSelect(normalIsValid, contactNormal.Y, Vector<float>.One);
            contactNormal.Z = Vector.ConditionalSelect(normalIsValid, contactNormal.Z, Vector<float>.Zero);
            depth = radiiA + radiiB - centerDistance;
            //The position should be placed at the average of the extremePoint(a, a->b) and extremePoint(b, b->a). That puts it in the middle of the overlapping or nonoverlapping interval.
            //The contact normal acts as the direction from a to b.
            Vector3Wide.Scale(ref contactNormal, ref radiiA, out var extremeA);
            Vector3Wide.Scale(ref contactNormal, ref radiiB, out var extremeB);
            //note the following subtraction: contactNormal goes from a to b, so the negation pushes the extreme point in the proper direction from b to a.
            Vector3Wide.Subtract(ref relativePositionB, ref extremeB, out extremeB);
            Vector3Wide.Add(ref extremeA, ref extremeB, out relativeContactPosition);
            var scale = new Vector<float>(0.5f);
            Vector3Wide.Scale(ref relativeContactPosition, ref scale, out relativeContactPosition);
            contactCount = Vector.ConditionalSelect(Vector.GreaterThanOrEqual(depth, minimumDepth), Vector<int>.One, Vector<int>.Zero);
        }
    }

    public struct CollidableDataSource
    {
        public Shapes Shapes;
        public Bodies Bodies;

        public CollidableDataSource(Bodies bodies, Shapes shapes)
        {
            Shapes = shapes;
            Bodies = bodies;
        }
        public void GatherRigidPair(ref CollidablePair pair,
            int laneIndex, ref Vector<float> minimumDepth, ref Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB)
        {
            ref var minimumDepthLane = ref GatherScatter.Get(ref minimumDepth, laneIndex);
            BodyPose poseA, poseB;
            if (pair.A.IsStatic)
            {
                //TODO: When non-body collidables exist, this needs to seek out the proper data source.
                poseA = new BodyPose();
                minimumDepthLane = 0;
            }
            else
            {
                var bodyIndex = Bodies.HandleToIndex[pair.A.CollidableIndex];
                Bodies.GetPoseByIndex(bodyIndex, out poseA);
                minimumDepthLane = -Bodies.Collidables[bodyIndex].SpeculativeMargin;
            }
            if (pair.B.IsStatic)
            {
                poseB = new BodyPose();
            }
            else
            {
                var bodyIndex = Bodies.HandleToIndex[pair.B.CollidableIndex];
                Bodies.GetPoseByIndex(bodyIndex, out poseB);
                minimumDepthLane -= Bodies.Collidables[bodyIndex].SpeculativeMargin;
            }
            BodyPose.GetRelativePosition(ref poseA, ref poseB, out var localB);
            GatherScatter.SetLane(ref localPositionB, laneIndex, ref localB, 3);
            GatherScatter.SetLane(ref orientationA.X, laneIndex, ref poseA.Orientation.X, 4);
            GatherScatter.SetLane(ref orientationB.X, laneIndex, ref poseB.Orientation.X, 4);
        }
        //We special case the position-only version for the sake of sphere-sphere tests. Kinda questionable from a maintainability standpoint, but hey, super minor speedup!
        public void GatherRigidPair(ref CollidablePair pair,
            int laneIndex, ref Vector<float> minimumDepth, ref Vector3Wide localPositionB)
        {
            ref var minimumDepthLane = ref GatherScatter.Get(ref minimumDepth, laneIndex);
            BodyPose poseA, poseB;
            if (pair.A.IsStatic)
            {
                //TODO: When non-body collidables exist, this needs to seek out the proper data source.
                poseA = new BodyPose();
                minimumDepthLane = 0;
            }
            else
            {
                var bodyIndex = Bodies.HandleToIndex[pair.A.CollidableIndex];
                Bodies.GetLane((bodyIndex, out poseA);
                minimumDepthLane = -Bodies.Collidables[bodyIndex].SpeculativeMargin;
            }
            if (pair.B.IsStatic)
            {
                poseB = new BodyPose();
            }
            else
            {
                var bodyIndex = Bodies.HandleToIndex[pair.B.CollidableIndex];
                Bodies.GetPoseByIndex(bodyIndex, out poseB);
                minimumDepthLane -= Bodies.Collidables[bodyIndex].SpeculativeMargin;
            }
            BodyPose.GetRelativePosition(ref poseA, ref poseB, out var localB);
            GatherScatter.SetLane(ref localPositionB, laneIndex, ref localB, 3);
        }
    }


    public struct SpherePairGatherer
    {
        public
    }


    /// <summary>
    /// Handles collision detection for a batch of collidable pairs together once filled or forced.
    /// </summary>
    /// <remarks>This is used by a single thread to accumulate collidable pairs over time until enough have been found to justify a wide execution.</remarks>
    public struct PairBatch<TTester> where TTester : ICollidablePairTester
    {
        public QuickList<CollidablePair, Buffer<CollidablePair>> PendingPairs;
        public TTester Tester;

        public PairBatch(TTester tester, BufferPool pool) : this()
        {
            Tester = tester;
            QuickList<CollidablePair, Buffer<CollidablePair>>.Create(pool.SpecializeFor<CollidablePair>(), tester.BatchSize, out PendingPairs);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref CollidablePair pair)
        {
            PendingPairs.AddUnsafely(pair);
            if (PendingPairs.Count == Tester.BatchSize)
            {
                Tester.Test(ref PendingPairs);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {
            if (PendingPairs.Count > 0)
                Tester.Test(ref PendingPairs);

        }
    }

    public class NarrowPhase
    {
        public Bodies Bodies;
        public BufferPool Pool;
        //TODO: It is possible that some types will benefit from per-overlap data, like separating axes. For those, we should have type-dedicated overlap dictionaries.
        //The majority of type pairs, however, only require a constraint handle.
        QuickDictionary<CollidablePair, int, Buffer<CollidablePair>, Buffer<int>, Buffer<int>, CollidablePairComparer> constraintHandles;

        public NarrowPhase(Bodies bodies, BufferPool pool, int initialOverlapCapacity = 32768)
        {
            Bodies = bodies;
            Pool = pool;
            QuickDictionary<CollidablePair, int, Buffer<CollidablePair>, Buffer<int>, Buffer<int>, CollidablePairComparer>.Create(
                pool.SpecializeFor<CollidablePair>(), pool.SpecializeFor<int>(), pool.SpecializeFor<int>(), SpanHelper.GetContainingPowerOf2(initialOverlapCapacity), 3, out constraintHandles);

        }

        public void EnsureCapacity(int overlapCapacity)
        {
            //TODO: If there are type specialized overlap dictionaries, this must be updated.
            constraintHandles.EnsureCapacity(overlapCapacity, Pool.SpecializeFor<CollidablePair>(), Pool.SpecializeFor<int>(), Pool.SpecializeFor<int>());

        }



        public void HandleOverlap(CollidableReference a, CollidableReference b)
        {
            var staticness = (a.packed >> 31) | ((b.packed & 0x7FFFFFFF) >> 30);
            switch (staticness)
            {
                case 0:
                    {
                        //Both references are bodies.
                        //This is a body. In order to dispatch it properly, we need to know some metadata.
                        //TODO: Once inactive bodies exist, this will need to be updated.
                        ref var aCollidable = ref Bodies.Collidables[a.CollidableIndex];
                        ref var bCollidable = ref Bodies.Collidables[b.CollidableIndex];
                        if (aCollidable.Continuity.UseSubstepping || bCollidable.Continuity.UseSubstepping)
                        {
                            //Pull the velocity information for all involved bodies. We will request a number of steps that will cover the motion path.
                            //number of substeps = min(maximum substep count, 1 + floor(estimated displacement / step length)), where
                            //estimated displacement = dt * (length(linear velocity A - linear velocity B) +
                            //                               maximum radius A * (length(angular velocity A) + maximum radius B * length(angular velocity B)) 
                            //Once we have a number of 
                            //We use the minimum step length of each contributing collidable. Treat non-substepping collidables as having a step length of infinity.
                            var stepLengthA = aCollidable.Continuity.UseSubstepping ? aCollidable.Continuity.MaximumStepLength : float.MaxValue;
                            var stepLengthB = bCollidable.Continuity.UseSubstepping ? bCollidable.Continuity.MaximumStepLength : float.MaxValue;
                            float stepLength = stepLengthA < stepLengthB ? stepLengthA : stepLengthB;
                        }
                    }
                    break;
                case 1:
                    {
                        //Collidable a is a body, b is a static.
                        //TODO: Once non-body collidables exist, this will need to be updated.
                    }
                    break;
                case 2:
                    {
                        //Collidable a is a static, b is a body.
                        //TODO: Once non-body collidables exist, this will need to be updated.
                    }
                    break;
                case 3:
                    {
                        //Both collidables are statics. This is a bit of a weird situation- under normal conditions, static bodies will belong to the 
                        //'inactive' broad phase tree, and the inactive tree is not tested against itself. The user must have configured this static to be in the active tree to act
                        //as a detector or something along those lines.
                        //TODO: Once non-body collidables exist, this will need to be updated.
                    }
                    break;
            }

        }
    }
}