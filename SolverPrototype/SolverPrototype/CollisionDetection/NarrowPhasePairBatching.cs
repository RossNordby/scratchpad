using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{
    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public struct CollidablePair
    {
        [FieldOffset(0)]
        public CollidableReference A;
        [FieldOffset(4)]
        public CollidableReference B;
    }

    public struct CollidablePairComparer : IEqualityComparerRef<CollidablePair>
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


    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SpherePairTester
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Test(
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
        //public void GatherRigidPair(ref CollidablePair pair,
        //    out int shapeIndexA, out int shapeIndexB,
        //    int laneIndex, ref Vector<float> minimumDepth, ref Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB)
        //{
        //    ref var minimumDepthLane = ref GatherScatter.Get(ref minimumDepth, laneIndex);
        //    BodyPose poseA, poseB;
        //    if (pair.A.IsStatic)
        //    {
        //        //TODO: When non-body collidables exist, this needs to seek out the proper data source.
        //        poseA = new BodyPose();
        //        minimumDepthLane = 0;
        //    }
        //    else
        //    {
        //        var bodyIndex = Bodies.HandleToIndex[pair.A.CollidableIndex];
        //        Bodies.GetPoseByIndex(bodyIndex, out poseA);
        //        minimumDepthLane = -Bodies.Collidables[bodyIndex].SpeculativeMargin;
        //    }
        //    if (pair.B.IsStatic)
        //    {
        //        poseB = new BodyPose();
        //    }
        //    else
        //    {
        //        var bodyIndex = Bodies.HandleToIndex[pair.B.CollidableIndex];
        //        Bodies.GetPoseByIndex(bodyIndex, out poseB);
        //        minimumDepthLane -= Bodies.Collidables[bodyIndex].SpeculativeMargin;
        //    }
        //    BodyPose.GetRelativePosition(ref poseA, ref poseB, out var localB);
        //    GatherScatter.SetLane(ref localPositionB, laneIndex, ref localB, 3);
        //    GatherScatter.SetLane(ref orientationA.X, laneIndex, ref poseA.Orientation.X, 4);
        //    GatherScatter.SetLane(ref orientationB.X, laneIndex, ref poseB.Orientation.X, 4);
        //}
        //We special case the position-only version for the sake of sphere-sphere tests. Kinda questionable from a maintainability standpoint, but hey, super minor speedup!
        public void GatherRigidPair(ref CollidablePair pair,
            out int shapeIndexA, out int shapeIndexB,
            int laneIndex, ref Vector<float> minimumDepth, ref Vector3Wide localPositionB)
        {
            shapeIndexA = 0;
            shapeIndexB = 0;
            //ref var minimumDepthLane = ref GatherScatter.Get(ref minimumDepth, laneIndex);
            //BodyPose poseA, poseB;
            //if (pair.A.IsStatic)
            //{
            //    //TODO: When non-body collidables exist, this needs to seek out the proper data source.
            //    poseA = new BodyPose();
            //    minimumDepthLane = 0;
            //}
            //else
            //{
            //    var bodyIndex = Bodies.HandleToIndex[pair.A.CollidableIndex];
            //    Bodies.GetLane(bodyIndex, out poseA);
            //    minimumDepthLane = -Bodies.Collidables[bodyIndex].SpeculativeMargin;
            //}
            //if (pair.B.IsStatic)
            //{
            //    poseB = new BodyPose();
            //}
            //else
            //{
            //    var bodyIndex = Bodies.HandleToIndex[pair.B.CollidableIndex];
            //    Bodies.GetPoseByIndex(bodyIndex, out poseB);
            //    minimumDepthLane -= Bodies.Collidables[bodyIndex].SpeculativeMargin;
            //}
            //BodyPose.GetRelativePosition(ref poseA, ref poseB, out var localB);
            //GatherScatter.SetLane(ref localPositionB, laneIndex, ref localB, 3);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GetShapeBuffer<TShape>(out Buffer<TShape> shapesBuffer) where TShape : struct, IShape
        {
            var untypedBatch = Shapes[TypeIds<IShape>.GetId<TShape>()];
            shapesBuffer = Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref untypedBatch).shapes;
        }
    }

    ///// <summary>
    ///// Performs a collision test on a set of collidable pairs as a batch. Responsible for gathering necessary state, executing batches, and reporting results to pair owners.
    ///// </summary>
    //public abstract class CollidablePairTester
    //{
    //    public int BatchSize { get; protected set; }
    //    public abstract void Test(ref QuickList<PairJob, Buffer<PairJob>> jobs, ref Continuations owners, ref CollidableDataSource collidableSource);
    //}


    public enum ContinuationType
    {
        /// <summary>
        /// Convex-convex pair which will directly produce constraints.
        /// </summary>
        ConvexConstraintGenerator = 0,
        /// <summary>
        /// One of potentially multiple substeps produced by a collidable pair using substepped continuous collision detection.
        /// </summary>
        Substep = 1,
        /// <summary>
        /// Inner sphere test associated with a collidable pair using inner sphere continuous collision detection.
        /// </summary>
        InnerSphere = 2,
        /// <summary>
        /// Compound-convex or compound-compound subpair.
        /// </summary>
        Compound = 3,
        /// <summary>
        /// The pair belongs to a mesh-convex pair.
        /// </summary>
        Mesh = 3,
        /// <summary>
        /// Marks a pair as being owned by a mesh-mesh pair.
        /// </summary>
        MeshMesh = 4,
        /// <summary>
        /// Marks a pair as being owned by a mesh-compound pair.
        /// </summary>
        MeshCompound = 5,


    }
    public struct Continuation
    {
        uint packed;
        const int IndexBitCount = 28;
        public ContinuationType Type { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (ContinuationType)(packed >> IndexBitCount); } }

        public int Index { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (int)(packed & ((1 << IndexBitCount) - 1)); } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Continuation(ContinuationType type, int index)
        {
            packed = ((uint)type << 28) | (uint)index;
        }
    }


    public struct ConvexContactManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetB0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetB1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetB2;
        public Vector3Wide OffsetA3;
        public Vector3Wide OffsetB3;
        /// <summary>
        /// Surface basis for the manifolds, defining both normal and tangents.
        /// </summary>
        public QuaternionWide Basis;
        /// <summary>
        /// The number of contacts in the manifolds.
        /// </summary>
        public Vector<int> Count;
        /// <summary>
        /// The maximum number of contacts that this pair type could ever generate.
        /// </summary>
        public int MaximumCount;
    }

    //TODO: If we have any pair types that compute manifolds in a non-simd batched way, you'll need an overload of the continuations executor which is able to take them.
    //This is pretty likely- going wide on hull-hull is going to be tricky, and there's a lot of opportunity for internal SIMD usage.

    public enum PairConstraintType
    {
        Convex1 = 0, Convex2 = 1, Convex3 = 2, Convex4 = 3,
        Nonconvex1 = 4, Nonconvex2 = 5, Nonconvex3 = 6, Nonconvex4 = 7
    }
    public struct PairConstraintReference
    {
        uint packed;
        public PairConstraintType Type { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (PairConstraintType)(packed >> 29); } }
        public int Handle { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (int)(packed & ((1 << 29) - 1)); } }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairConstraintReference(PairConstraintType type, int constraintHandle)
        {
            Debug.Assert(constraintHandle < (1 << 29), "Constraint handles are assumed to be contiguous, positive, and not absurdly large.");
            packed = ((uint)type << 29) | (uint)constraintHandle;
        }
    }



    public struct CachedPairData
    {
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        uint packed;

        public bool Convex { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (packed & (1 << 31)) == (1 << 31); } }
        public int ContactCount { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return 1 + (int)((packed >> 29) & 0x3); } }
        public int Handle { [MethodImpl(MethodImplOptions.AggressiveInlining)]get { return (int)(packed & ((1 << 29) - 1)); } }

        public static ref int GetFeatureId(ref CachedPairData data, int i)
        {
            return ref Unsafe.Add(ref data.FeatureId0, i);
        }

        [Conditional("DEBUG")]
        private static void Validate(int constraintHandle)
        {
            Debug.Assert(constraintHandle < (1 << 29), "Constraint handles are assumed to be contiguous, positive, and not absurdly large.");
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateConvex(int constraintHandle, int featureId0, int featureId1, int featureId2, int featureId3, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = ((1u << 31) | (3u << 29)) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = featureId2;
            cachedPairData.FeatureId3 = featureId3;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateConvex(int constraintHandle, int featureId0, int featureId1, int featureId2, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = ((1u << 31) | (2u << 29)) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = featureId2;
            cachedPairData.FeatureId3 = 0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateConvex(int constraintHandle, int featureId0, int featureId1, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = ((1u << 31) | (1u << 29)) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = 0;
            cachedPairData.FeatureId3 = 0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateConvex(int constraintHandle, int featureId0, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = (1u << 31) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = 0;
            cachedPairData.FeatureId2 = 0;
            cachedPairData.FeatureId3 = 0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateNonconvex(int constraintHandle, int featureId0, int featureId1, int featureId2, int featureId3, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = (3u << 29) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = featureId2;
            cachedPairData.FeatureId3 = featureId3;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateNonconvex(int constraintHandle, int featureId0, int featureId1, int featureId2, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = (2u << 29) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = featureId2;
            cachedPairData.FeatureId3 = 0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateNonconvex(int constraintHandle, int featureId0, int featureId1, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = (1u << 29) | (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = featureId1;
            cachedPairData.FeatureId2 = 0;
            cachedPairData.FeatureId3 = 0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateNonconvex(int constraintHandle, int featureId0, out CachedPairData cachedPairData)
        {
            Validate(constraintHandle);
            cachedPairData.packed = (uint)constraintHandle;
            cachedPairData.FeatureId0 = featureId0;
            cachedPairData.FeatureId1 = 0;
            cachedPairData.FeatureId2 = 0;
            cachedPairData.FeatureId3 = 0;
        }
    }

    public struct PairMaterialProperties
    {
        public float FrictionCoefficient;
        public float MaximumRecoveryVelocity;
        public SpringSettingsAOS SpringSettings;
    }




    public struct PairJob
    {
        public CollidablePair Pair;
        public Continuation Continuation;
    }


    //public class SpherePairGatherExecuteReport : CollidablePairTester
    //{
    //    public SpherePairGatherExecuteReport()
    //    {
    //        BatchSize = 32;
    //    }
    //    //TODO: Compound children don't have a collidable reference, so they cannot be gathered in the same way.
    //    //Mesh children don't have a collidable reference, nor do they have a shape reference, so they can't be gathered in the same way.
    //    //We'll need to refactor this interface. 
    //    //Compound-Convex would decompose into a series of ChildConvex-Convex that know to gather the pose from the compound pose + child relative pose.
    //    //Mesh-Convex would decompose into a series of MeshTriangle-Convex pairs that gather vertex data from the mesh transform + local vertex positions.
    //    //Ideally we could find a good layout that shares as much as possible- it's not required that we have a separate batch for ChildSphere-Box, for example.
    //    //(However, in the mesh case, it's unlikely that we will support a standalone triangle shape, so there will likely be a MeshTriangle-Box and so on.)

    //    //Note that the collidable pair tester itself has no dynamic state.
    //    //This will be called from many threads. The caller is responsible for maintaining the necessary state in a thread safe way.
    //    public override void Test(ref QuickList<PairJob, Buffer<PairJob>> jobs, ref Continuations owners, ref CollidableDataSource collidableSource)
    //    {
    //        collidableSource.GetShapeBuffer(out Buffer<Sphere> shapes);
    //        Vector<float> radiiA, radiiB;
    //        Vector3Wide localPositionB;
    //        Vector<float> minimumDepth;
    //        ref var radiiStartA = ref Unsafe.As<Vector<float>, float>(ref radiiA);
    //        ref var radiiStartB = ref Unsafe.As<Vector<float>, float>(ref radiiB);
    //        for (int pairIndex = 0; pairIndex < jobs.Count; pairIndex += Vector<float>.Count)
    //        {
    //            var count = jobs.Count - pairIndex;
    //            if (count > Vector<float>.Count)
    //                count = Vector<float>.Count;
    //            //Gather everything necessary for the pair.
    //            for (int innerIndex = 0; innerIndex < count; ++innerIndex)
    //            {
    //                ref var pair = ref jobs[pairIndex + innerIndex];
    //                //For pairs between different shapes, this gather phase will need to work out which entry needs to go first. 
    //                //For example, if the sphere-box pair always uses the sphere first, then we might need to swap the order of A and B.
    //                //This is a sphere-sphere pair, so there's no need.
    //                collidableSource.GatherRigidPair(ref pair.Pair, out var shapeIndexA, out var shapeIndexB, innerIndex, ref minimumDepth, ref localPositionB);
    //                Unsafe.Add(ref radiiStartA, innerIndex) = shapes[shapeIndexA].Radius;
    //                Unsafe.Add(ref radiiStartB, innerIndex) = shapes[shapeIndexB].Radius;
    //            }
    //            //TODO: Check type punning impact on codegen. Had bugs and perf issues with that in the past.
    //            SpherePairTester.Test(ref radiiA, ref radiiB, ref minimumDepth, ref localPositionB, out var localContactPosition, out var contactNormal, out var depth, out var contactCount);
    //            //Scatter results to owners.
    //            for (int innerIndex = 0; innerIndex < count; ++innerIndex)
    //            {
    //                owners.Execute(jobs[pairIndex + innerIndex].Continuation);
    //            }
    //        }
    //    }
    //}


    ///// <summary>
    ///// Handles collision detection for a batch of collidable pairs together once filled or forced.
    ///// </summary>
    ///// <remarks>This is used by a single thread to accumulate collidable pairs over time until enough have been found to justify a wide execution.</remarks>
    //public struct PairBatch<TTester> where TTester : CollidablePairTester
    //{
    //    public QuickList<CollidablePair, Buffer<CollidablePair>> PendingPairs;
    //    public TTester Tester;

    //    public PairBatch(TTester tester, BufferPool pool) : this()
    //    {
    //        Tester = tester;
    //        QuickList<CollidablePair, Buffer<CollidablePair>>.Create(pool.SpecializeFor<CollidablePair>(), tester.BatchSize, out PendingPairs);
    //    }

    //    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    //    public void Add(ref CollidablePair pair)
    //    {
    //        PendingPairs.AddUnsafely(pair);
    //        if (PendingPairs.Count == Tester.BatchSize)
    //        {
    //            //Tester.Test(ref PendingPairs);
    //        }
    //    }

    //    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    //    public void Flush()
    //    {
    //        //if (PendingPairs.Count > 0)
    //        //    Tester.Test(ref PendingPairs);

    //    }
    //}
}
