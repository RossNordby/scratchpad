using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{

    public struct BallSocket : IConstraintDescription<BallSocket>
    {
        public Vector3 LocalOffsetA;
        public Vector3 LocalOffsetB;
        public SpringSettingsAOS SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ConstraintTypeIds.GetId<BallSocketTypeBatch>();
            }
        }

        public void ApplyDescription(TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch is BallSocketTypeBatch, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Unsafe.As<BallSocketTypeBatch>(batch).PrestepData[bundleIndex].LocalOffsetA.X, innerIndex);
            lane = LocalOffsetA.X;
            Unsafe.Add(ref lane, Vector<float>.Count) = LocalOffsetA.Y;
            Unsafe.Add(ref lane, 2 * Vector<float>.Count) = LocalOffsetA.Z;
            Unsafe.Add(ref lane, 3 * Vector<float>.Count) = LocalOffsetB.X;
            Unsafe.Add(ref lane, 4 * Vector<float>.Count) = LocalOffsetB.Y;
            Unsafe.Add(ref lane, 5 * Vector<float>.Count) = LocalOffsetB.Z;
            Unsafe.Add(ref lane, 6 * Vector<float>.Count) = SpringSettings.NaturalFrequency;
            Unsafe.Add(ref lane, 7 * Vector<float>.Count) = SpringSettings.DampingRatio;
        }

        public void BuildDescription(TypeBatch batch, int bundleIndex, int innerIndex, out BallSocket description)
        {
            Debug.Assert(batch is BallSocketTypeBatch, "The type batch passed to the description must match the description's expected type.");
            ref var lane = ref GatherScatter.Get(ref Unsafe.As<BallSocketTypeBatch>(batch).PrestepData[bundleIndex].LocalOffsetA.X, innerIndex);
            description.LocalOffsetA.X = lane;
            description.LocalOffsetA.Y = Unsafe.Add(ref lane, Vector<float>.Count);
            description.LocalOffsetA.Z = Unsafe.Add(ref lane, 2 * Vector<float>.Count);
            description.LocalOffsetB.X = Unsafe.Add(ref lane, 3 * Vector<float>.Count);
            description.LocalOffsetB.Y = Unsafe.Add(ref lane, 4 * Vector<float>.Count);
            description.LocalOffsetB.Z = Unsafe.Add(ref lane, 5 * Vector<float>.Count);
            description.SpringSettings.NaturalFrequency = Unsafe.Add(ref lane, 6 * Vector<float>.Count);
            description.SpringSettings.DampingRatio = Unsafe.Add(ref lane, 7 * Vector<float>.Count);
        }
    }

    public struct BallSocketPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public SpringSettings SpringSettings;
    }

    public struct BallSocketProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide BiasVelocity;
        public Triangular3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct BallSocketFunctions : IConstraintFunctions<BallSocketPrestepData, BallSocketProjection, Vector3Wide>
    {
        //TODO: There may be an argument for some extra level of abstraction here. If we gave the prestep function the data it needed (i.e. pose and inertia)
        //directly, it would be easier to share the implementation across different constraints. For example, it may be extremely common to use a particular set of 
        //constraints together- like ball socket, swing limit, and revolute angular joint- and you wouldn't want to regather inertia and pose for all of them.
        //(In that specific case, using a simultaneously solved ball socket + revolute angular joint would be preferred for stability, but the point stands.)
        //(Also... any time you use combo constraints, it is extremely likely that you need to modify the projections for optimal memory packing.
        //There are very few cases where a combo constraint will have less than 3DOFs...)
        //The only reason not to do that is codegen concerns. But we may want to stop holding back just because of some hopefully-not-permanent quirks in the JIT.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref UnpackedTwoBodyReferences bodyReferences, float dt, ref BallSocketPrestepData prestep,
            out BallSocketProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences,
                out var localPositionB, out var orientationA, out var orientationB,
                out projection.InertiaA, out projection.InertiaB);

            //Anchor points attached to each body are constrained to stay in the same position, yielding a position constraint of:
            //C = positionA + anchorOffsetA - (positionB + anchorOffsetB) = 0
            //C' = velocityA + d/dt(anchorOffsetA) - (velocityB + d/dt(anchorOffsetB)) = 0
            //C' = velocityA + angularVelocity x anchorOffsetA - (velocityB + angularVelocityB x anchorOffsetB) = 0
            //C' = velocityA * I + angularVelocity * skewSymmetric(anchorOffsetA) - velocityB * I - angularVelocityB * skewSymmetric(anchorOffsetB) = 0
            //So, the jacobians:
            //LinearA: I
            //AngularA: skewSymmetric(anchorOffsetA)
            //LinearB: -I
            //AngularB: skewSymmetric(-anchorOffsetB)
            //Each of these is a 3x3 matrix. However, we don't need to explicitly compute or store any of these.
            //Not storing the identity matrix is obvious enough, but we can also just store the offset rather than the full skew symmetric matrix for the angular jacobians.
            //The skew symmetric matrix is replaced by a simple cross product.

            //The projection should strive to contain the smallest possible representation necessary for the constraint to work.
            //We can sorta-kinda make use of the baking-inertia-into-jacobian trick, because I * softenedEffectiveMass is... softenedEffectiveMass.
            //Likewise, J * inverseInertia is just the scalar inverseMasses for the two linear contributions.
            //Note that there's no reason to try to bake the softenedEffectiveMass into the angular jacobian, because we already have to perform at least one 
            //softenedEffectiveMass multiply due to the linear components. We can just store the raw angular jacobians and piggyback on the linear component effective mass multiply.

            //So, the question becomes... is there a way to bundle inverseInertia into the angularJacobians?
            //Yes, but it's not useful. If we premultiply the skew(offset) * inverseInertia for CSIToWSV, the result is no longer symmetric.
            //That means we would have to store 3 additional scalars per body compared to the symmetric inverse inertias. That's not particularly useful;
            //it only saves a cross product. Loading 6 more scalars to save 2 cross products (12 multiplies, 6 adds) is a terrible trade, even at SIMD128.

            //Note that we must reconstruct the world offsets from the body orientations since we do not store world offsets.
            QuaternionWide.Transform(ref prestep.LocalOffsetA, ref orientationA, out projection.OffsetA);
            QuaternionWide.Transform(ref prestep.LocalOffsetB, ref orientationB, out projection.OffsetB);
            Triangular3x3Wide.SkewSandwichWithoutOverlap(ref projection.OffsetA, ref projection.InertiaA.InverseInertiaTensor, out var inverseEffectiveMass);
            //Note that the jacobian is technically skewSymmetric(-OffsetB), but the sign doesn't matter due to the sandwich.
            Triangular3x3Wide.SkewSandwichWithoutOverlap(ref projection.OffsetB, ref projection.InertiaB.InverseInertiaTensor, out var angularBContribution);
            Triangular3x3Wide.Add(ref inverseEffectiveMass, ref angularBContribution, out inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            var linearContribution = projection.InertiaA.InverseMass + projection.InertiaB.InverseMass;
            inverseEffectiveMass.M11 += linearContribution;
            inverseEffectiveMass.M22 += linearContribution;
            inverseEffectiveMass.M33 += linearContribution;
            Triangular3x3Wide.SymmetricInvert(ref inverseEffectiveMass, out projection.EffectiveMass);
            Springiness.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Triangular3x3Wide.Scale(ref projection.EffectiveMass, ref effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(ref localPositionB, ref projection.OffsetB, out var localB);
            Vector3Wide.Subtract(ref localB, ref projection.OffsetA, out var error);
            Vector3Wide.Scale(ref error, ref positionErrorToVelocity, out projection.BiasVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketProjection projection, ref Vector3Wide csi)
        {
            Vector3Wide.CrossWithoutOverlap(ref projection.OffsetA, ref csi, out var wsi);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref wsi, ref projection.InertiaA.InverseInertiaTensor, out var change);
            Vector3Wide.Add(ref velocityA.AngularVelocity, ref change, out velocityA.AngularVelocity);

            Vector3Wide.Scale(ref csi, ref projection.InertiaA.InverseMass, out change);
            Vector3Wide.Add(ref velocityA.LinearVelocity, ref change, out velocityA.LinearVelocity);

            Vector3Wide.CrossWithoutOverlap(ref csi, ref projection.OffsetB, out wsi); //note flip-negation
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref wsi, ref projection.InertiaB.InverseInertiaTensor, out change);
            Vector3Wide.Add(ref velocityB.AngularVelocity, ref change, out velocityB.AngularVelocity);

            Vector3Wide.Scale(ref csi, ref projection.InertiaB.InverseMass, out change);
            Vector3Wide.Subtract(ref velocityB.LinearVelocity, ref change, out velocityB.LinearVelocity); //note subtraction; the jacobian is -I
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Subtract(ref velocityA.LinearVelocity, ref velocityA.AngularVelocity, out var csv);
            Vector3Wide.CrossWithoutOverlap(ref velocityA.AngularVelocity, ref projection.OffsetA, out var angularCSV);
            Vector3Wide.Add(ref csv, ref angularCSV, out csv);
            Vector3Wide.CrossWithoutOverlap(ref projection.OffsetB, ref velocityB.AngularVelocity, out angularCSV);
            Vector3Wide.Add(ref csv, ref angularCSV, out csv);
            Vector3Wide.Subtract(ref projection.BiasVelocity, ref csv, out csv);

            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref csv, ref projection.EffectiveMass, out var csi);
            Vector3Wide.Scale(ref accumulatedImpulse, ref projection.SoftnessImpulseScale, out var softness);
            Vector3Wide.Subtract(ref csi, ref softness, out csi);

            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);

        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class BallSocketTypeBatch : TwoBodyTypeBatch<BallSocketPrestepData, BallSocketProjection, Vector3Wide, BallSocketFunctions>
    {

    }
}
