using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    public struct BallSocketPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated BallSocketConstraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public SpringSettings SpringSettings;
    }

    public struct BallSocketProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector<float> SoftnessImpulseScale;
        //The csi * (skew(offset) * inverseInertia) term is baked together to avoid unnecessary cross products.
        //The alternative was to store the inverse inertia, so this costs no extra space.
        public Matrix3x3Wide AngularCSIToWSVA;
        public Matrix3x3Wide AngularCSIToWSVB;
    }

    public struct BallSocketFunctions : IConstraintFunctions<BallSocketPrestepData, BallSocketProjection, Vector3Wide>
    {
        public void Prestep(Bodies bodies, ref UnpackedTwoBodyReferences bodyReferences, float dt, float inverseDt, ref BallSocketPrestepData prestep,
            out BallSocketProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences,
                out var localPositionB, out var orientationA, out var orientationB, out var inertiaA, out var inertiaB);

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

            //Note that we must reconstruct the world offsets from the body orientations since we do not store world offsets.
            Quaternion.Transform(ref prestep.LocalOffsetA, ref orientationA, out projection.OffsetA);
            Quaternion.Transform(ref prestep.LocalOffsetB, ref orientationB, out projection.OffsetB);
            //Even though we do not explicitly use the matrix form as a projection, it is useful to create it in the prestep for the sake of constructing the effective mass matrix.
            Matrix3x3Wide.CreateCrossProduct(ref projection.OffsetA, out var offsetMatrixA);
            Matrix3x3Wide.MultiplyWithoutOverlap(ref offsetMatrixA, ref inertiaA.InverseInertia, out var intermediate);
            Matrix3x3Wide.MultiplyWithoutOverlap(ref intermediate, ref offsetMatrixA, out var inverseEffectiveMass);
            Matrix3x3Wide.CreateCrossProduct(ref projection.OffsetB, out var offsetMatrixB);
            Matrix3x3Wide.MultiplyWithoutOverlap(ref offsetMatrixA, ref inertiaA.InverseInertia, out intermediate);
            //There's a weak argument to combine this multiply with the add. 
            //There's a good chance doing so would improve codegen, especially regarding locals initialization. Complexity costs.
            Matrix3x3Wide.MultiplyWithoutOverlap(ref intermediate, ref offsetMatrixA, out var angularBContribution);
            Matrix3x3Wide.Add(ref angularBContribution, ref inverseEffectiveMass, out inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            var linearContribution = inertiaA.InverseMass + inertiaB.InverseMass;
            inverseEffectiveMass.X.X += linearContribution;
            inverseEffectiveMass.Y.Y += linearContribution;
            inverseEffectiveMass.Z.Z += linearContribution;
            Matrix3x3Wide.CholeskyInverse(ref inverseEffectiveMass, out var projection.EffectiveMass);
            Springiness.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Matrix3x3Wide.Scale(ref projection.EffectiveMass, ref effectiveMassCFMScale, out projection.EffectiveMass);



            //The projection should strive to contain the smallest possible representation necessary for the constraint to work.
            //We can sorta-kinda  make use of the baking-inertia-into-jacobian trick, because I * softenedEffectiveMass is... softenedEffectiveMass.
            //Likewise, J * inverseInertia is just the scalar inverseMasses for the two linear contributions.
            //Note that there's no reason to try to bake the softenedEffectiveMass into the angular jacobian, because we already have to perform at least one 
            //softenedEffectiveMass multiply due to the linear components. We can just store the raw angular jacobians and piggyback on the linear component effective mass multiply.

            //So, the question becomes... is there a way to bundle inverseInertia into the angularJacobians?
            //Yes!

            //(impulse x angularJ) * inverseInertia
            //(impulse * skew(angularJ)) * inverseInertia
            //impulse * (skew(angularJ) * inverseInertia)
            //Rather than storing out the inverseInertia for each of the two bodies, we can store the skew symmetric representation of the cross product multiplied by the inertia.
            //This doesn't save any space, but it does avoid two cross products per solve iteration. 

            //We can't compress the angular component down to nothing but a cross product, because there is no vector v such that:
            //skew(angularJ) * inverseInertia = skew(v)
            //for all inverseInertias. 

            //But there's another consideration:
            //The (inverse) inertia tensor is symmetric. 


            //However, we have one more trick:
            //The softened effective mass matrix and the (skew(angularJ) * inverseInertia) are both symmetric. In other words, it's pointless to store one of the two repeated halves.
            //That saves 9 scalars per constraint!



        }

        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            throw new NotImplementedException();
        }

        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            throw new NotImplementedException();
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class BallSocketTypeBatch : TwoBodyTypeBatch<BallSocketPrestepData, BallSocketProjection, Vector3Wide, BallSocketFunctions>
    {
        struct BallSocketPrestep : IPosedPrestep
        {
            public void Prestep(ref BallSocketPrestepData prestepData,
                ref BodyInertias inertiaA, ref BodyInertias inertiaB,
                ref Vector3Wide relativePosition, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
                out BallSocketProjection projection)
            {
                projection = new BallSocketProjection();
            }
        }

        public override void Prestep(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            PosedPrestep<BallSocketPrestep>(bodies, dt, inverseDt, startBundle, exclusiveEndBundle);
        }
        public override void WarmStart(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            ref var projectionBase = ref Projection[0];
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }

        public override void SolveIteration(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var projectionBase = ref Projection[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);

                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }

    }
}
