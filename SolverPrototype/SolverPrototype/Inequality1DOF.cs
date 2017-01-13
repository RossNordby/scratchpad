﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype
{
    /// <summary>
    /// Body data is stored in AOSOA for the integration step.
    /// From the solver's perspective, some form of gather is required for velocities regardless of the layout, so it might as well be optimal for some other stage.
    /// We also reuse this layout for storing constraint space velocities.
    /// </summary>
    public struct BodyVelocities
    {
        public Vector3Wide LinearVelocity;
        public Vector3Wide AngularVelocity;
    }


    public struct BodyInertias
    {
        public Matrix3x3Wide InverseInertiaTensor;
        public Vector<float> InverseMass;
    }

    public struct TwoBody1DOFJacobians
    {
        public Vector3Wide LinearA;
        public Vector3Wide AngularA;
        public Vector3Wide LinearB;
        public Vector3Wide AngularB;
    }
    /// <summary>
    /// A constraint's body references. Stored separately from the iteration data since it is accessed by both the prestep and solve.
    /// Two address streams isn't much of a problem for prefetching.
    /// </summary>
    public struct BodyReferences
    {
        //Unfortunately, there does not exist any Vector<int>.Shift instruction yet, so we cannot efficiently derive the bundle and inner indices from the 'true' indices on the fly.
        //Instead, group references are preconstructed and cached in a nonvectorized way.
        public Vector<int> BundleIndexA;
        public Vector<int> InnerIndexA;
        public Vector<int> BundleIndexB;
        public Vector<int> InnerIndexB;
        //TODO: there may be an argument to make this a full Vector<int> for padding reasons. We'd only ever access one component, but if alignment becomes an issue it could be a net win.
        public int Count;
    }

    public struct SpringSettings
    {
        public Vector<float> NaturalFrequency;
        public Vector<float> DampingRatio;
    }

    public struct ContactData
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide Normal;
        public Vector<float> PenetrationDepth;
    }

    public struct IterationData
    {
        //Rather than projecting from world space to constraint space *velocity* using JT, we precompute JT * effective mass
        //and go directly from world space velocity to constraint space impulse.
        public Vector3Wide WSVtoCSILinearA;
        public Vector3Wide WSVtoCSIAngularA;
        public Vector3Wide WSVtoCSILinearB;
        public Vector3Wide WSVtoCSIAngularB;

        //Since we jump directly from world space velocity to constraint space impulse, the velocity bias needs to be precomputed into an impulse offset too.
        public Vector<float> BiasImpulse;
        //And once again, CFM becomes CFM * EffectiveMass- massively cancels out due to the derivation of CFM. (See prestep notes.)
        public Vector<float> SoftnessImpulseScale;

        //It also needs to project from constraint space to world space.
        //We bundle this with the inertia/mass multiplier, so rather than taking a constraint impulse to world impulse and then to world velocity change,
        //we just go directly from constraint impulse to world velocity change.
        //For constraints with lower DOF counts, using this format also saves us some memory bandwidth- 
        //the inverse inertia tensor and inverse mass for a 2 body constraint cost 20 floats, compared to this implementation's 12.
        //(Note that even in an implementation where we use the body inertias, we should still cache it constraint-locally to avoid big gathers.)
        public Vector3Wide CSIToWSVLinearA;
        public Vector3Wide CSIToWSVAngularA;
        public Vector3Wide CSIToWSVLinearB;
        public Vector3Wide CSIToWSVAngularB;
    }


    public struct PenetrationConstraint
    {
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobiansAndError(ref ContactData contact, out TwoBody1DOFJacobians jacobians, out Vector<float> error)
        {
            //Technically we could take advantage of the redundant form on the linear jacobians, but it's made complex by the pretransformations.
            //For now, just leave it fully generic and unspecialized.

            //The contact penetration constraint takes the form:
            //dot(positionA + offsetA, N) <= dot(positionB + offsetB, N)
            //Or:
            //dot(positionA + offsetA, N) - dot(positionB + offsetB, N) <= 0
            //dot(positionA + offsetA - positionB - offsetB, N) <= 0
            //In practice, we'll use the collision detection system's penetration depth instead of trying to recompute the error here.

            //So, treating the normal as constant, the velocity constraint is:
            //dot(d/dt(positionA + offsetA - positionB - offsetB), N) <= 0
            //dot(linearVelocityA + d/dt(offsetA) - linearVelocityB - d/dt(offsetB)), N) <= 0
            //The velocity of the offsets are defined by the angular velocity.
            //dot(linearVelocityA + angularVelocityA x offsetA - linearVelocityB - angularVelocityB x offsetB), N) <= 0
            //dot(linearVelocityA, N) + dot(angularVelocityA x offsetA, N) - dot(linearVelocityB, N) - dot(angularVelocityB x offsetB), N) <= 0
            //Use the properties of the scalar triple product:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) - dot(linearVelocityB, N) - dot(offsetB x N, angularVelocityB) <= 0
            //Bake in the negations:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(-offsetB x N, angularVelocityB) <= 0
            //A x B = -B x A:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(N x offsetB, angularVelocityB) <= 0
            //And there you go, the jacobians!
            //linearA: N
            //angularA: offsetA x N
            //linearB: -N
            //angularB: N x offsetB
            jacobians.LinearA = contact.Normal;
            Vector3Wide.Negate(ref contact.Normal, out jacobians.LinearB);
            Vector3Wide.CrossWithoutOverlap(ref contact.OffsetA, ref contact.Normal, out jacobians.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref contact.Normal, ref contact.OffsetB, out jacobians.AngularB);

            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            error = contact.PenetrationDepth;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(BodyInertias[] bodyInertias, ref BodyReferences references, ref IterationData data, ref TwoBody1DOFJacobians jacobians, ref SpringSettings springSettings,
            ref Vector<float> positionError, ref Vector<float> maximumRecoveryVelocity, float dt, float inverseDt)
        {
            //effective mass = (J * M^-1 * JT)^-1
            //where J is a constraintDOF x bodyCount*3 sized matrix, JT is its transpose, and for two bodies M^-1 is:
            //[inverseMassA,    0, 0, 0]
            //[0, inverseInertiaA, 0, 0]
            //[0, 0, inverseMassB,    0]
            //[0, 0, 0, inverseInertiaB]
            //The entries of J match up to this convention, containing the linear and angular components of each body in sequence, so for a 2 body 1DOF constraint J would look like:
            //[linearA 1x3, angularA 1x3, linearB 1x3, angularB 1x3]
            //Note that it is a row vector by convention. When transforming velocities from world space into constraint space, it is assumed that the velocity vector is organized as a
            //row vector matching up to the jacobian (that is, [linearA 1x3, angularA 1x3, linearB 1x3, angularB 1x3]), so for a 2 body 2 DOF constraint,
            //worldVelocity * JT would be a [worldVelocity: 1x12] * [JT: 12x2], resulting in a 1x2 constraint space velocity row vector.
            //Similarly, when going from constraint space impulse to world space impulse in the above example, we would do [csi: 1x2] * [J: 2x12] to get a 1x12 world impulse row vector.

            //Note that the engine uses row vectors for all velocities and positions and so on. Rotation and inertia tensors are constructed for premultiplication. 
            //In other words, unlike many of the presentations in the space, we use v * JT and csi * J instead of J * v and JT * csi.
            //There is no meaningful difference- the two conventions are just transpositions of each other.

            //(If you want to know how this stuff works, go read the constraint related presentations: http://box2d.org/downloads/
            //Be mindful of the difference in conventions. You'll see J * v instead of v * JT, for example. Everything is still fundamentally the same, though.)
            GatherScatter.GatherInertia(bodyInertias, ref references, out var inertiaA, out var inertiaB);

            //Due to the block structure of the mass matrix, we can handle each component separately and then sum the results.
            //For this 1DOF constraint, the result is a simple scalar.
            //Note that we store the intermediate results of J * M^-1 for use when projecting from constraint space impulses to world velocity changes. 
            //If we didn't store those intermediate values, we could just scale the dot product of jacobians.LinearA with itself to save 4 multiplies.
            Vector3Wide.Multiply(ref jacobians.LinearA, ref inertiaA.InverseMass, out data.CSIToWSVLinearA);
            Vector3Wide.Multiply(ref jacobians.LinearB, ref inertiaB.InverseMass, out data.CSIToWSVLinearB);
            Vector3Wide.Dot(ref data.CSIToWSVLinearA, ref jacobians.LinearA, out var linearA);
            Vector3Wide.Dot(ref data.CSIToWSVLinearB, ref jacobians.LinearB, out var linearB);

            //The angular components are a little more involved; (J * I^-1) * JT is explicitly computed.
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out data.CSIToWSVAngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out data.CSIToWSVAngularB);
            Vector3Wide.Dot(ref data.CSIToWSVAngularA, ref jacobians.AngularA, out var angularA);
            Vector3Wide.Dot(ref data.CSIToWSVAngularB, ref jacobians.AngularB, out var angularB);

            //Now for a digression!
            //Softness is applied along the diagonal (which, for a 1DOF constraint, is just the only element).
            //Check the the ODE reference for a bit more information: http://ode.org/ode-latest-userguide.html#sec_3_8_0
            //And also see Erin Catto's Soft Constraints presentation for more details: http://box2d.org/files/GDC2011/GDC2011_Catto_Erin_Soft_Constraints.pdf)

            //There are some very interesting tricks you can use here, though.
            //Our core tuning variables are the damping ratio and natural frequency.
            //Our runtime used variables are softness and an error reduction feedback scale..
            //(For the following, I'll use the ODE terms CFM and ERP, constraint force mixing and error reduction parameter.)
            //So first, we need to get from damping ratio and natural frequency to stiffness and damping spring constants.
            //From there, we'll go to CFM/ERP.
            //Then, we'll create an expression for a softened effective mass matrix (i.e. one that takes into account the CFM term),
            //and an expression for the contraint force mixing term in the solve iteration.
            //Finally, compute ERP.
            //(And then some tricks.)

            //1) Convert from damping ratio and natural frequency to stiffness and damping constants.
            //The raw expressions are:
            //stiffness = effectiveMass * naturalFrequency^2
            //damping = effectiveMass * 2 * dampingRatio * naturalFrequency
            //Rather than using any single object as the reference for the 'mass' term involved in this conversion, use the effective mass of the constraint.
            //In other words, we're dynamically picking the spring constants necessary to achieve the desired behavior for the current constraint configuration.
            //(See Erin Catto's presentation above for more details on this.)

            //(Note that this is different from BEPUphysics v1. There, users configured stiffness and damping constants. That worked okay, but people often got confused about
            //why constraints didn't behave the same when they changed masses. Usually it manifested as someone creating an incredibly high mass object relative to the default
            //stiffness/damping, and they'd post on the forum wondering why constraints were so soft. Basically, the defaults were another sneaky tuning factor to get wrong.
            //Since damping ratio and natural frequency define the behavior independent of the mass, this problem goes away- and it makes some other interesting things happen...)

            //2) Convert from stiffness and damping constants to CFM and ERP.
            //CFM = (stiffness * dt + damping)^-1
            //ERP = (stiffness * dt) * (stiffness * dt + damping)^-1
            //Or, to rephrase:
            //ERP = (stiffness * dt) * CFM

            //3) Use CFM and ERP to create a softened effective mass matrix and a force mixing term for the solve iterations.
            //Start with a couple of base definitions which we won't be deriving. First, the softened effective mass:
            //softenedEffectiveMass = (effectiveMass^-1 + CFM)^-1
            //Then, the velocity target of the constraint. 
            //This means 'world space velocity projected into constraint space should equal the velocity bias term plus the constraint force mixing term'.
            //(The velocity bias term will be computed later- it's the position error scaled by the error reduction parameter, ERP. Position error is used to create a velocity motor goal.)
            //wsv * JT = bias + accumulatedImpulse * CFM

            //Here's where some trickiness occurs. (Be mindful of the distinction between the softened and unsoftened effective mass).
            //Start by substituting CFM into the softened effective mass definition:
            //softenedEffectiveMass = (effectiveMass^-1 + (stiffness * dt + damping)^-1)^-1
            //Now substitute the definitions of stiffness and damping, treating the scalar components as uniform scaling matrices of dimension equal to effectiveMass:
            //softenedEffectiveMass = (effectiveMass^-1 + ((effectiveMass * naturalFrequency^2) * dt + (effectiveMass * 2 * dampingRatio * naturalFrequency))^-1)^-1
            //Combine the inner effectiveMass coefficients, given matrix multiplication distributes over addition:
            //softenedEffectiveMass = (effectiveMass^-1 + (effectiveMass * (naturalFrequency^2 * dt) + effectiveMass * (2 * dampingRatio * naturalFrequency))^-1)^-1
            //softenedEffectiveMass = (effectiveMass^-1 + (effectiveMass * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency))^-1)^-1
            //Apply the inner matrix inverse:
            //softenedEffectiveMass = (effectiveMass^-1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * effectiveMass^-1)^-1
            //Once again, combine coefficients of the inner effectiveMass^-1 terms:
            //softenedEffectiveMass = ((1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1) * effectiveMass^-1)^-1
            //Apply the inverse again:
            //softenedEffectiveMass = effectiveMass * (1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1)^-1

            //So, to put it another way- because CFM is based on the effective mass, applying it to the effective mass results in a simple downscale.

            //What has been gained? Consider what happens in the solve iteration.
            //We take the velocity error:
            //velocityError = bias + accumulatedImpulse * CFM - wsv * JT 
            //and convert it to a corrective impulse with the effective mass:
            //impulse = (bias + accumulatedImpulse * CFM - wsv * JT) * softenedEffectiveMass
            //The effective mass distributes over the set:
            //impulse = bias * softenedEffectiveMass + accumulatedImpulse * CFM * softenedEffectiveMass - wsv * JT * softenedEffectiveMass
            //Focus on the CFM term:
            //accumulatedImpulse * CFM * softenedEffectiveMass
            //What is CFM * softenedEffectiveMass? Substitute.
            //(stiffness * dt + damping)^-1 * softenedEffectiveMass
            //((effectiveMass * naturalFrequency^2) * dt + (effectiveMass * 2 * dampingRatio * naturalFrequency))^-1 * softenedEffectiveMass
            //Combine terms: 
            //(effectiveMass * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency))^-1 * softenedEffectiveMass
            //Apply inverse:
            //(naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * effectiveMass^-1 * softenedEffectiveMass
            //Expand softened effective mass from earlier:
            //(naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * effectiveMass^-1 * effectiveMass * (1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1)^-1
            //Cancel effective masses: (!)
            //(naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * (1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1)^-1
            //Because CFM was created from effectiveMass, the CFM * effectiveMass term is actually independent of the effectiveMass!
            //The remaining expression is still a matrix, but fortunately it is a simple uniform scaling matrix that we can store and apply as a single scalar.

            //4) How do you compute ERP?
            //ERP = (stiffness * dt) * CFM
            //ERP = (stiffness * dt) * (stiffness * dt + damping)^-1
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * ((effectiveMass * naturalFrequency^2) * dt + (effectiveMass * 2 * dampingRatio * naturalFrequency))^-1
            //Combine denominator terms:
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * ((effectiveMass * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency))^-1
            //Apply denominator inverse:
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * effectiveMass^-1
            //Uniform scaling matrices commute:
            //ERP = (naturalFrequency^2 * dt) * effectiveMass * effectiveMass^-1 * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1
            //Cancellation!
            //ERP = (naturalFrequency^2 * dt) * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1
            //ERP = (naturalFrequency * dt) * (naturalFrequency * dt + 2 * dampingRatio)^-1
            //ERP is a simple scalar, independent of mass.

            //5) So we can compute CFM, ERP, the softened effective mass matrix, and we have an interesting shortcut on the constraint force mixing term of the solve iterations.
            //Is there anything more that can be done? You bet!
            //Let's look at the post-distribution impulse computation again:
            //impulse = bias * effectiveMass + accumulatedImpulse * CFM * effectiveMass - wsv * JT * effectiveMass
            //During the solve iterations, the only quantities that vary are the accumulated impulse and world space velocities. So the rest can be precomputed.
            //bias * effectiveMass,
            //CFM * effectiveMass,
            //JT * effectiveMass
            //In other words, we bypass the intermediate velocity state and go directly from source velocities to an impulse.
            //Note the sizes of the precomputed types above:
            //bias * effective mass is the same size as bias (vector with dimension equal to constrained DOFs)
            //CFM * effectiveMass is a single scalar regardless of constrained DOFs,
            //JT * effectiveMass is the same size as JT
            //But note that we no longer need to load the effective mass! It is implicit.
            //The resulting computation is:
            //impulse = a + accumulatedImpulse * b - wsv * c
            //two DOF-width adds (add/subtract), one DOF-width multiply, and a 1xDOF * DOFx12 jacobian-sized transform.
            //Compare to;
            //(accumulatedImpulse * softness + bias - wsv * JT) * effectiveMass
            //two DOF-width adds (add/subtract), one DOF width multiply, a 1xDOF * DOFx12 jacobian-sized transform, and a 1xDOF * DOFxDOF transform.
            //In other words, we shave off a whole 1xDOF * DOFxDOF transform per iteration.
            //So, taken in isolation, this is a strict win both in terms of memory and the amount of computation.

            //Unfortunately, it's not quite so simple- jacobians are ALSO used to transform the impulse into world space so that it can be used to change the body velocities.
            //We still need to have those around. So while we no longer store the effective mass, our jacobian has sort of been duplicated.
            //But wait, there's more!

            //That process looks like:
            //wsv += impulse * J * M^-1
            //So while we need to store something here, we can take advantage of the fact that we aren't using the jacobian anywhere else (it's replaced by the JT * effectiveMass term above).
            //Precompute J*M^-1, too.
            //So you're still loading a jacobian-sized matrix, but you don't need to load M^-1! That saves you 20 scalars. (3x3 + 1 + 3x3 + 1, inverse inertia and mass of two bodies.)
            //That saves you the multiplication of (impulse * J) * M^-1, which is 6 multiplies and 6 dot products.

            //Note that this optimization's value depends on the number of constrained DOFs.

            //Net memory change, opt vs no opt, in scalars:
            //1DOF: costs 1x12, saves 1x1 effective mass and the 20 scalar M^-1: -9
            //2DOF: costs 2x12, saves 2x2 effective mass and the 20 scalar M^-1: 0
            //3DOF: costs 3x12, saves 3x3 effective mass and the 20 scalar M^-1: 7
            //4DOF: costs 4x12, saves 4x4 effective mass and the 20 scalar M^-1: 12
            //5DOF: costs 5x12, saves 5x5 effective mass and the 20 scalar M^-1: 15
            //6DOF: costs 6x12, saves 6x6 effective mass and the 20 scalar M^-1: 16


            //Net compute savings, opt vs no opt:
            //DOF savings = 1xDOF * DOFxDOF (DOF DOFdot products), 2 1x3 * scalar (6 multiplies), 2 1x3 * 3x3 (6 3dot products)
            //            = (DOF*DOF multiplies + DOF*(DOF-1) adds) + (6 multiplies) + (18 multiplies + 12 adds)
            //            = DOF*DOF + 24 multiplies, DOF*DOF-DOF + 12 adds
            //1DOF: 25 multiplies, 12 adds
            //2DOF: 28 multiplies, 14 adds
            //3DOF: 33 multiplies, 18 adds
            //4DOF: 40 multiplies, 24 adds
            //5DOF: 49 multiplies, 32 adds
            //6DOF: 60 multiplies, 42 adds

            //So does our 'optimization' actually do anything useful? 
            //In 1 or 2 DOF constraints, it's a win with no downsides.
            //3+ are difficult to determine.
            //This depends on heavily on the machine's SIMD width. You do every lane's ALU ops in parallel, but the loads are still fundamentally bound by memory bandwidth.
            //The loads are coherent, at least- no gathers on this stuff. But I wouldn't be surprised if 3DOF+ constraints end up being faster *without* the pretransformations on wide SIMD.
            //This is just something that will require testing.

            //(Also, note that large DOF jacobians are often very sparse. Consider the jacobians used by a 6DOF weld joint. You could likely do special case optimizations to reduce the
            //load further. It is unlikely that you could find a way to do the same to JT * effectiveMass. J * M^-1 might have some savings, though. But J*M^-1 isn't *sparser*
            //than J by itself, so the space savings are limited. As long as you precompute, the above load requirement offset will persist.)

            //Good news, though! There are a lot of 1DOF and 2DOF constraints where this is an unambiguous win.

            //We'll start with the unsoftened effective mass, constructed from the contributions computed above:
            var effectiveMass = Vector<float>.One / (linearA + linearB + angularA + angularB);
            var frequencyDt = springSettings.NaturalFrequency * dt;
            var twiceDampingRatio = springSettings.DampingRatio * 2; //Could precompute.
            var extra = Vector<float>.One / (springSettings.NaturalFrequency * (frequencyDt + twiceDampingRatio));
            var effectiveMassCFMScale = Vector<float>.One / (Vector<float>.One + extra);
            var softenedEffectiveMass = effectiveMass * effectiveMassCFMScale;

            //CFM * softenedEffectiveMass:
            //(naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * (1 + (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1)^-1
            data.SoftnessImpulseScale = extra * effectiveMassCFMScale;

            //ERP = (naturalFrequency * dt) * (naturalFrequency * dt + 2 * dampingRatio)^-1
            var erp = frequencyDt / (frequencyDt + twiceDampingRatio);

            //Note that we use a bit of a hack when computing the bias velocity- even if our damping ratio/natural frequency implies a strongly springy response
            //that could cause a significant velocity overshoot, we apply an arbitrary clamping value to keep it reasonable.
            //This is useful for a variety of inequality constraints (like contacts) because you don't always want them behaving as true springs.
            var biasVelocity = Vector.Min(positionError * erp, maximumRecoveryVelocity);
            data.BiasImpulse = biasVelocity * softenedEffectiveMass;

            //Precompute the wsv * (JT * softenedEffectiveMass) term.
            Vector3Wide.Multiply(ref jacobians.LinearA, ref softenedEffectiveMass, out data.WSVtoCSILinearA);
            Vector3Wide.Multiply(ref jacobians.AngularA, ref softenedEffectiveMass, out data.WSVtoCSIAngularA);
            Vector3Wide.Multiply(ref jacobians.LinearB, ref softenedEffectiveMass, out data.WSVtoCSILinearB);
            Vector3Wide.Multiply(ref jacobians.AngularB, ref softenedEffectiveMass, out data.WSVtoCSIAngularB);
        }
        //Naming conventions:
        //We transform between two spaces, world and constraint space. We also deal with two quantities- velocities, and impulses. 
        //And we have some number of entities involved in the constraint. So:
        //wsva: world space velocity of body A
        //wsvb: world space velocity of body B
        //csvError: constraint space velocity error- when the body velocities are projected into constraint space and combined with the velocity biases, the result is a single constraint velocity error
        //csva: constraint space velocity of body A; the world space velocities projected onto transpose(jacobianA)
        //csvaLinear: contribution to the constraint space velocity by body A's linear velocity


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies,
        /// then scatters the result to the body velocity memory locations.
        /// </summary>
        public static void ApplyImpulse(BodyVelocities[] velocities, ref BodyReferences references, ref IterationData data,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Vector<float> correctiveImpulse)
        {
            //Applying the impulse requires transforming the constraint space impulse into a world space velocity change.
            //The first step is to transform into a world space impulse, which requires transforming by the transposed jacobian
            //(transpose(jacobian) goes from world to constraint space, jacobian goes from constraint to world space).
            //That world space impulse is then converted to a corrective velocity change by scaling the impulse by the inverse mass/inertia.
            //As an optimization for constraints with smaller jacobians, the jacobian * (inertia or mass) transform is precomputed.
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Multiply(ref data.CSIToWSVLinearA, ref correctiveImpulse, out correctiveVelocityA.LinearVelocity);
            Vector3Wide.Multiply(ref data.CSIToWSVAngularA, ref correctiveImpulse, out correctiveVelocityA.AngularVelocity);
            Vector3Wide.Multiply(ref data.CSIToWSVLinearB, ref correctiveImpulse, out correctiveVelocityB.LinearVelocity);
            Vector3Wide.Multiply(ref data.CSIToWSVAngularB, ref correctiveImpulse, out correctiveVelocityB.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.LinearVelocity, ref wsvA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.AngularVelocity, ref wsvA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.LinearVelocity, ref wsvB.LinearVelocity, out wsvB.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.AngularVelocity, ref wsvB.AngularVelocity, out wsvB.AngularVelocity);
            GatherScatter.ScatterVelocities(velocities, ref references, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(BodyVelocities[] velocities, ref BodyReferences references, ref IterationData data, ref Vector<float> accumulatedImpulse)
        {
            GatherScatter.GatherVelocities(velocities, ref references, out var wsvA, out var wsvB);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(velocities, ref references, ref data, ref wsvA, ref wsvB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref IterationData data, ref Vector<float> accumulatedImpulse,
            out Vector<float> correctiveCSI)
        {
            //Take the world space velocity of each body into constraint space by transforming by the transpose(jacobian).
            //(The jacobian is a row vector by convention, while we treat our velocity vectors as a 12x1 row vector for the purposes of constraint space velocity calculation.
            //So we are multiplying v * JT.)
            //Then, transform it into an impulse by applying the effective mass.
            //Here, we combine the projection and impulse conversion into a precomputed value, i.e. v * (JT * softenedEffectiveMass).
            Vector3Wide.Dot(ref wsvA.LinearVelocity, ref data.WSVtoCSILinearA, out var csiaLinear);
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref data.WSVtoCSIAngularA, out var csiaAngular);
            Vector3Wide.Dot(ref wsvB.LinearVelocity, ref data.WSVtoCSILinearB, out var csibLinear);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref data.WSVtoCSIAngularB, out var csibAngular);
            //Combine it all together, following:
            //constraint space impulse = (targetVelocity - currentVelocity) * softenedEffectiveMass
            //constraint space impulse = (bias + accumulatedImpulse * softness - wsv * JT) * softenedEffectiveMass
            //constraint space impulse = (bias * softenedEffectiveMass) + accumulatedImpulse * (softness * softenedEffectiveMass) - wsv * (JT * softenedEffectiveMass)
            var csi = data.BiasImpulse + accumulatedImpulse * data.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Min(Vector<float>.Zero, accumulatedImpulse + csi);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(BodyVelocities[] velocities, ref BodyReferences references, ref IterationData data, ref Vector<float> accumulatedImpulse)
        {
            GatherScatter.GatherVelocities(velocities, ref references, out var wsvA, out var wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(velocities, ref references, ref data, ref wsvA, ref wsvB, ref correctiveCSI);

        }

    }
}
