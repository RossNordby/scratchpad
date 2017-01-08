using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype
{

    public struct Matrix3x3Wide
    {
        public Vector<float> M11;
        public Vector<float> M12;
        public Vector<float> M13;
        public Vector<float> M21;
        public Vector<float> M22;
        public Vector<float> M23;
        public Vector<float> M31;
        public Vector<float> M32;
        public Vector<float> M33;
    }
    public struct Vector3Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Dot(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Multiply(ref Vector<float> scalar, ref Vector3Wide vector, out Vector3Wide result)
        {
            result.X = scalar * vector.X;
            result.Y = scalar * vector.Y;
            result.Z = scalar * vector.Z;
        }
    }


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

    /// <summary>
    /// A constraint's body references. Stored separately from the iteration data since it is accessed by both the prestep and solve.
    /// Two address streams isn't much of a problem for prefetching.
    /// </summary>
    public struct BodyReferences
    {
        //Unfortunately, there does not exist any Vector<int>.Shift instruction yet, so we cannot efficiently derive the bundle and inner indices from the 'true' indices on the fly.
        //Instead, group references are preconstructed and cached in a nonvectorized way.
        public Vector<int> BundleIndexA;
        public Vector<int> BundleIndexB;
        //Inner indices contain 
        public Vector<int> InnerIndexA;
        public Vector<int> InnerIndexB;
        //TODO: there may be an argument to make this a full Vector<int> for padding reasons. We'd only ever access one component, but if alignment becomes an issue it could be a net win.
        public int Count;
    }



    public struct InequalityConstraint1DOF
    {
        public struct IterationData
        {
            //The iteration needs to project from world space to constraint space initially.
            public Vector3Wide LinearJacobianA;
            public Vector3Wide LinearJacobianB;
            public Vector3Wide AngularJacobianA;
            public Vector3Wide AngularJacobianB;

            public Vector<float> EffectiveMass;
            public Vector<float> VelocityBias;
            public Vector<float> Softness;

            //It also needs to project from constraint space to world space.
            //We bundle this with the inertia/mass multiplier, so rather than taking a constraint impulse to world impulse and then to world velocity change,
            //we just go directly from constraint impulse to world velocity change.
            //For constraints with lower DOF counts, using this format also saves us some memory bandwidth- 
            //the inverse inertia tensor and inverse mass for a 2 body constraint cost 20 floats, compared to this implemenation's 12.
            //(Note that even in an implementation where we use the body inertias, we should still cache it constraint-locally to avoid big gathers.)
            public Vector3Wide CSIToWSVLinearA;
            public Vector3Wide CSIToWSVAngularA;
            public Vector3Wide CSIToWSVLinearB;
            public Vector3Wide CSIToWSVAngularB;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep()
        {
            //While gathering the inertia and mass, note that kinematic entities and null connections are treated as having infinite mass.
            //In other words, their inverse mass is zero, and their inverse inertia is the zero matrix.
            //Further, during velocity gather and scatter operations in WarmStart and Solve, all null or kinematic connections are treated as -1. 
        }

        //Naming conventions:
        //We transform between two spaces, world and constraint space. We also deal with two quantities- velocities, and impulses. 
        //And we have some number of entities involved in the constraint. So:
        //wsva: world space velocity of body A
        //wsvb: world space velocity of body B
        //csv: constraint space velocity- when the body velocities are projected into constraint space and combined with the velocity biases, the result is a single constraint velocity error
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
            //(jacobian goes from world to constraint space, transposed jacobian goes from constraint to world space).
            //That world space impulse is then converted to a corrective velocity change by scaling the impulse by the inverse mass/inertia.
            //As an optimization for constraints with smaller jacobians, the transpose(jacobian) * (inertia or mass) transform is precomputed.
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Multiply(ref correctiveImpulse, ref data.CSIToWSVLinearA, out correctiveVelocityA.LinearVelocity);
            Vector3Wide.Multiply(ref correctiveImpulse, ref data.CSIToWSVAngularA, out correctiveVelocityA.AngularVelocity);
            Vector3Wide.Multiply(ref correctiveImpulse, ref data.CSIToWSVLinearB, out correctiveVelocityB.LinearVelocity);
            Vector3Wide.Multiply(ref correctiveImpulse, ref data.CSIToWSVAngularB, out correctiveVelocityB.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.LinearVelocity, ref wsvA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.AngularVelocity, ref wsvA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.LinearVelocity, ref wsvB.LinearVelocity, out wsvB.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.AngularVelocity, ref wsvB.AngularVelocity, out wsvB.AngularVelocity);
            GatherScatter.ScatterVelocities(velocities, ref references, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(BodyVelocities[] velocities, ref BodyReferences references, ref IterationData data, ref Vector<float> accumulatedImpulse)
        {
            var wsvA = new BodyVelocities();
            var wsvB = new BodyVelocities();
            GatherScatter.GatherVelocities(velocities, ref references, ref wsvA, ref wsvB);
            ApplyImpulse(velocities, ref references, ref data, ref wsvA, ref wsvB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref IterationData data, ref Vector<float> accumulatedImpulse,
            out Vector<float> correctiveCSI)
        {
            //Take the world space velocity of each body into constraint space by transforming by the jacobian (a column vector, as compared to our linear velocity row vectors).
            Vector3Wide.Dot(ref wsvA.LinearVelocity, ref data.LinearJacobianA, out var csvaLinear);
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref data.AngularJacobianA, out var csvaAngular);
            Vector3Wide.Dot(ref wsvB.LinearVelocity, ref data.LinearJacobianB, out var csvbLinear);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref data.AngularJacobianB, out var csvbAngular);
            //Note that we did not premultiply the jacobians with the effective mass because of softness. While all the other components could have been pretransformed,
            //pretransforming softness would have just resulted in a effectivemass-sized matrix multiply, i.e. CreateScale(data.Softness) * EffectiveMass.
            //No computational savings, and worse cache behavior due to a second effectivemass-sized matrix.
            var csv = (csvaLinear + csvaAngular) + (csvbLinear + csvbAngular) + (data.Softness * accumulatedImpulse + data.VelocityBias);

            //Convert constraint space velocity to impulses.
            //For 1DOF constraints, the effective mass is just a scalar.
            var csi = csv * data.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + csi);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(BodyVelocities[] velocities, ref BodyReferences references, ref IterationData data, ref Vector<float> accumulatedImpulse)
        {
            var wsvA = new BodyVelocities();
            var wsvB = new BodyVelocities();
            GatherScatter.GatherVelocities(velocities, ref references, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(velocities, ref references, ref data, ref wsvA, ref wsvB, ref correctiveCSI);

        }

    }
}
