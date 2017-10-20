﻿using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace SolverPrototype.Constraints
{
    public struct ContactManifold1OneBodyConstraint : IConstraintDescription<ContactManifold1OneBodyConstraint>
    {
        //TODO: In a 'real' use case, we will likely split the description for contact manifolds into two parts: mutable contact data and initialize-once spring/friction data.
        //SpringSettings and FrictionCoefficient don't usually change over the lifetime of the constraint, so there's no reason to set them every time.
        //For now, though, we'll use this combined representation.
        public ManifoldContactDataAOS Contact0;
        public float FrictionCoefficient;
        public Vector3 Normal;
        public SpringSettingsAOS SpringSettings;
        public float MaximumRecoveryVelocity;

        public void ApplyDescription(TypeBatch batch, int bundleIndex, int innerIndex)
        {
            //We assume a contiguous block of Vector<T> types, where T is a 32 bit type. It is unlikely that future runtime changes will introduce
            //packing on the fields, since each of them are a Vector<T> in size- which will tend to be 16, 32, or in the future, 64 bytes.
            //That said, relying on non-explicit memory layouts is still a risk.

            //TODO: Note that this is a maintenance nightmare. There's always going to be a bit of maintenance nightmare, but this is pretty much maximizing it.
            //We can only justify this by saying that contact manifolds are highly performance sensitive, but other constraints that don't undergo constant modification
            //should probably use a somewhat less gross option. For example, while it's still a nightmare, aligning the description's memory layout such that it matches a lane
            //(except the lane has a longer stride between elements) would allow a *relatively* clean and reusable helper that simply loops across the lane.
            //At the end of the day, the important thing is that this mapping is kept localized so that not every system needs to be aware of it.

            //Note that we use an unsafe cast.
            Debug.Assert(batch is ContactManifold1OneBodyTypeBatch, "The type batch passed to the description must match the description's expected type.");
            var typedBatch = Unsafe.As<ContactManifold1OneBodyTypeBatch>(batch);
            ref var lane = ref GatherScatter.Get(ref typedBatch.PrestepData[bundleIndex].OffsetA0.X, innerIndex);
            lane = Contact0.OffsetA.X;
            Unsafe.Add(ref lane, Vector<float>.Count) = Contact0.OffsetA.Y;
            Unsafe.Add(ref lane, 2 * Vector<float>.Count) = Contact0.OffsetA.Z;
            
            Unsafe.Add(ref lane, 3 * Vector<float>.Count) = FrictionCoefficient;

            Unsafe.Add(ref lane, 4 * Vector<float>.Count) = Normal.X;
            Unsafe.Add(ref lane, 5 * Vector<float>.Count) = Normal.Y;
            Unsafe.Add(ref lane, 6 * Vector<float>.Count) = Normal.Z;

            Unsafe.Add(ref lane, 7 * Vector<float>.Count) = SpringSettings.NaturalFrequency;
            Unsafe.Add(ref lane, 8 * Vector<float>.Count) = SpringSettings.DampingRatio;
            Unsafe.Add(ref lane, 9 * Vector<float>.Count) = MaximumRecoveryVelocity;

            Unsafe.Add(ref lane, 10 * Vector<float>.Count) = Contact0.PenetrationDepth;



        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(TypeBatch batch, int bundleIndex, int innerIndex, out ContactManifold1OneBodyConstraint description)
        {
            Debug.Assert(batch is ContactManifold1OneBodyTypeBatch, "The type batch passed to the description must match the description's expected type.");
            var typedBatch = Unsafe.As<ContactManifold1OneBodyTypeBatch>(batch);
            ref var lane = ref GatherScatter.Get(ref typedBatch.PrestepData[bundleIndex].OffsetA0.X, innerIndex);
            description.Contact0.OffsetA.X = lane;
            description.Contact0.OffsetA.Y = Unsafe.Add(ref lane, Vector<float>.Count);
            description.Contact0.OffsetA.Z = Unsafe.Add(ref lane, 2 * Vector<float>.Count);
            
            description.FrictionCoefficient = Unsafe.Add(ref lane, 3 * Vector<float>.Count);

            description.Normal.X = Unsafe.Add(ref lane, 4 * Vector<float>.Count);
            description.Normal.Y = Unsafe.Add(ref lane, 5 * Vector<float>.Count);
            description.Normal.Z = Unsafe.Add(ref lane, 6 * Vector<float>.Count);

            description.SpringSettings.NaturalFrequency = Unsafe.Add(ref lane, 7 * Vector<float>.Count);
            description.SpringSettings.DampingRatio = Unsafe.Add(ref lane, 8 * Vector<float>.Count);
            description.MaximumRecoveryVelocity = Unsafe.Add(ref lane, 9 * Vector<float>.Count);

            description.Contact0.PenetrationDepth = Unsafe.Add(ref lane, 10 * Vector<float>.Count);

        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ContactManifold1OneBodyTypeBatch.BatchTypeId;
            }
        }

        public Type BatchType => typeof(ContactManifold1OneBodyTypeBatch);
    }

}
