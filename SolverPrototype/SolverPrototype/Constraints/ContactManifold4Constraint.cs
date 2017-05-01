using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BEPUutilities2.Quaternion;
namespace SolverPrototype.Constraints
{

    public struct ManifoldContactDataAOS
    {
        //TODO: Arguably storing this to match the prestep layout would be a better idea for contiguity. Consider it later.
        public Vector3 OffsetA;
        public Vector3 OffsetB;
        public float PenetrationDepth;
    }
    public struct SpringSettingsAOS
    {
        public float NaturalFrequency;
        public float DampingRatio;
        public float MaximumRecoveryVelocity;
    }

    public struct ContactManifold4Constraint : IConstraintDescription<ContactManifold4Constraint>
    {

        //TODO: In a 'real' use case, we will likely split the description for contact manifolds into two parts: mutable contact data and initialize-once spring/friction data.
        //SpringSettings and FrictionCoefficient don't usually change over the lifetime of the constraint, so there's no reason to set them every time.
        //For now, though, we'll use this combined representation.]
        public ManifoldContactDataAOS Contact0;
        public ManifoldContactDataAOS Contact1;
        public ManifoldContactDataAOS Contact2;
        public ManifoldContactDataAOS Contact3;
        public float FrictionCoefficient;
        public Quaternion SurfaceBasis;
        public SpringSettingsAOS SpringSettings;

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
            Debug.Assert(batch is ContactManifold4TypeBatch, "The type batch passed to the description must match the description's expected type.");
            var typedBatch = Unsafe.As<ContactManifold4TypeBatch>(batch);
            ref var lane = ref GatherScatter.Get(ref typedBatch.PrestepData[bundleIndex].OffsetA0.X, innerIndex);
            lane = Contact0.OffsetA.X;
            Unsafe.Add(ref lane, Vector<float>.Count) = Contact0.OffsetA.Y;
            Unsafe.Add(ref lane, 2 * Vector<float>.Count) = Contact0.OffsetA.Z;
            Unsafe.Add(ref lane, 3 * Vector<float>.Count) = Contact1.OffsetA.X;
            Unsafe.Add(ref lane, 4 * Vector<float>.Count) = Contact1.OffsetA.Y;
            Unsafe.Add(ref lane, 5 * Vector<float>.Count) = Contact1.OffsetA.Z;
            Unsafe.Add(ref lane, 6 * Vector<float>.Count) = Contact2.OffsetA.X;
            Unsafe.Add(ref lane, 7 * Vector<float>.Count) = Contact2.OffsetA.Y;
            Unsafe.Add(ref lane, 8 * Vector<float>.Count) = Contact2.OffsetA.Z;
            Unsafe.Add(ref lane, 9 * Vector<float>.Count) = Contact3.OffsetA.X;
            Unsafe.Add(ref lane, 10 * Vector<float>.Count) = Contact3.OffsetA.Y;
            Unsafe.Add(ref lane, 11 * Vector<float>.Count) = Contact3.OffsetA.Z;

            Unsafe.Add(ref lane, 12 * Vector<float>.Count) = Contact0.OffsetB.X;
            Unsafe.Add(ref lane, 13 * Vector<float>.Count) = Contact0.OffsetB.Y;
            Unsafe.Add(ref lane, 14 * Vector<float>.Count) = Contact0.OffsetB.Z;
            Unsafe.Add(ref lane, 15 * Vector<float>.Count) = Contact1.OffsetB.X;
            Unsafe.Add(ref lane, 16 * Vector<float>.Count) = Contact1.OffsetB.Y;
            Unsafe.Add(ref lane, 17 * Vector<float>.Count) = Contact1.OffsetB.Z;
            Unsafe.Add(ref lane, 18 * Vector<float>.Count) = Contact2.OffsetB.X;
            Unsafe.Add(ref lane, 19 * Vector<float>.Count) = Contact2.OffsetB.Y;
            Unsafe.Add(ref lane, 20 * Vector<float>.Count) = Contact2.OffsetB.Z;
            Unsafe.Add(ref lane, 21 * Vector<float>.Count) = Contact3.OffsetB.X;
            Unsafe.Add(ref lane, 22 * Vector<float>.Count) = Contact3.OffsetB.Y;
            Unsafe.Add(ref lane, 23 * Vector<float>.Count) = Contact3.OffsetB.Z;

            Unsafe.Add(ref lane, 24 * Vector<float>.Count) = FrictionCoefficient;

            Unsafe.Add(ref lane, 25 * Vector<float>.Count) = SurfaceBasis.X;
            Unsafe.Add(ref lane, 26 * Vector<float>.Count) = SurfaceBasis.Y;
            Unsafe.Add(ref lane, 27 * Vector<float>.Count) = SurfaceBasis.Z;
            Unsafe.Add(ref lane, 28 * Vector<float>.Count) = SurfaceBasis.W;

            Unsafe.Add(ref lane, 29 * Vector<float>.Count) = SpringSettings.NaturalFrequency;
            Unsafe.Add(ref lane, 30 * Vector<float>.Count) = SpringSettings.DampingRatio;
            Unsafe.Add(ref lane, 31 * Vector<float>.Count) = SpringSettings.MaximumRecoveryVelocity;

            Unsafe.Add(ref lane, 32 * Vector<float>.Count) = Contact0.PenetrationDepth;
            Unsafe.Add(ref lane, 33 * Vector<float>.Count) = Contact1.PenetrationDepth;
            Unsafe.Add(ref lane, 34 * Vector<float>.Count) = Contact2.PenetrationDepth;
            Unsafe.Add(ref lane, 35 * Vector<float>.Count) = Contact3.PenetrationDepth;



        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(TypeBatch batch, int bundleIndex, int innerIndex, out ContactManifold4Constraint description)
        {
            Debug.Assert(batch is ContactManifold4TypeBatch, "The type batch passed to the description must match the description's expected type.");
            var typedBatch = Unsafe.As<ContactManifold4TypeBatch>(batch);
            ref var lane = ref GatherScatter.Get(ref typedBatch.PrestepData[bundleIndex].OffsetA0.X, innerIndex);
            description.Contact0.OffsetA.X = lane;
            description.Contact0.OffsetA.Y = Unsafe.Add(ref lane, Vector<float>.Count);
            description.Contact0.OffsetA.Z = Unsafe.Add(ref lane, 2 * Vector<float>.Count);
            description.Contact1.OffsetA.X = Unsafe.Add(ref lane, 3 * Vector<float>.Count);
            description.Contact1.OffsetA.Y = Unsafe.Add(ref lane, 4 * Vector<float>.Count);
            description.Contact1.OffsetA.Z = Unsafe.Add(ref lane, 5 * Vector<float>.Count);
            description.Contact2.OffsetA.X = Unsafe.Add(ref lane, 6 * Vector<float>.Count);
            description.Contact2.OffsetA.Y = Unsafe.Add(ref lane, 7 * Vector<float>.Count);
            description.Contact2.OffsetA.Z = Unsafe.Add(ref lane, 8 * Vector<float>.Count);
            description.Contact3.OffsetA.X = Unsafe.Add(ref lane, 9 * Vector<float>.Count);
            description.Contact3.OffsetA.Y = Unsafe.Add(ref lane, 10 * Vector<float>.Count);
            description.Contact3.OffsetA.Z = Unsafe.Add(ref lane, 11 * Vector<float>.Count);

            description.Contact0.OffsetB.X = Unsafe.Add(ref lane, 12 * Vector<float>.Count);
            description.Contact0.OffsetB.Y = Unsafe.Add(ref lane, 13 * Vector<float>.Count);
            description.Contact0.OffsetB.Z = Unsafe.Add(ref lane, 14 * Vector<float>.Count);
            description.Contact1.OffsetB.X = Unsafe.Add(ref lane, 15 * Vector<float>.Count);
            description.Contact1.OffsetB.Y = Unsafe.Add(ref lane, 16 * Vector<float>.Count);
            description.Contact1.OffsetB.Z = Unsafe.Add(ref lane, 17 * Vector<float>.Count);
            description.Contact2.OffsetB.X = Unsafe.Add(ref lane, 18 * Vector<float>.Count);
            description.Contact2.OffsetB.Y = Unsafe.Add(ref lane, 19 * Vector<float>.Count);
            description.Contact2.OffsetB.Z = Unsafe.Add(ref lane, 20 * Vector<float>.Count);
            description.Contact3.OffsetB.X = Unsafe.Add(ref lane, 21 * Vector<float>.Count);
            description.Contact3.OffsetB.Y = Unsafe.Add(ref lane, 22 * Vector<float>.Count);
            description.Contact3.OffsetB.Z = Unsafe.Add(ref lane, 23 * Vector<float>.Count);

            description.FrictionCoefficient = Unsafe.Add(ref lane, 24 * Vector<float>.Count);

            description.SurfaceBasis.X = Unsafe.Add(ref lane, 25 * Vector<float>.Count);
            description.SurfaceBasis.Y = Unsafe.Add(ref lane, 26 * Vector<float>.Count);
            description.SurfaceBasis.Z = Unsafe.Add(ref lane, 27 * Vector<float>.Count);
            description.SurfaceBasis.W = Unsafe.Add(ref lane, 28 * Vector<float>.Count);

            description.SpringSettings.NaturalFrequency = Unsafe.Add(ref lane, 29 * Vector<float>.Count);
            description.SpringSettings.DampingRatio = Unsafe.Add(ref lane, 30 * Vector<float>.Count);
            description.SpringSettings.MaximumRecoveryVelocity = Unsafe.Add(ref lane, 31 * Vector<float>.Count);

            description.Contact0.PenetrationDepth = Unsafe.Add(ref lane, 32 * Vector<float>.Count);
            description.Contact1.PenetrationDepth = Unsafe.Add(ref lane, 33 * Vector<float>.Count);
            description.Contact2.PenetrationDepth = Unsafe.Add(ref lane, 34 * Vector<float>.Count);
            description.Contact3.PenetrationDepth = Unsafe.Add(ref lane, 35 * Vector<float>.Count);

        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ConstraintTypeIds.GetId<ContactManifold4TypeBatch>();
            }
        }
    }

}
