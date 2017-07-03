﻿using SolverPrototype;
using SolverPrototype.Collidables;
using System.Numerics;

namespace SolverPrototypeTests
{
    public struct RegularGridWithKinematicBaseBuilder : IBodyBuilder
    {
        public Vector3 Spacing;
        public Vector3 Origin;
        public float InverseInertiaMultiplier;
        public TypedIndex ShapeIndex;
        public RegularGridWithKinematicBaseBuilder(Vector3 spacing, Vector3 origin, float inverseInertiaMultiplier = 0, TypedIndex shapeIndex = new TypedIndex())
        {
            Spacing = spacing;
            Origin = origin;
            InverseInertiaMultiplier = inverseInertiaMultiplier;
            ShapeIndex = shapeIndex;
        }

        public void Build(int columnIndex, int rowIndex, int sliceIndex, out BodyDescription bodyDescription)
        {
            bodyDescription = new BodyDescription
            {
                Pose = new BodyPose
                {
                    Position = new Vector3(columnIndex, rowIndex, sliceIndex) * Spacing + Origin,
                    Orientation = BEPUutilities2.Quaternion.Identity
                },
                LocalInertia = new BodyInertia { InverseMass = rowIndex > 0 ? 1 : 0 },
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings(),
                    SpeculativeMargin = 0.1f,
                    Shape = ShapeIndex
                }
            };

            var inverseInertia = bodyDescription.LocalInertia.InverseMass * InverseInertiaMultiplier;
            bodyDescription.LocalInertia.InverseInertiaTensor.M11 = inverseInertia;
            bodyDescription.LocalInertia.InverseInertiaTensor.M22 = inverseInertia;
            bodyDescription.LocalInertia.InverseInertiaTensor.M33 = inverseInertia;

        }
    }
}

