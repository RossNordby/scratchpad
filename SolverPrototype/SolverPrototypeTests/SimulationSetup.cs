using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using SolverPrototype.Constraints;
using System.Collections.Generic;
using System.Numerics;

using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    

    static partial class SimulationSetup<TNarrowPhase, TCollidableData> where TNarrowPhase : NarrowPhase, new() where TCollidableData : struct
    {
        //Most users can get away with just creating a Simulation-derived type that hides the generic implementation details. Here, we're just gonna bite the bullet.
        public interface IBodyBuilder
        {
            void Build(int columnIndex, int rowIndex, int sliceIndex, out BodyDescription<TCollidableData> bodyDescription);
        }
        public interface IConstraintBuilder
        {
            void RegisterConstraintTypes();
            void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex,
                ref BodyDescription<TCollidableData> bodyDescription, ref LatticeBodyGetter ids, ref ConstraintAdder constraintAdder);
        }
        public struct LatticeBodyGetter
        {
            int width, height, length;
            int[] bodyHandles;
            Bodies<TCollidableData> bodies;
            public LatticeBodyGetter(int width, int height, int length, int[] bodyHandles, Bodies<TCollidableData> bodies)
            {
                this.width = width;
                this.height = height;
                this.length = length;
                this.bodyHandles = bodyHandles;
                this.bodies = bodies;
            }
            public bool TryGetId(int columnIndex, int rowIndex, int sliceIndex, out int id)
            {
                if (columnIndex < 0 || columnIndex >= width || rowIndex < 0 || rowIndex >= height || sliceIndex < 0 || sliceIndex >= length)
                {
                    id = -1;
                    return false;
                }
                id = sliceIndex * (height * width) + rowIndex * width + columnIndex;
                return true;
            }
            public bool GetBody(int columnIndex, int rowIndex, int sliceIndex, out int handle, out BodyDescription<TCollidableData> bodyDescription)
            {
                if (!TryGetId(columnIndex, rowIndex, sliceIndex, out var id))
                {
                    handle = -1;
                    bodyDescription = new BodyDescription<TCollidableData>();
                    return false;
                }
                handle = bodyHandles[id];
                bodies.GetDescription(handle, out bodyDescription);
                return true;
            }
        }

        public struct ConstraintAdder
        {
            public int LocalBodyHandle;
            Simulation<TNarrowPhase, TCollidableData> simulation;
            public List<int> ConstraintHandles;
            public ConstraintAdder(Simulation<TNarrowPhase, TCollidableData> simulation, List<int> constraintHandles)
            {
                this.simulation = simulation;
                this.ConstraintHandles = constraintHandles;
                LocalBodyHandle = 0;
            }

            public void Add<T>(ref T description, int otherBodyHandle) where T : IConstraintDescription<T>
            {
                var constraintHandle = simulation.Add(LocalBodyHandle, otherBodyHandle, ref description);
                ConstraintHandles.Add(constraintHandle);
            }
        }

        public static void BuildBasis(ref BodyPose a, ref BodyPose b, out Vector3 x, out Vector3 y, out Vector3 z)
        {
            y = Vector3.Normalize(a.Position - b.Position);
            Quaternion.TransformZ(1, ref a.Orientation, out var ax);
            x = Vector3.Cross(ax, y);
            var xLength = x.Length();
            if (xLength > 1e-7)
            {
                x /= xLength;
            }
            else
            {
                Quaternion.TransformX(1, ref a.Orientation, out var az);
                x = Vector3.Normalize(Vector3.Cross(az, y));
            }
            z = Vector3.Cross(x, y);
        }

        public static void BuildLattice<TBodyBuilder, TConstraintBuilder>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder, int width, int height, int length, Simulation<TNarrowPhase, TCollidableData> simulation,
            out int[] bodyHandles, out int[] constraintHandles) where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder
        {
            var bodyCount = width * height * length;
            bodyHandles = new int[bodyCount];

            var bodyGetter = new LatticeBodyGetter(width, height, length, bodyHandles, simulation.Bodies);

            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        bodyBuilder.Build(columnIndex, rowIndex, sliceIndex, out var bodyDescription);
                        bodyGetter.TryGetId(columnIndex, rowIndex, sliceIndex, out var id);
                        bodyHandles[id] = simulation.Add(ref bodyDescription);
                    }
                }
            }

            constraintBuilder.RegisterConstraintTypes();
            var constraintAdder = new ConstraintAdder(simulation, new List<int>(width * height * length * 3));
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                //The bottom rows are all kinematic, so don't create connections between them.
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        bodyGetter.GetBody(columnIndex, rowIndex, sliceIndex, out constraintAdder.LocalBodyHandle, out var bodyDescription);
                        constraintBuilder.BuildConstraintsForBody(sliceIndex, rowIndex, columnIndex, ref bodyDescription, ref bodyGetter, ref constraintAdder);
                    }
                }
            }
            constraintHandles = constraintAdder.ConstraintHandles.ToArray();


        }


        public static void BuildLattice<TBodyBuilder, TConstraintBuilder>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder, int width, int height, int length, 
            out Simulation<TNarrowPhase, TCollidableData> simulation,
            out int[] bodyHandles, out int[] constraintHandles) where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder
        {
            var bodyCount = width * height * length;
            simulation = new Simulation<TNarrowPhase, TCollidableData>(
                new BufferPool(),
                new SimulationAllocationSizes
                {
                    Bodies = bodyCount,
                    ShapesPerType = 128,
                    CollidablesPerType = bodyCount,
                    Constraints = bodyCount * 3,
                    ConstraintsPerTypeBatch = (bodyCount * 3) / 6,
                    ConstraintCountPerBodyEstimate = 6
                });
            BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);

        }




    }
}

