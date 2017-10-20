using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using SolverPrototype.CollisionDetection;
using SolverPrototype.CollisionDetection.CollisionTasks;

namespace SolverPrototype
{
    /// <summary>
    /// Helper class to register the default types of constraints and shapes within a simulation instance.
    /// </summary>
    public static class DefaultTypes
    {
        /// <summary>
        /// Registers the set of shapes constraints that are packaged in the engine.
        /// </summary>
        public static void Register(TypeBatchAllocation typeBatchAllocation, out CollisionTaskRegistry defaultTaskRegistry)
        {
            typeBatchAllocation.Register<BallSocket>();
            typeBatchAllocation.Register<ContactManifold1OneBodyConstraint>();
            typeBatchAllocation.Register<ContactManifold1Constraint>();
            typeBatchAllocation.Register<ContactManifold4Constraint>();

            defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new SpherePairCollisionTask());
        }
    }
}
