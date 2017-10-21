using SolverPrototype.Constraints;
using SolverPrototype.CollisionDetection;
using SolverPrototype.CollisionDetection.CollisionTasks;
using SolverPrototype.Constraints.Contact;

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
            typeBatchAllocation.Register<Contact1OneBody>();
            typeBatchAllocation.Register<Contact1>();
            typeBatchAllocation.Register<Contact4>();

            defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new SpherePairCollisionTask());
        }
    }
}
