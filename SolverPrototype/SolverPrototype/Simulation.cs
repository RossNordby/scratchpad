using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    /// <summary>
    /// Orchestrates the bookkeeping and execution of a full dynamic simulation.
    /// </summary>
    public class Simulation
    {
        public ConstraintConnectivityGraph ConstraintGraph { get; private set; }
        public Bodies Bodies { get; private set; }
        public BodyLayoutOptimizer BodyLayoutOptimizer { get; private set; }
        public ConstraintLayoutOptimizer ConstraintLayoutOptimizer { get; private set; }
        public Solver Solver { get; private set; }

        //You might wonder why this calls out 'full featured', since there are no other non-full featured simulations.
        //Later on, we might include such a partial simulation- for example, kinematic only, or even collision detection only.
        //The only real difference is what stages are included (and a few changes in memory layout).
        /// <summary>
        /// Constructs a full featured simulation supporting dynamic movement and constraints.
        /// </summary>
        /// <param name="initialBodyCapacity">Expected number of bodies. If the simulation exceeds this number, a resize of relevant resources will be performed.</param>
        /// <param name="initialConstraintCapacity">Expected number of constraints across all types. If the simulation exceeds this number, a resize of relevant resources will be performed.</param>
        /// <param name="minimumCapacityPerTypeBatch">Minimum number of constraints of one type in a constraint batch.
        /// This can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation.</param>
        /// <param name="initialConstraintCountPerBodyEstimate">Expected number of constraints connected to any one body. If this number is exceeded for a body, a resize of the body's list will be performed, but all affected resources are pooled.</param>
        public Simulation(
            int initialBodyCapacity = 4096, 
            int initialConstraintCapacity = 4096, 
            int minimumCapacityPerTypeBatch = 64, 
            int initialConstraintCountPerBodyEstimate = 8)
        {
            Bodies = new Bodies(initialBodyCapacity);
            Solver = new Solver(Bodies, initialCapacity: initialConstraintCapacity, minimumCapacityPerTypeBatch: minimumCapacityPerTypeBatch);
            ConstraintGraph = new ConstraintConnectivityGraph(Solver, initialBodyCapacity, initialConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, ConstraintGraph, Solver);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
        }

        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <remarks>
        /// Be wary of variable timesteps. They can harm stability. Whenever possible, keep the timestep the same across multiple frames unless you have a specific reason not to.
        /// </remarks>
        /// <param name="dt">Duration of the time step in time.</param>
        public void Timestep(float dt)
        {
            var inverseDt = 1f / dt;
            Solver.Update(dt, inverseDt);
        }
    }
}
