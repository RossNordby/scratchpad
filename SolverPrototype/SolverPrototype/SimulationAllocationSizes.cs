﻿namespace SolverPrototype
{
    /// <summary>
    /// The common set of allocation sizes for a simulation.
    /// </summary>
    public struct SimulationAllocationSizes
    {
        /// <summary>
        /// The number of bodies to allocate space for.
        /// </summary>
        public int Bodies;
        /// <summary>
        /// Minimum number of shapes to allocate space for in each shape type batch.
        /// </summary>
        public int ShapesPerType;
        /// <summary>
        /// Minimum number of collidable descriptions to allocate space for in each collidable type batch.
        /// </summary>
        public int CollidablesPerType;
        /// <summary>
        /// The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.
        /// </summary>
        public int Constraints;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each individual type batch.
        /// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.
        /// </summary>
        public int ConstraintsPerTypeBatch;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each body's constraint list.
        /// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// </summary>
        public int ConstraintCountPerBodyEstimate;

       
    }
}
