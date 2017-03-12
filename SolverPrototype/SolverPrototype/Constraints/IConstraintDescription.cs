﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype.Constraints
{
    /// <summary>
    /// Marks a type as a description of a constraint associated with a particular batch.
    /// </summary>
    /// <remarks>
    /// Note that one batch may have multiple description types associated with it, each one potentially offering a different subset of properties or translation logic.
    /// </remarks>
    /// <typeparam name="TDescription">Type of the description object.</typeparam>
    /// <typeparam name="TBatch">Batch associated with the constraint description.</typeparam>
    public interface IConstraintDescription<TDescription, TBatch> where TBatch : TypeBatch where TDescription : IConstraintDescription<TDescription, TBatch>
    {
        /// <summary>
        /// Creates a description from the batch-held memory at a given location.
        /// </summary>
        /// <param name="batch">Batch to read.</param>
        /// <param name="bundleIndex">Index of the source constraint's bundle.</param>
        /// <param name="innerIndex">Index of the source constraint within its bundle.</param>
        /// <param name="description">Description of the constraint.</param>
        void BuildDescription(TBatch batch, int bundleIndex, int innerIndex, out TDescription description);
        /// <summary>
        /// Changes the batch-held memory at a given location to match the given description.
        /// </summary>
        /// <param name="batch">Batch to modify.</param>
        /// <param name="bundleIndex">Index of the target constraint's bundle.</param>
        /// <param name="innerIndex">Index of the target constraint within its bundle.</param>
        /// <param name="description">Description of the constraint to apply.</param>
        void ApplyDescription(TBatch batch, int bundleIndex, int innerIndex, ref TDescription description);
    }
}
