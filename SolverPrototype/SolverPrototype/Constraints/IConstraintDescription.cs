using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype.Constraints
{
    //would you like some generics with a side of generics

    //It's a bit odd that the builder is separated from the description itself, but it's for a few specific and annoying reasons:
    //1) Interfaces don't define constructors, and
    //2) Interfaces don't define static functions, and
    //3) An instance function requires that the instance be initialized, which may be a relatively slow process for a large description 
    //(and the jit can't really be trusted to elide unnecessary initializations).

    /// <summary>
    /// Marks a type as able to gather information from a type batch into a description.
    /// </summary>
    /// <typeparam name="TDescription">Type of the built description.</typeparam>
    /// <typeparam name="TBatch">Type of the batch that is the source of the description.</typeparam>
    public interface IConstraintDescriptionBuilder<TDescription, TBatch>
        where TBatch : TypeBatch
    {
        /// <summary>
        /// Creates a description from the batch-held memory at a given location.
        /// </summary>
        /// <param name="batch">Batch to read.</param>
        /// <param name="bundleIndex">Index of the source constraint's bundle.</param>
        /// <param name="innerIndex">Index of the source constraint within its bundle.</param>
        /// <param name="description">Description of the constraint.</param>
        void BuildDescription(TBatch batch, int bundleIndex, int innerIndex, out TDescription description);

    }
    /// <summary>
    /// Marks a type as a description of a constraint associated with a particular batch.
    /// </summary>
    /// <remarks>
    /// Note that one batch may have multiple description types associated with it, each one potentially offering a different subset of properties or translation logic.
    /// </remarks>
    /// <typeparam name="TDescription">Type of the description object.</typeparam>
    /// <typeparam name="TBatch">Batch associated with the constraint description.</typeparam>
    public interface IConstraintDescription<TDescription, TBatch>
        where TDescription : IConstraintDescription<TDescription, TBatch>
        where TBatch : TypeBatch
    {
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
