using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Collidables
{


    /// <summary>
    /// Defines a type that acts as a source of data needed for bounding box calculations.
    /// </summary>
    /// <remarks>
    /// Collidables may be pulled from objects directly in the world or from compound children. Compound children have to pull and compute information from the parent compound,
    /// and the result of the calculation has to be pushed back to the compound parent for further processing. In contrast, body collidables that live natively in the space simply
    /// gather data directly from the bodies set and scatter bounds directly into the broad phase.
    /// </remarks>
    public interface ICollidableBundleSource
    {
        /// <summary>
        /// Gets the number of collidables in this set of bundles.
        /// </summary>
        int Count { get; }
        /// <summary>
        /// Gathers collidable data required to calculate the bounding boxes for a bundle.
        /// </summary>
        /// <param name="collidablesStartIndex">Start index of the bundle in the collidables set to gather bounding box relevant data for.</param>
        void GatherCollidableBundle(int collidablesStartIndex, out Vector<int> shapeIndices, out Vector<float> maximumExpansion,
            out BodyPoses poses, out BodyVelocities velocities);
        /// <summary>
        /// Scatters the calculated bounds into the target memory locations.
        /// </summary>
        void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int collidablesStartIndex);
    }

    public struct BodyBundleSource : ICollidableBundleSource
    {
        public Bodies Bodies;
        public QuickList<int, Buffer<int>> BodyIndices;
        public int Count
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return BodyIndices.Count; }
        }

        public void GatherCollidableBundle(int collidablesStartIndex, out Vector<int> shapeIndices, out Vector<float> maximumExpansion, out BodyPoses poses, out BodyVelocities velocities)
        {
            var count = BodyIndices.Count - collidablesStartIndex;
            if (count > Vector<float>.Count)
                count = Vector<float>.Count;
            Bodies.GatherDataForBounds(ref BodyIndices[collidablesStartIndex], count, out poses, out velocities, out shapeIndices, out maximumExpansion);
        }

        public void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int collidablesStartIndex)
        {
            for (int i = collidablesStartIndex; i < Vector<float>.Count; ++i)
            {
                //TODO: body bundle updates scatter to the broad phase.
                //ref var startIndex = ref BroadPhase.GetBoundsReference(Bodies.Collidables[BodyIndices[i]].BroadPhaseIndex);
            }
        }
    }
}
