using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Collidables
{
    /// <summary>
    /// Description of a collidable instance living in the broad phase and able to generate collision pairs.
    /// Collidables with a ShapeIndex that points to nothing (a default constructed TypedIndex) do not actually refer to any existing Collidable.
    /// This can be used for a body which needs no collidable representation.
    /// </summary>
    public struct Collidable
    {
        /// <summary>
        /// Index of the shape used by the body.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters.
        /// </summary>
        public int BroadPhaseIndex;
        /// <summary>
        /// Size of the margin around the surface of the shape in which contacts can be generated. These contacts will have negative depth and only contribute if the frame's velocities
        /// would push the shapes of a pair into overlap. This should be positive to avoid jittering. It can also be used as a form of continuous collision detection, but excessively 
        /// high values combined with fast motion may result in visible 'ghost collision' artifacts. 
        /// <para>For continuous collision detection with less chance of ghost collisions, use the dedicated continuous collision detection modes.</para>
        /// </summary>
        public float SpeculativeMargin;

        /// <summary>
        /// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
        /// </summary>
        public ContinuousDetectionSettings Continuity;
        //These CCD settings are bundled together away from the rest of the collidable data for a few reasons:
        //1) They do a little packing to avoid pointless memory overhead,
        //2) It's possible that we'll want to split them out later if data access patterns suggest that it's a good idea,
        //3) Don't really want to pollute this structure's members with CCD-conditional tuning variables.
    }


    public interface IBundleBounder
    {
        void Initialize(ShapeBatch batch);
        //Note that this requires the bundle bounder to have access to the type-specific shape data; a typed shape collection is not passed in.
        //This helps limit type exposure and generics explosion.
        void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max);
    }

    public interface IBoundsScatterer
    {
        void Scatter(ref Vector3Wide min, ref Vector3Wide max, int index);
    }


    public static class BodyCollidableBatch
    {


        public static void UpdateBodyBoundingBoxes<TBundleBounder>(ref QuickList<int, Buffer<int>> bodyIndices, Bodies bodies,
            ref TBundleBounder bundleBounder, float dt) where TBundleBounder : IBundleBounder
        {
            for (int i = 0; i < bodyIndices.Count; i += Vector<float>.Count)
            {
                var count = bodyIndices.Count - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;
                bodies.GatherDataForBounds(ref bodyIndices[i], count, out var poses, out var velocities, out var shapeIndices, out var maximumExpansion);


                //The bundle bounder is responsible for gathering shapes from type specific sources. Since it has type knowledge, it is able to both
                //gather the necessary shape information and call the appropriate bounding box calculator.
                bundleBounder.GetBounds(ref shapeIndices, ref poses, out var maximumRadius, out var min, out var max);
                BoundingBoxUpdater.ExpandBoundingBoxes(ref min, ref max, ref velocities, dt, ref maximumRadius, ref maximumExpansion);

                //The bounding boxes are now fully expanded. Scatter them to the target location. For raw primitives in the broad phase, this means sticking them
                //in the broad phase's acceleration structure. For compound children, we stick them in the waiting compound temporary slots.
                //To avoid branching or virtual indirections, we once again abuse generics to supply the scattering logic.
                bundleSource.ScatterBounds(ref min, ref max, i);

            }
        }
    }
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
        QuickList<>
        public int Count => throw new NotImplementedException();

        public void GatherCollidableBundle(int collidablesStartIndex, out Vector<int> shapeIndices, out Vector<float> maximumExpansion, out BodyPoses poses, out BodyVelocities velocities)
        {
            throw new NotImplementedException();
        }

        public void ScatterBounds(ref Vector3Wide min, ref Vector3Wide max, int collidablesStartIndex)
        {
            throw new NotImplementedException();
        }
    }


    struct SphereBundleBounder : IBundleBounder
    {
        ShapeBatch<Sphere> shapes;

        //Using an initialize function gets around generic issues with constructors.
        public void Initialize(ShapeBatch shapes)
        {
            this.shapes = (ShapeBatch<Sphere>)shapes;
        }

        public void GetBounds(ref Vector<int> shapeIndices, ref BodyPoses poses, out Vector<float> maximumRadius, out Vector3Wide min, out Vector3Wide max)
        {
        }
    }





}
