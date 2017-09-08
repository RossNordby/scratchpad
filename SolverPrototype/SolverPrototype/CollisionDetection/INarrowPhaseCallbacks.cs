﻿using System;
using System.Collections.Generic;
using System.Text;
using BEPUutilities2;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;

namespace SolverPrototype.CollisionDetection
{
    public interface INarrowPhaseCallbacks
    {
        void Initialize<TNarrowPhase>(Simulation<TNarrowPhase> simulation) where TNarrowPhase : NarrowPhase, new();
        /// <summary>
        /// Chooses whether to allow contact generation to proceed for two overlapping collidables.
        /// </summary>
        /// <param name="workerIndex">Index of the worker that identified the overlap.</param>
        /// <param name="a">Reference to the first collidable in the pair.</param>
        /// <param name="b">Reference to the second collidable in the pair.</param>
        /// <returns>True if collision detection should proceed, false otherwise.</returns>
        bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b);
        /// <summary>
        /// Provides a notification that a manifold has been created. Offers an opportunity to change the manifold's details. 
        /// </summary>
        /// <param name="workerIndex">Index of the worker thread that created this manifold.</param>
        /// <param name="pair">Pair of collidables that the manifold was detected between.</param>
        /// <param name="manifold">Set of contacts detected between the collidables.</param>
        /// <param name="pairMaterial">Material properties of the manifold.</param>
        /// <returns>True if a constraint should be created for the manifold, false otherwise.</returns>
        bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ref ContactManifold manifold, out PairMaterialProperties pairMaterial);

        /// <summary>
        /// Chooses whether to allow contact generation to proceed for the children of two overlapping collidables in a compound-including pair.
        /// </summary>
        /// <param name="a">Reference to the first collidable in the pair.</param>
        /// <param name="b">Reference to the second collidable in the pair.</param>
        /// <param name="childIndexA">Index of the child of collidable A in the pair. If collidable A is not compound, then this is always 0.</param>
        /// <param name="childIndexB">Index of the child of collidable B in the pair. If collidable B is not compound, then this is always 0.</param>
        /// <returns>True if collision detection should proceed, false otherwise.</returns>
        /// <remarks>This is called for each sub-overlap in a collidable pair involving compound collidables. If neither collidable in a pair is compound, this will not be called.
        /// For compound-including pairs, if the earlier call to AllowContactGeneration returns false for owning pair, this will not be called.</remarks>
        bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, int childIndexA, int childIndexB);
        /// <summary>
        /// Provides a notification that a manifold has been created between the children of two collidables in a compound-including pair.
        /// Offers an opportunity to change the manifold's details. 
        /// </summary>
        /// <param name="workerIndex">Index of the worker thread that created this manifold.</param>
        /// <param name="pair">Pair of collidables that the manifold was detected between.</param>
        /// <param name="childIndexA">Index of the child of collidable A in the pair. If collidable A is not compound, then this is always 0.</param>
        /// <param name="childIndexB">Index of the child of collidable B in the pair. If collidable B is not compound, then this is always 0.</param>
        /// <param name="manifold">Set of contacts detected between the collidables.</param>
        /// <returns>True if this manifold should be considered for constraint generation, false otherwise.</returns>
        bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ContactManifold manifold);
                
        /// <summary>
        /// Requests that a constraint be added to the solver between two bodies.
        /// Implementers must notify the NarrowPhase.PairCache of the constraint handle using NarrowPhase.PairCache.FillConstraintHandle.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description.</typeparam>
        /// <param name="workerIndex">Index of the worker that generated the constraint description.</param>
        /// <param name="constraintCacheIndex">Index of the constraint cache associated with the constraint in the pair cache.
        /// Used to notify the narrow phase's pair cache of the constraint handle after constraint addition.</param>
        /// <param name="impulses">Impulses associated with the new constraint. Passed back to the narrow phase to complete constraint initialization once the constraint exists.</param>
        /// <param name="bodyHandleA">Handle of the first body in the pair.</param>
        /// <param name="bodyHandleB">Handle of the second body in the pair.</param>
        /// <param name="constraintDescription">Description of the constraint being added to the solver.</param>
        void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>;
        /// <summary>
        /// Requests that a constraint be added to the solver between a body and a static collidable.
        /// Implementers must notify the NarrowPhase.PairCache of the constraint handle using NarrowPhase.PairCache.FillConstraintHandle.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description.</typeparam>
        /// <param name="workerIndex">Index of the worker that generated the constraint description.</param>
        /// <param name="constraintCacheIndex">Index of the constraint cache associated with the constraint in the pair cache.
        /// Used to notify the narrow phase's pair cache of the constraint handle after constraint addition.</param>
        /// <param name="impulses">Impulses associated with the new constraint. Passed back to the narrow phase to complete constraint initialization once the constraint exists.</param>
        /// <param name="bodyHandle">Handle of the body in the pair.</param>
        /// <param name="constraintDescription">Description of the constraint being added to the solver.</param>
        void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandle, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>;
        
        /// <summary>
        /// Performs any post-narrowphase execution tasks.
        /// </summary>
        /// <param name="threadDispatcher">Thread dispatcher for use in the flush.</param>
        void Flush(IThreadDispatcher threadDispatcher);

        /// <summary>
        /// Releases any resources held by the callbacks. Called by the owning narrow phase when it is being disposed.
        /// </summary>
        void Dispose();
    }
}
