using System;
using System.Collections.Generic;
using System.Text;
using SolverPrototype.Constraints;

namespace SolverPrototype.CollisionDetection
{
    public interface INarrowPhaseCallbacks
    {
        bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ref ConvexContactManifold manifold, out PairMaterialProperties pairMaterial);
        void AddConstraint<TDescription>(int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>;
    }
}
