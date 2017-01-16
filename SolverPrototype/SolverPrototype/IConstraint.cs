using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    public interface IConstraint<TPrestepData, TIterationData>
    {
        void Prestep(BodyInertias[] bodyInertias, ref TPrestepData prestepData, ref TIterationData iterationData);
        void WarmStart(BodyVelocities[] bodyVelocities, ref TIterationData data);
        void SolveIteration(BodyVelocities[] bodyVelocities, ref TIterationData data);
    }
}
