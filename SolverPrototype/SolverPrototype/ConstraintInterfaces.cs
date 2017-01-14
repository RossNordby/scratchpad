using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    public interface IConstraintDescription
    {
        int ConstraintTypeId { get; }

    }
    public interface ITwoBodyConstraintDescription : IConstraintDescription
    {
        int BodyHandleA { get; }
        int BodyHandleB { get; }

    }
}
