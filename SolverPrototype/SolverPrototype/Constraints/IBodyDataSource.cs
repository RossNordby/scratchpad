using BEPUutilities2.Memory;

namespace SolverPrototype.Constraints
{
    public interface IBodyDataSource
    {
        int BodyCount { get; }
        ref Buffer<int> HandleToIndex { get; }
        ref Buffer<int> IndexToHandle { get; }
        ref Buffer<BodyVelocities> Velocities { get; }
    }
}