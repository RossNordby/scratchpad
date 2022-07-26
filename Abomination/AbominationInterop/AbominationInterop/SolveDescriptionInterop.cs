using System.Runtime.InteropServices;

namespace AbominationInterop;

[StructLayout(LayoutKind.Explicit)]
public unsafe struct SolveDescriptionInterop
{
    /// <summary>
    /// Number of velocity iterations to use in the solver if there is no <see cref="VelocityIterationScheduler"/> or if it returns a non-positive value for a substep.
    /// </summary>
    [FieldOffset(0)]
    public int VelocityIterationCount;
    /// <summary>
    /// Number of substeps to execute each time the solver runs.
    /// </summary>
    [FieldOffset(4)]
    public int SubstepCount;
    /// <summary>
    /// Number of synchronzed constraint batches to use before using a fallback approach.
    /// </summary>
    [FieldOffset(8)]
    public int FallbackBatchThreshold;
    /// <summary>
    /// Callback executed to determine how many velocity iterations should be used for a given substep. If null, or if it returns a non-positive value, the <see cref="VelocityIterationCount"/> will be used instead.
    /// </summary>
    [FieldOffset(16)]
    public delegate*<int, int> VelocityIterationScheduler;
}
