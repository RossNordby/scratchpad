using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Memory;

namespace HeadlessTests23.DemoStyle;

public abstract class Demo : IDisposable
{
    /// <summary>
    /// Gets the simulation created by the demo's Initialize call.
    /// </summary>
    public Simulation Simulation { get; protected set; }

    /// <summary>
    /// Gets the buffer pool used by the demo's simulation.
    /// </summary>
    public BufferPool BufferPool { get; private set; }

    /// <summary>
    /// Gets the thread dispatcher available for use by the simulation.
    /// </summary>
    public ThreadDispatcher ThreadDispatcher { get; protected set; }

    protected Demo()
    {
        BufferPool = new BufferPool();
    }

    public abstract void Initialize(int threadCount);

    public const float TimestepDuration = 1 / 60f;
    public virtual void Update()
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
    }

    protected virtual void OnDispose()
    {

    }

    bool disposed;
    public void Dispose()
    {
        if (!disposed)
        {
            disposed = true;
            OnDispose();
            Simulation.Dispose();
            BufferPool.Clear();
            ThreadDispatcher.Dispose();
        }
    }
}

