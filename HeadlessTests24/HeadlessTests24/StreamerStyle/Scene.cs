using BepuUtilities.Memory;
using BepuPhysics;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle;
public abstract class Scene : IDisposable
{
    /// <summary>
    /// Gets the simulation created by the demo's Initialize call.
    /// </summary>
    public Simulation Simulation { get; protected set; }

    //Note that the buffer pool used by the simulation is not considered to be *owned* by the simulation. The simulation merely uses the pool.
    //Disposing the simulation will not dispose or clear the buffer pool.
    /// <summary>
    /// Gets the buffer pool used by the demo's simulation.
    /// </summary>
    public BufferPool BufferPool { get; private set; }

    /// <summary>
    /// Gets the thread dispatcher available for use by the simulation.
    /// </summary>
    public ThreadDispatcher ThreadDispatcher { get; private set; }

    public const int ThreadCount = 32;

    public abstract float TimestepDuration { get; }

    public abstract Vector3 Gravity { get; }

    public Scene()
    {
        BufferPool = new BufferPool();
        ThreadDispatcher = new ThreadDispatcher(ThreadCount);
    }


    public abstract void Initialize(Random random);

    public BoundingBox RegionOfInterest { get; protected set; }
    protected void CreateRegionOfInterest()
    {
        var boundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
        for (int setIndex = 0; setIndex < Simulation.Bodies.Sets.Length; ++setIndex)
        {
            ref var set = ref Simulation.Bodies.Sets[setIndex];
            if (set.Allocated)
            {
                for (int i = 0; i < set.Count; ++i)
                {
                    BoundingBox.CreateMerged(boundingBox, Simulation.Bodies.GetBodyReference(set.IndexToHandle[i]).BoundingBox, out boundingBox);
                }
            }
        }
        if (boundingBox.Min == new Vector3(float.MaxValue) || boundingBox.Max == new Vector3(float.MinValue))
        {
            boundingBox = default;
        }
        RegionOfInterest = boundingBox;
    }

    public abstract void Update();

    bool disposed;
    public void Dispose()
    {
        if (!disposed)
        {
            disposed = true;
            Simulation.Dispose();
            BufferPool.Clear();
            ThreadDispatcher.Dispose();
        }
    }

}
