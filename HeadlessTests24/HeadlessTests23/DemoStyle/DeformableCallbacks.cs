using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace HeadlessTests23.DemoStyle;
struct DeformableCollisionFilter
{
    int localIndices;
    int instanceId;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public DeformableCollisionFilter(int x, int y, int z, int instanceId)
    {
        const int max = 1 << 10;
        Debug.Assert(x >= 0 && x < max && y >= 0 && y < max && z >= 0 && z < max, "This filter packs local indices, so their range is limited.");
        localIndices = x | (y << 10) | (z << 20);
        this.instanceId = instanceId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Test(in DeformableCollisionFilter a, in DeformableCollisionFilter b)
    {
        if (a.instanceId != b.instanceId)
            return true;
        //Disallow collisions between vertices which are near each other. We measure distance as max(abs(ax - bx), abs(ay - by), abs(az - bz)).
        const int minimumDistance = 3;
        const int mask = (1 << 10) - 1;
        var ax = a.localIndices & mask;
        var bx = b.localIndices & mask;
        var differenceX = ax - bx;
        if (differenceX < -minimumDistance || differenceX > minimumDistance)
            return true;
        var ay = (a.localIndices >> 10) & mask;
        var by = (b.localIndices >> 10) & mask;
        var differenceY = ay - by;
        if (differenceY < -minimumDistance || differenceY > minimumDistance)
            return true;
        var az = (a.localIndices >> 20) & mask;
        var bz = (b.localIndices >> 20) & mask;
        var differenceZ = az - bz;
        if (differenceZ < -minimumDistance || differenceZ > minimumDistance)
            return true;
        return false;
    }
}


struct DeformableCallbacks : INarrowPhaseCallbacks, Dancers.IDancerNarrowPhaseCallbacks<DeformableCallbacks, DeformableCollisionFilter> //"IDancerNarrowPhaseCallbacks" just means this is a INarrowPhaseCallbacks usable with the DemoDancers.
{
    public CollidableProperty<DeformableCollisionFilter> Filters;
    public PairMaterialProperties Material;
    /// <summary>
    /// Minimum manhattan distance in cloth nodes required for two cloth nodes to collide. Stops adjacent cloth nodes from generating contacts and interfering with clothy behavior.
    /// </summary>
    public int MinimumDistanceForSelfCollisions;
    public void Initialize(Simulation simulation)
    {
        Filters.Initialize(simulation);
    }

    public DeformableCallbacks(CollidableProperty<DeformableCollisionFilter> filters, PairMaterialProperties material, int minimumDistanceForSelfCollisions = 3)
    {
        Filters = filters;
        Material = material;
        MinimumDistanceForSelfCollisions = minimumDistanceForSelfCollisions;
    }
    public DeformableCallbacks(CollidableProperty<DeformableCollisionFilter> filters, int minimumDistanceForSelfCollisions = 3)
        : this(filters, new PairMaterialProperties { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) }, minimumDistanceForSelfCollisions)
    {
    }
    //This slightly awkward factory is just here for the dancer demos.
    DeformableCallbacks Dancers.IDancerNarrowPhaseCallbacks<DeformableCallbacks, DeformableCollisionFilter>.Create(CollidableProperty<DeformableCollisionFilter> filters, PairMaterialProperties pairMaterialProperties, int minimumDistanceForSelfCollisions)
    {
        return new DeformableCallbacks(filters, pairMaterialProperties, minimumDistanceForSelfCollisions);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
    {
        if (a.Mobility == CollidableMobility.Dynamic && b.Mobility == CollidableMobility.Dynamic)
        {
            return DeformableCollisionFilter.Test(Filters[a.BodyHandle], Filters[b.BodyHandle]);
        }
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : struct, IContactManifold<TManifold>
    {
        pairMaterial = Material;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
        Filters.Dispose();
    }
}