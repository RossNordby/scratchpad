using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace HeadlessTests23.DemoStyle;
struct ClothCollisionFilter
{
    ushort x;
    ushort y;
    int instanceId;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ClothCollisionFilter(int x, int y, int instanceId)
    {
        const int max = 1 << 16;
        Debug.Assert(x >= 0 && x < max && y >= 0 && y < max, "This filter packs local indices, so their range is limited.");
        this.x = (ushort)x;
        this.y = (ushort)y;
        this.instanceId = instanceId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Test(ClothCollisionFilter a, ClothCollisionFilter b, int minimumDistance)
    {
        if (a.instanceId != b.instanceId)
            return true;
        //Disallow collisions between vertices which are near each other. We measure distance as max(abs(ax - bx), abs(ay - by), abs(az - bz)).
        var differenceX = a.x - b.x;
        if (differenceX < -minimumDistance || differenceX > minimumDistance)
            return true;
        var differenceY = a.y - b.y;
        if (differenceY < -minimumDistance || differenceY > minimumDistance)
            return true;
        return false;
    }
}


struct ClothCallbacks : INarrowPhaseCallbacks, Dancers.IDancerNarrowPhaseCallbacks<ClothCallbacks, ClothCollisionFilter> //"IDancerNarrowPhaseCallbacks" just means this is a INarrowPhaseCallbacks usable with the DemoDancers.
{
    public CollidableProperty<ClothCollisionFilter> Filters;
    public PairMaterialProperties Material;
    /// <summary>
    /// Minimum manhattan distance in cloth nodes required for two cloth nodes to collide. Stops adjacent cloth nodes from generating contacts and interfering with clothy behavior.
    /// </summary>
    public int MinimumDistanceForSelfCollisions;

    public ClothCallbacks(CollidableProperty<ClothCollisionFilter> filters, PairMaterialProperties material, int minimumDistanceForSelfCollisions = 3)
    {
        Filters = filters;
        Material = material;
        MinimumDistanceForSelfCollisions = minimumDistanceForSelfCollisions;
    }
    ClothCallbacks Dancers.IDancerNarrowPhaseCallbacks<ClothCallbacks, ClothCollisionFilter>.Create(CollidableProperty<ClothCollisionFilter> filters, PairMaterialProperties pairMaterialProperties, int minimumDistanceForSelfCollisions)
    {
        return new ClothCallbacks(filters, pairMaterialProperties, minimumDistanceForSelfCollisions);
    }
    public ClothCallbacks(CollidableProperty<ClothCollisionFilter> filters, int minimumDistanceForSelfCollisions = 3)
        : this(filters, new PairMaterialProperties { SpringSettings = new SpringSettings(30, 1), FrictionCoefficient = 0.25f, MaximumRecoveryVelocity = 2f }, minimumDistanceForSelfCollisions)
    {
    }

    public void Initialize(Simulation simulation)
    {
        Filters.Initialize(simulation);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
    {
        if (a.Mobility != CollidableMobility.Static && b.Mobility != CollidableMobility.Static)
        {
            return ClothCollisionFilter.Test(Filters[a.BodyHandle], Filters[b.BodyHandle], MinimumDistanceForSelfCollisions);
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