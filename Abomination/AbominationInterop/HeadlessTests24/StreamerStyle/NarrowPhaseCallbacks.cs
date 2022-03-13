using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace HeadlessTests24.StreamerStyle;

struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public CollidableProperty<SubgroupCollisionFilter> Filters;
    public CollidableProperty<float> Friction;
    public SpringSettings SpringSettings;
    public void Initialize(Simulation simulation)
    {
        if (SpringSettings.AngularFrequency == 0)
            SpringSettings = new SpringSettings(30, 1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
        if (Filters != null && b.Mobility != CollidableMobility.Static)
        {
            return SubgroupCollisionFilter.AllowCollision(Filters[a.BodyHandle], Filters[b.BodyHandle]);
        }
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        if (Friction != null)
        {
            pairMaterial.FrictionCoefficient = Friction[pair.A.BodyHandle];
            if (pair.B.Mobility != CollidableMobility.Static)
            {
                //If two bodies collide, just average the friction.
                pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + Friction[pair.B.BodyHandle]) * 0.5f;
            }
        }
        else
        {
            pairMaterial.FrictionCoefficient = 2f;
        }
        pairMaterial.MaximumRecoveryVelocity = 10;
        pairMaterial.SpringSettings = SpringSettings;
        return true;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
        if (Filters != null)
            Filters.Dispose();
        if (Friction != null)
            Friction.Dispose();
    }
}
