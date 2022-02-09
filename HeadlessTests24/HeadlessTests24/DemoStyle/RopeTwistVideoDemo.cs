using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace HeadlessTests24.DemoStyle;
/// <summary>
/// Filter for a body in a rope, used by the <see cref="RopeNarrowPhaseCallbacks"/>.
/// </summary>
struct RopeFilter
{
    public short RopeIndex;
    public short IndexInRope;
}

/// <summary>
/// Narrow phase callbacks that include collision filters designed for ropes. Adjacent bodies in a rope do not collide with each other.
/// </summary>
unsafe struct RopeNarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public CollidableProperty<RopeFilter> Filters;
    public PairMaterialProperties Material;
    public int MinimumDistanceForCollisions;

    public RopeNarrowPhaseCallbacks(CollidableProperty<RopeFilter> filters, PairMaterialProperties contactMaterial, int minimumDistanceForCollisions = 3)
    {
        Filters = filters;
        Material = contactMaterial;
        MinimumDistanceForCollisions = minimumDistanceForCollisions;
    }
    public RopeNarrowPhaseCallbacks(CollidableProperty<RopeFilter> filters, int minimumDistanceForCollisions = 3) : this(filters, new PairMaterialProperties(1, 2, new SpringSettings(30, 1)), minimumDistanceForCollisions)
    {
    }

    public void Initialize(Simulation simulation)
    {
        Filters.Initialize(simulation);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        var aFilter = Filters[a];
        var bFilter = Filters[b];
        return (aFilter.RopeIndex != bFilter.RopeIndex || Math.Abs(aFilter.IndexInRope - bFilter.IndexInRope) > MinimumDistanceForCollisions) && (a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial = Material;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
    }
}

/// <summary>
/// Shows a bundle of ropes being tangled up by spinning weights.
/// </summary>
public class RopeTwistVideoDemo : Demo
{
    public static BodyHandle[] BuildRopeBodies(Simulation simulation, in Vector3 start, int bodyCount, float bodySize, float bodySpacing, float massPerBody, float inverseInertiaScale)
    {
        BodyHandle[] handles = new BodyHandle[bodyCount + 1];
        var ropeShape = new Sphere(bodySize);
        var ropeInertia = ropeShape.ComputeInertia(massPerBody);
        Symmetric3x3.Scale(ropeInertia.InverseInertiaTensor, inverseInertiaScale, out ropeInertia.InverseInertiaTensor);
        var ropeShapeIndex = simulation.Shapes.Add(ropeShape);
        //Build the links.
        var bodyDescription = new BodyDescription
        {
            Activity = .01f,
            Collidable = ropeShapeIndex
        };
        for (int linkIndex = 0; linkIndex < bodyCount + 1; ++linkIndex)
        {
            //Make the uppermost block kinematic to hold up the rest of the chain.
            bodyDescription.LocalInertia = linkIndex == 0 ? new BodyInertia() : ropeInertia;
            bodyDescription.Pose = start - new Vector3(0, linkIndex * (bodySpacing + 2 * bodySize), 0);
            handles[linkIndex] = simulation.Bodies.Add(bodyDescription);
        }

        return handles;
    }

    public unsafe override void Initialize(int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);

        var filters = new CollidableProperty<RopeFilter>();
        Simulation = Simulation.Create(BufferPool,
            new RopeNarrowPhaseCallbacks(filters, new PairMaterialProperties(1.0f, float.MaxValue, new SpringSettings(1200, 1))),
            new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 60));

        for (int twistIndex = 0; twistIndex < 10; ++twistIndex)
        {
            const int ropeCount = 4;
            var startLocation = new Vector3(0 + twistIndex * 30, 30, 0);

            var bigWreckingBall = new Sphere(3);
            //This wrecking ball is much, much heavier.
            var bigWreckingBallInertia = bigWreckingBall.ComputeInertia(10000);
            var bigWreckingBallIndex = Simulation.Shapes.Add(bigWreckingBall);
            const float ropeBodySpacing = -0.1f;
            const float ropeBodyRadius = 0.1f;
            const int ropeBodyCount = 130;
            var wreckingBallPosition = startLocation - new Vector3(0, ropeBodyRadius + (ropeBodyRadius * 2 + ropeBodySpacing) * ropeBodyCount + bigWreckingBall.Radius, 0);
            var description = BodyDescription.CreateDynamic(wreckingBallPosition, bigWreckingBallInertia, bigWreckingBallIndex, 0.01f);
            var wreckingBallBodyHandle = Simulation.Bodies.Add(description);
            var wreckingBallBody = Simulation.Bodies[wreckingBallBodyHandle];
            wreckingBallBody.Velocity.Angular = new Vector3(0, 20, 0);
            filters.Allocate(wreckingBallBodyHandle) = new RopeFilter { RopeIndex = (short)(16384 + twistIndex), IndexInRope = ropeBodyCount };

            for (int ropeIndex = 0; ropeIndex < ropeCount; ++ropeIndex)
            {
                var angle = ropeIndex * MathF.PI * 2 / ropeCount;
                const float ropeDistributionRadius = 1f;
                var horizontalOffset = ropeDistributionRadius * new Vector3(MathF.Sin(angle), 0, MathF.Cos(angle));
                var ropeStartLocation = startLocation + horizontalOffset;

                var springSettings = new SpringSettings(600, 100);
                var bodyHandles = BuildRopeBodies(Simulation, ropeStartLocation, ropeBodyCount, ropeBodyRadius, ropeBodySpacing, 1f, 0);
                for (int i = 0; i < bodyHandles.Length; ++i)
                {
                    filters.Allocate(bodyHandles[i]) = new RopeFilter { RopeIndex = (short)ropeIndex, IndexInRope = (short)i };
                }

                bool TryCreateConstraint(int handleIndexA, int handleIndexB)
                {
                    if (handleIndexA >= bodyHandles.Length || handleIndexB >= bodyHandles.Length)
                        return false;
                    var maximumDistance = Vector3.Distance(
                        new BodyReference(bodyHandles[handleIndexA], Simulation.Bodies).Pose.Position,
                        new BodyReference(bodyHandles[handleIndexB], Simulation.Bodies).Pose.Position);
                    Simulation.Solver.Add(bodyHandles[handleIndexA], bodyHandles[handleIndexB], new DistanceLimit(default, default, .01f, maximumDistance, springSettings));
                    return true;
                }
                const int constraintsPerBody = 1;
                for (int i = 0; i < bodyHandles.Length - 1; ++i)
                {
                    //Note that you could also create constraints which span even more links. For example, connect i and i+1, i+2, i+4, i+8 and i+16 rather than just the nearest bodies.
                    //That tends to make mass ratios less of an issue, but this demo is a worst case stress test.
                    for (int j = 1; j <= constraintsPerBody; ++j)
                    {
                        if (!TryCreateConstraint(i, i + j))
                            break;
                    }
                }

                var wreckingBallConnectionOffset = horizontalOffset + new Vector3(0, bigWreckingBall.Radius, 0);
                var ropeConnectionToBall = wreckingBallBody.Pose.Position + wreckingBallConnectionOffset;
                for (int i = 1; i <= constraintsPerBody; ++i)
                {
                    var targetBodyHandleIndex = bodyHandles.Length - i;
                    if (targetBodyHandleIndex < 0)
                        break;
                    var maximumDistance = Vector3.Distance(
                        new BodyReference(bodyHandles[targetBodyHandleIndex], Simulation.Bodies).Pose.Position,
                        ropeConnectionToBall);
                    Simulation.Solver.Add(bodyHandles[targetBodyHandleIndex], wreckingBallBodyHandle, new DistanceLimit(default, wreckingBallConnectionOffset, 0.01f, maximumDistance, springSettings));
                }

            }
        }

        Console.WriteLine($"body count: {Simulation.Bodies.ActiveSet.Count}");

        Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), Simulation.Shapes.Add(new Box(200, 1, 200))));
    }
}
