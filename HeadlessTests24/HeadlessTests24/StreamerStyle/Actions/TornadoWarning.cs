using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

public class TornadoWarning : IAction
{
    public struct Vortex
    {
        public Vector2 Location;
        public Vector3 LinearVelocity;
        public Vector3 Axis;
        public float BottomInnerRadius;
        public float TopInnerRadius;
        public float Height;
        public float InverseHeight;

        public float OuterRadiusMultiplier;
        public float InnerTangentVelocity;
        public float UpwardVelocity;
        public bool Clockwise;

        public Vector3 GetVelocityForOffset(bool opposingRotation, Vector3 offsetFromBase, out float influence, out Vector3 horizontalDirection, out float axisDistance, out float innerRadiusAtHeight)
        {
            var heightAlongAxis = Vector3.Dot(offsetFromBase, Axis);
            var topWeight = MathF.Max(0, MathF.Min(1, heightAlongAxis * InverseHeight));
            innerRadiusAtHeight = topWeight * TopInnerRadius + (1 - topWeight) * BottomInnerRadius;
            var horizontalOffset = offsetFromBase - heightAlongAxis * Axis;
            axisDistance = horizontalOffset.Length();
            horizontalDirection = axisDistance > 0 ? horizontalOffset / axisDistance : default;

            var outerRadiusAtHeight = innerRadiusAtHeight * OuterRadiusMultiplier;
            influence = 1f - MathF.Max(0, MathF.Min(1, (axisDistance - innerRadiusAtHeight) / (outerRadiusAtHeight - innerRadiusAtHeight)));
            influence *= influence * (1 - 0.75f * topWeight);

            return (Vector3.Cross(Axis, horizontalDirection) * InnerTangentVelocity * (opposingRotation ? -1f : 1f) + Axis * UpwardVelocity + LinearVelocity);
        }
        public struct Enumerator : IBreakableForEach<CollidableReference>
        {
            public Simulation Simulation;
            public Vortex Vortex;
            public float Dt;
            public bool LoopBody(CollidableReference i)
            {
                if (i.Mobility == CollidableMobility.Dynamic)
                {
                    var body = Simulation.Bodies.GetBodyReference(i.BodyHandle);
                    var offsetFromBase = body.Pose.Position - new Vector3(Vortex.Location.X, 0, Vortex.Location.Y);
                    var targetVelocity = Vortex.GetVelocityForOffset(Vortex.Clockwise, offsetFromBase, out var influence, out var horizontalDirection, out var axisDistance, out var innerRadiusAtHeight);


                    if (influence > 0 && targetVelocity.LengthSquared() > 1e-5f)
                        body.Awake = true;
                    ref var bodyVelocity = ref body.Velocity.Linear;
                    var velocityOffset = targetVelocity - bodyVelocity;
                    var radiusFraction = axisDistance / innerRadiusAtHeight;
                    var internalCentripitalAccelerationDecay = MathF.Max(0, MathF.Min(1f, radiusFraction));
                    //var centripitalAcceleration = 0.25f * internalCentripitalAccelerationDecay * Vortex.InnerTangentVelocity * Vortex.InnerTangentVelocity / MathF.Max(innerRadiusAtHeight, axisDistance);
                    var targetChange = velocityOffset - horizontalDirection * Vortex.InnerTangentVelocity * 1f * internalCentripitalAccelerationDecay;
                    var magnitudeSquared = targetChange.LengthSquared();
                    const float maximumAcceleration = 50;
                    float maximumAccelerationPerTimestep = influence * maximumAcceleration * Dt;
                    targetChange *= influence * Dt;
                    //if (magnitudeSquared > maximumAccelerationPerTimestep * maximumAccelerationPerTimestep)
                    //{
                    //    targetChange = targetChange * (maximumAccelerationPerTimestep / MathF.Sqrt(magnitudeSquared));
                    //}
                    bodyVelocity += targetChange;
                }
                return true;
            }
        }

        public void Update(Simulation simulation, float dt)
        {
            var top = new Cylinder(BottomInnerRadius * OuterRadiusMultiplier, 0);
            var bottom = new Cylinder(TopInnerRadius * OuterRadiusMultiplier, 0);
            QuaternionEx.GetQuaternionBetweenNormalizedVectors(Vector3.UnitY, Axis, out var orientation);
            bottom.ComputeBounds(orientation, out var bottomMin, out var bottomMax);
            top.ComputeBounds(orientation, out var topMin, out var topMax);
            var topOffset = Axis * Height;
            topMin += topOffset;
            topMax += topOffset;
            BoundingBox bounds;
            BoundingBox.CreateMerged(bottomMin, bottomMax, topMin, topMax, out bounds.Min, out bounds.Max);
            var baseLocation = new Vector3(Location.X, 0, Location.Y);
            bounds.Min += baseLocation;
            bounds.Max += baseLocation;
            Enumerator enumerator;
            enumerator.Simulation = simulation;
            enumerator.Vortex = this;
            enumerator.Dt = dt;
            simulation.BroadPhase.GetOverlaps(bounds, ref enumerator);

        }
    }

    public struct Subvortex
    {
        public Vortex Vortex;
        public float Distance;

        public void Update(Simulation simulation, ref Vortex parent, float dt)
        {
            //Integrate the position of the vortex, but maintain the distance by normalizing the offset.
            var offsetFromParent = Vortex.Location - parent.Location;
            var velocity = parent.GetVelocityForOffset(parent.Clockwise, new Vector3(offsetFromParent.X, 0, offsetFromParent.Y), out _, out _, out _, out _);
            Vortex.Location += new Vector2(velocity.X, velocity.Z) * dt;
            var newOffset = Vortex.Location - parent.Location;
            var newOffsetDistance = newOffset.Length();
            newOffset = newOffsetDistance > 0 ? newOffset * (Distance / newOffsetDistance) : new Vector2(Distance, 0);
            Vortex.Location = parent.Location + newOffset;

            Vortex.LinearVelocity = new Vector3(velocity.X, 0, velocity.Z);

            Vortex.Update(simulation, dt);
        }
    }
    public struct Tornado
    {
        public Vortex Vortex;
        public Buffer<Subvortex> Subvortices;

        public Tornado(int subvortexCount, BufferPool pool)
        {
            Vortex = default;
            if (subvortexCount > 0)
                pool.Take(subvortexCount, out Subvortices);
            else
                Subvortices = default;
        }

        public void Update(Simulation simulation, float dt)
        {
            Vortex.Update(simulation, dt);
            for (int i = 0; i < Subvortices.Length; ++i)
            {
                Subvortices[i].Update(simulation, ref Vortex, dt);
            }
            Vortex.Location += new Vector2(Vortex.LinearVelocity.X, Vortex.LinearVelocity.Z) * dt;
        }

        public void Dispose(BufferPool pool)
        {
            if (Subvortices.Allocated)
                pool.Return(ref Subvortices);
        }
    }

    Buffer<Tornado> tornadoes;

    float targetTime;

    public void Initialize(Random random, Scene scene)
    {
        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        var horizontalSpan = new Vector2(span.X, span.Z);
        var horizontalSpanLength = horizontalSpan.Length();

        var maxTornadoCount = (int)MathF.Max(1f, MathF.Min(3, horizontalSpanLength / 128));
        var tornadoCount = random.Next(1, maxTornadoCount + 1);

        var maximumTornadoRadius = horizontalSpanLength * 0.25f / tornadoCount;

        var spawnMin = scene.RegionOfInterest.Min - new Vector3(maximumTornadoRadius * 2 + 50);
        var spawnMax = scene.RegionOfInterest.Max + new Vector3(maximumTornadoRadius * 2 + 50);
        var spawnExclusionMin = scene.RegionOfInterest.Min - new Vector3(maximumTornadoRadius * 2);
        var spawnExclusionMax = scene.RegionOfInterest.Max + new Vector3(maximumTornadoRadius * 2);
        var spawnSpan = spawnMax - spawnMin;

        var longestTime = 0f;

        scene.BufferPool.Take(tornadoCount, out tornadoes);
        for (int i = 0; i < tornadoCount; ++i)
        {
            ref var tornado = ref tornadoes[i];
            var radius = (float)(maximumTornadoRadius * (0.5 + 0.5 * random.NextDouble()));

            Vector3 start;
            Vector3 target;
            int attemptCount = 0;
            bool blocked;
            do
            {
                target = scene.RegionOfInterest.Min * new Vector3(1, 0, 1) + span * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start = spawnMin + spawnSpan * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start.Y = 2;
                blocked = false;

                ++attemptCount;
                if (Vector3.Max(spawnExclusionMin, Vector3.Min(spawnExclusionMax, start)) == start)
                    blocked = true;
                else
                {
                    for (int j = 0; j < i; ++j)
                    {
                        if (Vector2.DistanceSquared(tornadoes[j].Vortex.Location, new Vector2(start.X, start.Z)) < radius * radius)
                        {
                            blocked = true;
                            break;
                        }
                    }
                }

            } while (blocked && attemptCount < 1000);
            if (blocked)
            {
                //Couldn't create all the tornadoes we want.
                tornadoCount = i;
                tornadoes = tornadoes.Slice(tornadoCount);
                break;
            }

            var maxSubvortexCount = (int)Math.Floor(radius / 16);
            tornado = new Tornado(random.Next(0, maxSubvortexCount + 1), scene.BufferPool);
            var movementDirection = Vector3.Normalize(target - start);
            var speedRand = random.NextDouble();
            var speed = MathF.Max(8, (float)((horizontalSpanLength / 10) * (1 - speedRand) + (horizontalSpanLength / 16) * speedRand));
            tornado.Vortex.Location = new Vector2(start.X, start.Z);
            tornado.Vortex.LinearVelocity = movementDirection * speed;
            tornado.Vortex.Axis = Vector3.Normalize(Vector3.UnitY + movementDirection * (float)random.NextDouble() * 0.5f);
            tornado.Vortex.TopInnerRadius = radius * (1f + 1f * (float)random.NextDouble());
            tornado.Vortex.BottomInnerRadius = radius;
            tornado.Vortex.Height = MathF.Max(150, span.Y * 1.5f);
            tornado.Vortex.InverseHeight = 1f / tornado.Vortex.Height;
            //F0 to F5, ish.
            tornado.Vortex.InnerTangentVelocity = 33 + 70 * (float)random.NextDouble();
            tornado.Vortex.UpwardVelocity = tornado.Vortex.InnerTangentVelocity * (0.05f + 0.05f * (float)random.NextDouble());
            tornado.Vortex.OuterRadiusMultiplier = 3.5f;
            tornado.Vortex.Clockwise = random.NextDouble() > 0.1;

            for (int j = 0; j < tornado.Subvortices.Length; ++j)
            {
                ref var subvortex = ref tornado.Subvortices[j];
                subvortex.Vortex = tornado.Vortex;
                subvortex.Distance = tornado.Vortex.BottomInnerRadius * (0.5f + 0.5f * (float)random.NextDouble() * tornado.Vortex.OuterRadiusMultiplier);
                var subvortexAngle = (float)random.NextDouble() * MathF.PI * 2f;
                subvortex.Vortex.Location = tornado.Vortex.Location + new Vector2(MathF.Cos(subvortexAngle), MathF.Sin(subvortexAngle)) * subvortex.Distance;
                subvortex.Vortex.Height /= 5f;
                subvortex.Vortex.InverseHeight = 1f / subvortex.Vortex.Height;
                subvortex.Vortex.BottomInnerRadius /= tornado.Subvortices.Length * 3;
                subvortex.Vortex.TopInnerRadius /= tornado.Subvortices.Length * 3;
                subvortex.Vortex.UpwardVelocity /= 2f;
                subvortex.Vortex.InnerTangentVelocity /= 3f;
                subvortex.Vortex.Clockwise = !tornado.Vortex.Clockwise;
            }

            float timeToCross = (maximumTornadoRadius * 2 + horizontalSpanLength) / speed;
            if (timeToCross > longestTime)
                longestTime = timeToCross;
        }

        targetTime = 5 + 1f * longestTime;
        var angle1 = (float)random.NextDouble() * MathF.PI * 2;
    }


    public bool Update(Scene scene, Random random, float accumulatedTime, float accumulatedRealTime)
    {
        for (int i = 0; i < tornadoes.Length; ++i)
        {
            tornadoes[i].Update(scene.Simulation, scene.TimestepDuration);
        }
        return accumulatedTime < targetTime;
    }

}

