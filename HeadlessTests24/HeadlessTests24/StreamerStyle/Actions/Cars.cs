using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

struct WheelHandles
{
    public BodyHandle Wheel;
    public ConstraintHandle SuspensionSpring;
    public ConstraintHandle SuspensionTrack;
    public ConstraintHandle Hinge;
    public ConstraintHandle Motor;
}
struct SimpleCar
{
    public BodyHandle Body;
    public WheelHandles FrontLeftWheel;
    public WheelHandles FrontRightWheel;
    public WheelHandles BackLeftWheel;
    public WheelHandles BackRightWheel;

    private Vector3 suspensionDirection;
    private AngularHinge hingeDescription;

    public void Steer(Simulation simulation, in WheelHandles wheel, float angle)
    {
        var steeredHinge = hingeDescription;
        Matrix3x3.CreateFromAxisAngle(suspensionDirection, -angle, out var rotation);
        Matrix3x3.Transform(hingeDescription.LocalHingeAxisA, rotation, out steeredHinge.LocalHingeAxisA);
        simulation.Solver.ApplyDescription(wheel.Hinge, steeredHinge);
    }

    public void SetSpeed(Simulation simulation, in WheelHandles wheel, float speed, float maximumForce)
    {
        simulation.Solver.ApplyDescription(wheel.Motor, new AngularAxisMotor
        {
            LocalAxisA = new Vector3(0, -1, 0),
            Settings = new MotorSettings(maximumForce, 1e-6f),
            TargetVelocity = speed
        });
    }

    public static WheelHandles CreateWheel(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, in RigidPose bodyPose,
        TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, in Vector3 suspensionDirection, float suspensionLength,
        in AngularHinge hingeDescription, in SpringSettings suspensionSettings, in Quaternion localWheelOrientation)
    {
        RigidPose wheelPose;
        RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, bodyPose, out wheelPose.Position);
        QuaternionEx.ConcatenateWithoutOverlap(localWheelOrientation, bodyPose.Orientation, out wheelPose.Orientation);
        WheelHandles handles;
        handles.Wheel = simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, wheelShape, -0.01f));

        handles.SuspensionSpring = simulation.Solver.Add(bodyHandle, handles.Wheel, new LinearAxisServo
        {
            LocalPlaneNormal = suspensionDirection,
            TargetOffset = suspensionLength,
            LocalOffsetA = bodyToWheelSuspension,
            LocalOffsetB = default,
            ServoSettings = ServoSettings.Default,
            SpringSettings = suspensionSettings
        });
        handles.SuspensionTrack = simulation.Solver.Add(bodyHandle, handles.Wheel, new PointOnLineServo
        {
            LocalDirection = suspensionDirection,
            LocalOffsetA = bodyToWheelSuspension,
            LocalOffsetB = default,
            ServoSettings = ServoSettings.Default,
            SpringSettings = new SpringSettings(30, 1)
        });
        //We're treating braking and acceleration as the same thing. It is, after all, a *simple* car! Maybe it's electric or something.
        //It would be fairly easy to split brakes and drive motors into different motors.
        handles.Motor = simulation.Solver.Add(handles.Wheel, bodyHandle, new AngularAxisMotor
        {
            LocalAxisA = new Vector3(0, 1, 0),
            Settings = default,
            TargetVelocity = default
        });
        handles.Hinge = simulation.Solver.Add(bodyHandle, handles.Wheel, hingeDescription);
        //The demos SubgroupCollisionFilter is pretty simple and only tests one direction, so we make the non-colliding relationship symmetric.
        ref var filter = ref filters.Allocate(handles.Wheel);
        ref var friction = ref frictions.Allocate(handles.Wheel);
        filter = new SubgroupCollisionFilter(bodyHandle.Value, 1);
        friction = wheelFriction;
        SubgroupCollisionFilter.DisableCollision(ref filter, ref bodyFilter);
        return handles;
    }

    public static SimpleCar Create(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, in RigidPose pose,
        TypedIndex bodyShape, BodyInertia bodyInertia, float bodyFriction, TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction,
        in Vector3 bodyToFrontLeftSuspension, in Vector3 bodyToFrontRightSuspension, in Vector3 bodyToBackLeftSuspension, in Vector3 bodyToBackRightSuspension,
        in Vector3 suspensionDirection, float suspensionLength, in SpringSettings suspensionSettings, in Quaternion localWheelOrientation)
    {
        SimpleCar car;
        car.Body = simulation.Bodies.Add(BodyDescription.CreateDynamic(pose, bodyInertia, bodyShape, -0.01f));
        ref var bodyFilter = ref filters.Allocate(car.Body);
        frictions.Allocate(car.Body) = bodyFriction;
        bodyFilter = new SubgroupCollisionFilter(car.Body.Value, 0);
        QuaternionEx.TransformUnitY(localWheelOrientation, out var wheelAxis);
        car.hingeDescription = new AngularHinge
        {
            LocalHingeAxisA = wheelAxis,
            LocalHingeAxisB = new Vector3(0, 1, 0),
            SpringSettings = new SpringSettings(30, 1)
        };
        car.suspensionDirection = suspensionDirection;
        car.BackLeftWheel = CreateWheel(simulation, filters, frictions, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyFilter, bodyToBackLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
        car.BackRightWheel = CreateWheel(simulation, filters, frictions, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyFilter, bodyToBackRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
        car.FrontLeftWheel = CreateWheel(simulation, filters, frictions, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyFilter, bodyToFrontLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
        car.FrontRightWheel = CreateWheel(simulation, filters, frictions, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyFilter, bodyToFrontRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
        return car;
    }

}

struct SimpleCarController
{
    public SimpleCar Car;

    private float steeringAngle;

    public float SteeringAngle { get { return steeringAngle; } }

    public float SteeringSpeed;
    public float MaximumSteeringAngle;

    public float ForwardSpeed;
    public float ForwardForce;
    public float ZoomMultiplier;
    public float BackwardSpeed;
    public float BackwardForce;
    public float IdleForce;
    public float BrakeForce;

    //Track the previous state to force wakeups if the constraint targets have changed.
    private float previousTargetSpeed;
    private float previousTargetForce;

    public SimpleCarController(SimpleCar car,
        float forwardSpeed, float forwardForce, float zoomMultiplier, float backwardSpeed, float backwardForce, float idleForce, float brakeForce,
        float steeringSpeed, float maximumSteeringAngle)
    {
        Car = car;
        ForwardSpeed = forwardSpeed;
        ForwardForce = forwardForce;
        ZoomMultiplier = zoomMultiplier;
        BackwardSpeed = backwardSpeed;
        BackwardForce = backwardForce;
        IdleForce = idleForce;
        BrakeForce = brakeForce;
        SteeringSpeed = steeringSpeed;
        MaximumSteeringAngle = maximumSteeringAngle;

        steeringAngle = 0;
        previousTargetForce = 0;
        previousTargetSpeed = 0;
    }
    public void Update(Simulation simulation, float dt, float targetSteeringAngle, float targetSpeedFraction, bool zoom, bool brake)
    {
        var steeringAngleDifference = targetSteeringAngle - steeringAngle;
        var maximumChange = SteeringSpeed * dt;
        var steeringAngleChange = MathF.Min(maximumChange, MathF.Max(-maximumChange, steeringAngleDifference));
        var previousSteeringAngle = steeringAngle;
        steeringAngle = MathF.Min(MaximumSteeringAngle, MathF.Max(-MaximumSteeringAngle, steeringAngle + steeringAngleChange));
        if (steeringAngle != previousSteeringAngle)
        {
            //By guarding the constraint modifications behind a state test, we avoid waking up the car every single frame.
            //(We could have also used the ApplyDescriptionWithoutWaking function and then explicitly woke the car up when changes occur.)
            Car.Steer(simulation, Car.FrontLeftWheel, steeringAngle);
            Car.Steer(simulation, Car.FrontRightWheel, steeringAngle);
        }
        float newTargetSpeed, newTargetForce;
        bool allWheels;
        if (brake)
        {
            newTargetSpeed = 0;
            newTargetForce = BrakeForce;
            allWheels = true;
        }
        else if (targetSpeedFraction > 0)
        {
            newTargetForce = zoom ? ForwardForce * ZoomMultiplier : ForwardForce;
            newTargetSpeed = targetSpeedFraction * (zoom ? ForwardSpeed * ZoomMultiplier : ForwardSpeed);
            allWheels = false;
        }
        else if (targetSpeedFraction < 0)
        {
            newTargetForce = zoom ? BackwardForce * ZoomMultiplier : BackwardForce;
            newTargetSpeed = targetSpeedFraction * (zoom ? BackwardSpeed * ZoomMultiplier : BackwardSpeed);
            allWheels = false;
        }
        else
        {
            newTargetForce = IdleForce;
            newTargetSpeed = 0;
            allWheels = true;
        }
        if (previousTargetSpeed != newTargetSpeed || previousTargetForce != newTargetForce)
        {
            previousTargetSpeed = newTargetSpeed;
            previousTargetForce = newTargetForce;
            Car.SetSpeed(simulation, Car.FrontLeftWheel, newTargetSpeed, newTargetForce);
            Car.SetSpeed(simulation, Car.FrontRightWheel, newTargetSpeed, newTargetForce);
            if (allWheels)
            {
                Car.SetSpeed(simulation, Car.BackLeftWheel, newTargetSpeed, newTargetForce);
                Car.SetSpeed(simulation, Car.BackRightWheel, newTargetSpeed, newTargetForce);
            }
            else
            {
                Car.SetSpeed(simulation, Car.BackLeftWheel, 0, 0);
                Car.SetSpeed(simulation, Car.BackRightWheel, 0, 0);
            }
        }
    }
}

struct CarAI
{
    public SimpleCarController Controller;
    public float NextChangeTime;
    public Vector2 Target;

    public void Update(Scene scene, Random random, float accumulatedTime, Vector2 targetMin, Vector2 targetSpan)
    {
        var carBody = scene.Simulation.Bodies.GetBodyReference(Controller.Car.Body);
        ref var pose = ref carBody.Pose;
        QuaternionEx.TransformUnitY(pose.Orientation, out var carUp);
        if (carUp.Y < 0.1f)
        {
            //Alas, car fall over go nini.
            Controller.Update(scene.Simulation, scene.TimestepDuration, 0, 0, false, false);
        }
        else
        {
            if (accumulatedTime >= NextChangeTime)
            {
                Target = targetMin + new Vector2((float)random.NextDouble(), (float)random.NextDouble()) * targetSpan;
                NextChangeTime = accumulatedTime + 1f + (float)random.NextDouble() * 2;
            }
            ref var position = ref carBody.Pose.Position;
            var offset = Target - new Vector2(position.X, position.Z);
            QuaternionEx.TransformUnitZ(pose.Orientation, out var backward);
            var forward = new Vector2(backward.X, backward.Z);
            var offsetLength = offset.Length();
            var forwardLength = forward.Length();
            if (offsetLength > 1e-5f && forwardLength > 1e-5f)
            {
                QuaternionEx.TransformUnitX(pose.Orientation, out var right);
                var offsetDirection = offset / offsetLength;
                var forwardDirection = forward / forwardLength;
                var cosTheta = Vector2.Dot(offsetDirection, forwardDirection);

                var theta = MathF.Acos(MathF.Max(-1, MathF.Min(1, cosTheta)));
                if (Vector3.Dot(right, new Vector3(offsetDirection.X, 0, offsetDirection.Y)) < 0)
                    theta = -theta;

                if (cosTheta < -0.5f)
                {
                    Controller.Update(scene.Simulation, scene.TimestepDuration, theta * 1f, -1, false, false);
                }
                else
                {
                    var speedMultiplier = MathF.Max(0.3f, 1f - MathF.Max(0, MathF.Abs(theta) - 0.1f));
                    //NO BRAKES
                    Controller.Update(scene.Simulation, scene.TimestepDuration, theta * 1f, speedMultiplier, false, false);
                }
            }
        }
    }
}

public class Cars : IAction
{
    float targetTime;


    QuickList<CarAI> cars;

    public void Initialize(Random random, Scene scene)
    {
        var carRand = random.NextDouble();
        var carCount = 512 + (int)(8192 * carRand * carRand * carRand * carRand * carRand * carRand * carRand * carRand);

        cars = new QuickList<CarAI>(carCount, scene.BufferPool);

        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;

        ref var filters = ref (scene.Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.Filters;
        ref var frictions = ref (scene.Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.Friction;
        filters = new CollidableProperty<SubgroupCollisionFilter>(scene.Simulation);
        frictions = new CollidableProperty<float>(scene.Simulation);

        for (int setIndex = 0; setIndex < scene.Simulation.Bodies.Sets.Length; ++setIndex)
        {
            ref var set = ref scene.Simulation.Bodies.Sets[setIndex];
            if (set.Allocated)
            {
                for (int i = 0; i < set.Count; ++i)
                {
                    filters[set.IndexToHandle[i]] = new SubgroupCollisionFilter(0);
                    frictions[set.IndexToHandle[i]] = 2f;
                }
            }
        }

        const float massMultiplier = 9;
        var builder = new CompoundBuilder(scene.BufferPool, scene.Simulation.Shapes, 2);
        builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10 * massMultiplier);
        builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 0.5f * massMultiplier);
        builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
        builder.Dispose();
        var bodyShape = new Compound(children);
        var bodyShapeIndex = scene.Simulation.Shapes.Add(bodyShape);
        var wheelShape = new Cylinder(0.4f, .18f);
        var wheelInertia = wheelShape.ComputeInertia(0.25f * massMultiplier);
        var wheelShapeIndex = scene.Simulation.Shapes.Add(wheelShape);

        const float x = 0.9f;
        const float y = -0.1f;
        const float frontZ = 1.7f;
        const float backZ = -1.7f;
        Span<Vector3> previousPositions = stackalloc Vector3[carCount];


        var interestCenter = (scene.RegionOfInterest.Min + scene.RegionOfInterest.Max) * 0.5f;
        var expansion = new Vector3(20);
        var rejectionRegionMin = scene.RegionOfInterest.Min - expansion;
        var rejectionRegionMax = scene.RegionOfInterest.Max + expansion;
        var rejectionRegionSpan = rejectionRegionMax - rejectionRegionMin;
        var spawnRegionMin = (rejectionRegionMin - interestCenter) * 1.2f - new Vector3(550) + interestCenter;
        var spawnRegionMax = (rejectionRegionMax - interestCenter) * 1.2f + new Vector3(550) + interestCenter;
        var spawnRegionSpan = spawnRegionMax - spawnRegionMin;

        for (int i = 0; i < carCount; ++i)
        {
            Vector3 start;
            Quaternion orientation;
            bool blocked;
            int attemptCount = 0;
            do
            {
                var target = scene.RegionOfInterest.Min * new Vector3(1, 0, 1) + span * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start = spawnRegionMin + spawnRegionSpan * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start.Y = 2;

                //new Vector3(0, 2, 0) + target + carStartDistance * new Vector3(MathF.Sin(theta), 0, MathF.Cos(theta));
                orientation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, MathF.Atan2(target.X - start.X, target.Z - start.Z));
                blocked = false;

                ++attemptCount;
                if (Vector3.Max(rejectionRegionMin, Vector3.Min(rejectionRegionMax, start)) == start)
                    blocked = true;
                else
                {
                    for (int j = 0; j < i; ++j)
                    {
                        if (Vector3.DistanceSquared(previousPositions[j], start) < 36)
                        {
                            blocked = true;
                            break;
                        }
                    }
                }

            } while (blocked && attemptCount < 1000);
            if (blocked)
                continue;

            previousPositions[i] = start;

            cars.AllocateUnsafely() = new CarAI
            {
                Controller = new SimpleCarController(SimpleCar.Create(scene.Simulation, filters, frictions, new RigidPose(start, orientation), bodyShapeIndex, bodyInertia, 0.5f, wheelShapeIndex, wheelInertia, 2f,
                    new Vector3(-x, y, frontZ), new Vector3(x, y, frontZ), new Vector3(-x, y, backZ), new Vector3(x, y, backZ), new Vector3(0, -1, 0), 0.25f,
                    new SpringSettings(5f, 0.7f), QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f)),
                    forwardSpeed: 150, forwardForce: 20 * massMultiplier, zoomMultiplier: 2, backwardSpeed: 50, backwardForce: 6 * massMultiplier, idleForce: 0.25f * massMultiplier, brakeForce: 7 * massMultiplier, steeringSpeed: 1.5f, maximumSteeringAngle: MathF.PI * 0.23f)
            };

        }

        targetTime = 20 + 6 * (float)random.NextDouble();
    }


    public bool Update(Scene scene, Random random, float accumulatedTime, float accumulatedRealTime)
    {
        var targetMin = new Vector2(scene.RegionOfInterest.Min.X, scene.RegionOfInterest.Min.Z);
        var targetSpan = new Vector2(scene.RegionOfInterest.Max.X, scene.RegionOfInterest.Max.Z) - targetMin;
        for (int i = 0; i < cars.Count; ++i)
        {
            cars[i].Update(scene, random, accumulatedTime, targetMin, targetSpan);
        }
        return accumulatedTime < targetTime;
    }
}

