using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

/// <summary>
/// Describes a tank's construction.
/// </summary>
public struct TankDescription
{
    /// <summary>
    /// Description of the tank's turret body.
    /// </summary>
    public TankPartDescription Turret;
    /// <summary>
    /// Description of the tank's barrel body.
    /// </summary>
    public TankPartDescription Barrel;
    /// <summary>
    /// Description of the tank's main body.
    /// </summary>
    public TankPartDescription Body;
    /// <summary>
    /// Location of the barrel's anchor in the tank's local space. The barrel will connect to the turret at this location.
    /// </summary>
    public Vector3 BarrelAnchor;
    /// <summary>
    /// Location of the turret's anchor in the tank's local space. The turret will connect to the main body at this location.
    /// </summary>
    public Vector3 TurretAnchor;
    /// <summary>
    /// Basis of the turret and barrel. (0, 0, -1) * TurretBasis in tank local space corresponds to 0 angle for both turret swivel and barrel pitch measurements.
    /// (1, 0, 0) * TurretBasis corresponds to a 90 degree swivel angle.
    /// (0, 1, 0) * TurretBasis corresponds to a 90 degree pitch angle, and is the axis around which the turret can swivel.
    /// </summary>
    public Quaternion TurretBasis;
    /// <summary>
    /// Servo properties for the tank's swivel constraint.
    /// </summary>
    public ServoSettings TurretServo;
    /// <summary>
    /// Spring properties for the tank's swivel constraint.
    /// </summary>
    public SpringSettings TurretSpring;
    /// <summary>
    /// Servo properties for the tank's barrel pitching constraint.
    /// </summary>
    public ServoSettings BarrelServo;
    /// <summary>
    /// Spring properties for the tank's barrel pitching constraint.
    /// </summary>
    public SpringSettings BarrelSpring;

    /// <summary>
    /// Location in the barrel body's local space where projectiles should be created.
    /// </summary>
    public Vector3 BarrelLocalProjectileSpawn;
    /// <summary>
    /// Inertia of fired projectiles.
    /// </summary>
    public BodyInertia ProjectileInertia;
    /// <summary>
    /// Shape of fired projectiles.
    /// </summary>
    public TypedIndex ProjectileShape;
    /// <summary>
    /// Speed of fired projectiles.
    /// </summary>
    public float ProjectileSpeed;


    /// <summary>
    /// Shape used for all wheels.
    /// </summary>
    public TypedIndex WheelShape;
    /// <summary>
    /// Inertia of each wheel body.
    /// </summary>
    public BodyInertia WheelInertia;

    /// <summary>
    /// Local orientation of the wheels. (1,0,0) * WheelOrientation is the suspension direction, (0,1,0) * WheelOrientation is the axis of rotation for the wheels, and (0,0,1) * WheelOrientation is the axis along which the treads will extend.
    /// </summary>
    public Quaternion WheelOrientation;
    /// <summary>
    /// Offset from the tank's local space origin to the left tread's center. The tread will be aligned along (0,0,1) * WheelOrientation.
    /// </summary>
    public Vector3 LeftTreadOffset;
    /// <summary>
    /// Offset from the tank's local space origin to the right tread's center. The tread will be aligned along (0,0,1) * WheelOrientation.
    /// </summary>
    public Vector3 RightTreadOffset;
    /// <summary>
    /// Number of wheels in each tread.
    /// </summary>
    public int WheelCountPerTread;
    /// <summary>
    /// How much space to put in between wheels in the tread.
    /// </summary>
    public float TreadSpacing;
    /// <summary>
    /// Resting length of the suspension for each wheel.
    /// </summary>
    public float SuspensionLength;
    /// <summary>
    /// Spring settings for the wheel suspension.
    /// </summary>
    public SpringSettings SuspensionSettings;
    /// <summary>
    /// Friction for the wheel bodies.
    /// </summary>
    public float WheelFriction;
}

/// <summary>
/// Describes properties of a piece of a tank.
/// </summary>
public struct TankPartDescription
{
    /// <summary>
    /// Shape index used by this part's collidable.
    /// </summary>
    public TypedIndex Shape;
    /// <summary>
    /// Inertia of this part's body.
    /// </summary>
    public BodyInertia Inertia;
    /// <summary>
    /// Pose of the part in the tank's local space.
    /// </summary>
    public RigidPose Pose;
    /// <summary>
    /// Friction of the body to be used in pair material calculations.
    /// </summary>
    public float Friction;

    public static TankPartDescription Create<TShape>(float mass, in TShape shape, in RigidPose pose, float friction, Shapes shapes) where TShape : unmanaged, IConvexShape
    {
        TankPartDescription description;
        description.Shape = shapes.Add(shape);
        description.Inertia = shape.ComputeInertia(mass);
        description.Pose = pose;
        description.Friction = friction;
        return description;
    }
}

/// <summary>
/// Set of handles and references to a tank instance.
/// </summary>
public struct Tank
{
    /// <summary>
    /// Body handle of the tank's main body.
    /// </summary>
    public BodyHandle Body;
    /// <summary>
    /// Body handle of the tank's turret.
    /// </summary>
    public BodyHandle Turret;
    /// <summary>
    /// Body handle of the tank's barrel.
    /// </summary>
    public BodyHandle Barrel;
    /// <summary>
    /// Constraint handle of the turret swivel servo.
    /// </summary>
    public ConstraintHandle TurretServo;
    /// <summary>
    /// Constraint handle of the barrel pitch servo.
    /// </summary>
    public ConstraintHandle BarrelServo;
    /// <summary>
    /// List of all wheel body handles associated with the tank.
    /// </summary>
    public Buffer<BodyHandle> WheelHandles;
    /// <summary>
    /// List of all constraint handles associated with the tank. Includes motors.
    /// </summary>
    public Buffer<ConstraintHandle> Constraints;
    /// <summary>
    /// List of constraint handles associated with the left tread's drive motors.
    /// </summary>
    public Buffer<ConstraintHandle> LeftMotors;
    /// <summary>
    /// List of constraint handles associated with the right tread's drive motors.
    /// </summary>
    public Buffer<ConstraintHandle> RightMotors;

    /// <summary>
    /// Transforms directions from body local space to turret basis local space. Used for computing aiming angles.
    /// </summary>
    Quaternion FromBodyLocalToTurretBasisLocal;
    /// <summary>
    /// Orientation of the body in the tank's local space.
    /// </summary>
    public Quaternion BodyLocalOrientation;
    /// <summary>
    /// Location in the barrel body's local space where projectiles should be created.
    /// </summary>
    Vector3 BarrelLocalProjectileSpawn;
    /// <summary>
    /// Direction in the barrel body's local space along which projectiles should be fired.
    /// </summary>
    Vector3 BarrelLocalDirection;
    /// <summary>
    /// Speed of projectiles fired by the tank.
    /// </summary>
    float ProjectileSpeed;
    /// <summary>
    /// Inertia of projectiles fired by the tank.
    /// </summary>
    BodyInertia ProjectileInertia;
    /// <summary>
    /// Shape of the projectiles fired by the tank.
    /// </summary>
    TypedIndex ProjectileShape;

    //We cache the motor descriptions so we don't need to recompute the bases.
    TwistServo BarrelServoDescription;
    TwistServo TurretServoDescription;

    public void SetSpeed(Simulation simulation, Buffer<ConstraintHandle> motors, float speed, float maximumForce)
    {
        //This sets all properties of a motor at once; it's possible to create a custom description that only assigns a subset of properties if you find this to be somehow expensive.
        var motorDescription = new AngularAxisMotor
        {
            //Assuming the wheels are cylinders oriented in the obvious way.
            LocalAxisA = new Vector3(0, -1, 0),
            Settings = new MotorSettings(maximumForce, 1e-6f),
            TargetVelocity = speed
        };
        for (int i = 0; i < motors.Length; ++i)
        {
            simulation.Solver.ApplyDescription(motors[i], motorDescription);
        }
    }


    /// <summary>
    /// Computes the swivel and pitch angles required to aim in a given direction based on the tank's current pose.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="aimDirection">Direction to aim in.</param>
    /// <returns>Swivel and pitch angles to point in the given direction.</returns>
    public (float targetSwivelAngle, float targetPitchAngle) ComputeTurretAngles(Simulation simulation, in Vector3 aimDirection)
    {
        //Decompose the aim direction into target angles for the turret and barrel servos.
        //First, we need to compute the frame of reference and transform the aim direction into the tank's local space.
        //aimDirection * inverse(body.Pose.Orientation) * Tank.LocalBodyPose.Orientation * inverse(Tank.TurretBasis)
        QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies.GetBodyReference(Body).Pose.Orientation), FromBodyLocalToTurretBasisLocal, out var toTurretBasis);
        //-Z in the turret basis points along the 0 angle direction for both swivel and pitch.
        //+Y is 90 degrees for pitch.
        //+X is 90 degres for swivel.
        //We'll compute the swivel angle first.
        QuaternionEx.TransformWithoutOverlap(aimDirection, toTurretBasis, out var aimDirectionInTurretBasis);
        var targetSwivelAngle = MathF.Atan2(aimDirectionInTurretBasis.X, -aimDirectionInTurretBasis.Z);

        //Barrel pitching is measured against the +Y axis and an axis created from the target swivel angle.
        var targetPitchAngle = MathF.Asin(MathF.Max(-1f, MathF.Min(1f, -aimDirectionInTurretBasis.Y)));
        return (targetSwivelAngle, targetPitchAngle);
    }

    /// <summary>
    /// Applies a target swivel and pitch angle to the turret's servos.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="targetSwivelAngle">Target swivel angle of the turret.</param>
    /// <param name="targetPitchAngle">Target pitch angle of the barrel.</param>
    public void SetAim(Simulation simulation, float targetSwivelAngle, float targetPitchAngle)
    {
        var turretDescription = TurretServoDescription;
        turretDescription.TargetAngle = targetSwivelAngle;
        simulation.Solver.ApplyDescription(TurretServo, turretDescription);
        var barrelDescription = BarrelServoDescription;
        barrelDescription.TargetAngle = targetPitchAngle;
        simulation.Solver.ApplyDescription(BarrelServo, barrelDescription);

    }

    /// <summary>
    /// Computes the direction along which the barrel points.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="barrelDirection">Direction in which the barrel points.</param>
    public void ComputeBarrelDirection(Simulation simulation, out Vector3 barrelDirection)
    {
        QuaternionEx.Transform(BarrelLocalDirection, simulation.Bodies.GetBodyReference(Barrel).Pose.Orientation, out barrelDirection);
    }

    /// <summary>
    /// Fires a projectile.
    /// </summary>
    /// <param name="simulation">Simulation that contains the tank.</param>
    /// <param name="bodyProperties">Body properties to allocate the projectile's properties in.</param>
    /// <returns>Handle of the created projectile body.</returns>
    public BodyHandle Fire(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions)
    {
        var barrel = simulation.Bodies.GetBodyReference(Barrel);
        ref var barrelPose = ref barrel.Pose;
        RigidPose.Transform(BarrelLocalProjectileSpawn, barrelPose, out var projectileSpawn);
        QuaternionEx.Transform(BarrelLocalDirection, barrelPose.Orientation, out var barrelDirection);
        var projectileHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(projectileSpawn, new BodyVelocity(barrelDirection * ProjectileSpeed + barrel.Velocity.Linear), ProjectileInertia,
            //The projectile moves pretty fast, so we'll use continuous collision detection.
            new CollidableDescription(ProjectileShape, 0.1f, ContinuousDetection.Continuous(1e-3f, 1e-3f)), 0.01f));
        ref var filter = ref filters.Allocate(projectileHandle);
        ref var friction = ref frictions.Allocate(projectileHandle);
        friction = 1f;
        //Prevent the projectile from colliding with the firing tank.
        filter = new SubgroupCollisionFilter(Body.Value);
        filter.CollidableSubgroups = 0;
        filter.SubgroupMembership = 0;

        barrel.Awake = true;
        barrel.ApplyLinearImpulse(barrelDirection * -ProjectileSpeed / ProjectileInertia.InverseMass);
        return projectileHandle;
    }

    static BodyHandle CreateWheel(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, in RigidPose tankPose, in RigidPose bodyLocalPose,
        TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, float suspensionLength,
        in SpringSettings suspensionSettings, in Quaternion localWheelOrientation,
        ref QuickList<BodyHandle> wheelHandles, ref QuickList<ConstraintHandle> constraints, ref QuickList<ConstraintHandle> motors)
    {
        RigidPose wheelPose;
        QuaternionEx.TransformUnitX(localWheelOrientation, out var suspensionDirection);
        RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, tankPose, out wheelPose.Position);
        QuaternionEx.ConcatenateWithoutOverlap(localWheelOrientation, tankPose.Orientation, out wheelPose.Orientation);

        var wheelHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, wheelShape, 0.01f));
        wheelHandles.AllocateUnsafely() = wheelHandle;

        //We need a LinearAxisServo to act as the suspension spring, pushing the wheel down.
        constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new LinearAxisServo
        {
            LocalPlaneNormal = suspensionDirection,
            TargetOffset = suspensionLength,
            LocalOffsetA = bodyToWheelSuspension,
            LocalOffsetB = default,
            ServoSettings = ServoSettings.Default,
            SpringSettings = suspensionSettings
        });
        //A PointOnLineServo keeps the wheel on a fixed track. Note that it does not constrain the angular behavior of the wheel at all.
        constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new PointOnLineServo
        {
            LocalDirection = suspensionDirection,
            LocalOffsetA = bodyToWheelSuspension,
            LocalOffsetB = default,
            ServoSettings = ServoSettings.Default,
            SpringSettings = new SpringSettings(30, 1)
        });
        //The angular component is handled by a hinge. Note that we only use the angular component of a hinge constraint here- the PointOnLineServo handles the linear degrees of freedom.
        //We're assuming the wheels will be cylinders. Pretty safe bet. A cylinder rolls around its local Y axis, so the motor will act along that axis.
        QuaternionEx.TransformUnitY(localWheelOrientation, out var wheelRotationAxis);
        constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new AngularHinge
        {
            LocalHingeAxisA = QuaternionEx.Transform(wheelRotationAxis, QuaternionEx.Conjugate(bodyLocalPose.Orientation)),
            LocalHingeAxisB = new Vector3(0, 1, 0),
            SpringSettings = new SpringSettings(30, 1)
        });
        //We'll need a velocity motor to actually make the tank move.
        var motorHandle = simulation.Solver.Add(wheelHandle, bodyHandle, new AngularAxisMotor
        {
            //(All these are technically set on the fly during the update right now, but a custom constraint description could set only the Settings and TargetVelocity,
            //leaving the LocalAxisA unchanged, so we'll go ahead and set it to a reasonable value.)
            LocalAxisA = new Vector3(0, 1, 0),
            Settings = default,
            TargetVelocity = default
        });
        motors.AllocateUnsafely() = motorHandle;
        constraints.AllocateUnsafely() = motorHandle;
        ref var friction = ref frictions.Allocate(wheelHandle);
        ref var filter = ref filters.Allocate(wheelHandle);
        filter = new SubgroupCollisionFilter(bodyHandle.Value, 3);
        friction = wheelFriction;
        //The wheels don't need to be tested against the body or each other.
        SubgroupCollisionFilter.DisableCollision(ref filter, ref bodyFilter);
        SubgroupCollisionFilter.DisableCollision(ref filter, ref filter);
        return wheelHandle;
    }

    static ref SubgroupCollisionFilter CreatePart(Simulation simulation, in TankPartDescription part, RigidPose pose, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, out BodyHandle handle)
    {
        RigidPose.MultiplyWithoutOverlap(part.Pose, pose, out var bodyPose);
        handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(bodyPose, part.Inertia, part.Shape, 0.01f));
        ref var partFilter = ref filters.Allocate(handle);
        ref var partFriction = ref frictions.Allocate(handle);
        partFriction = part.Friction;
        return ref partFilter;
    }

    /// <summary>
    /// Creates a tank from a provided tank description.
    /// </summary>
    /// <param name="simulation">Simulation to add the tank to.</param>
    /// <param name="properties">Property set to store per-body information into.</param>
    /// <param name="pool">Buffer pool to allocate tank resources from.</param>
    /// <param name="pose">Pose of the tank.</param>
    /// <param name="description">Description of the tank.</param>
    /// <returns>Tank instance containing references to the simulation tank parts.</returns>
    public static Tank Create(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, BufferPool pool, in RigidPose pose, in TankDescription description)
    {
        var wheelHandles = new QuickList<BodyHandle>(description.WheelCountPerTread * 2, pool);
        var constraints = new QuickList<ConstraintHandle>(description.WheelCountPerTread * 2 * 6 + 4, pool);
        var leftMotors = new QuickList<ConstraintHandle>(description.WheelCountPerTread, pool);
        var rightMotors = new QuickList<ConstraintHandle>(description.WheelCountPerTread, pool);
        Tank tank;
        ref var bodyFilter = ref CreatePart(simulation, description.Body, pose, filters, frictions, out tank.Body);
        ref var turretFilter = ref CreatePart(simulation, description.Turret, pose, filters, frictions, out tank.Turret);
        ref var barrelFilter = ref CreatePart(simulation, description.Barrel, pose, filters, frictions, out tank.Barrel);
        //Use the tank's body handle as the group id for collision filters.
        bodyFilter = new SubgroupCollisionFilter(tank.Body.Value, 0);
        turretFilter = new SubgroupCollisionFilter(tank.Body.Value, 1);
        barrelFilter = new SubgroupCollisionFilter(tank.Body.Value, 2);
        SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
        SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);

        Matrix3x3.CreateFromQuaternion(description.TurretBasis, out var turretBasis);

        //Attach the turret to the body.
        QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(description.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
        QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
        RigidPose.TransformByInverse(description.TurretAnchor, description.Body.Pose, out var bodyLocalTurretAnchor);
        RigidPose.TransformByInverse(description.TurretAnchor, description.Turret.Pose, out var turretLocalTurretAnchor);
        constraints.AllocateUnsafely() = simulation.Solver.Add(tank.Body, tank.Turret,
            new Hinge
            {
                LocalHingeAxisA = bodyLocalSwivelAxis,
                LocalHingeAxisB = turretLocalSwivelAxis,
                LocalOffsetA = bodyLocalTurretAnchor,
                LocalOffsetB = turretLocalTurretAnchor,
                SpringSettings = new SpringSettings(30, 1)
            });
        //The twist servo might seem like an odd choice to control 1 angular degree of freedom, but servo-like control over 1DOF requires a measurement basis to interpret the target angle.
        //Hence the apparent complexity.
        Matrix3x3 turretSwivelBasis;
        turretSwivelBasis.Z = -turretBasis.Y;
        turretSwivelBasis.X = -turretBasis.Z;
        turretSwivelBasis.Y = turretBasis.X;
        Debug.Assert(turretSwivelBasis.Determinant() > 0.999f && turretSwivelBasis.Determinant() < 1.0001f, "The turret swivel axis and forward axis should be perpendicular and unit length.");
        QuaternionEx.CreateFromRotationMatrix(turretSwivelBasis, out var turretSwivelBasisQuaternion);
        QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(description.Body.Pose.Orientation), out var bodyLocalTurretBasis);
        QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalTurretBasis);
        tank.TurretServoDescription = new TwistServo
        {
            LocalBasisA = bodyLocalTurretBasis,
            LocalBasisB = turretLocalTurretBasis,
            SpringSettings = description.TurretSpring,
            ServoSettings = description.TurretServo
        };
        tank.TurretServo = simulation.Solver.Add(tank.Body, tank.Turret, tank.TurretServoDescription);
        constraints.AllocateUnsafely() = tank.TurretServo;

        //Attach the barrel to the turret.
        QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalPitchAxis);
        QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
        RigidPose.TransformByInverse(description.BarrelAnchor, description.Turret.Pose, out var turretLocalBarrelAnchor);
        RigidPose.TransformByInverse(description.BarrelAnchor, description.Barrel.Pose, out var barrelLocalBarrelAnchor);
        constraints.AllocateUnsafely() = simulation.Solver.Add(tank.Turret, tank.Barrel,
            new Hinge
            {
                LocalHingeAxisA = turretLocalPitchAxis,
                LocalHingeAxisB = barrelLocalPitchAxis,
                LocalOffsetA = turretLocalBarrelAnchor,
                LocalOffsetB = barrelLocalBarrelAnchor,
                SpringSettings = new SpringSettings(30, 1)
            });
        //The twist servo might seem like an odd choice to control 1 angular degree of freedom, but servo-like control over 1DOF requires a measurement basis to interpret the target angle.
        //Hence the apparent complexity.
        Matrix3x3 barrelPitchBasis;
        barrelPitchBasis.Z = -turretBasis.X;
        barrelPitchBasis.X = -turretBasis.Z;
        barrelPitchBasis.Y = -turretBasis.Y;
        Debug.Assert(barrelPitchBasis.Determinant() > 0.999f && barrelPitchBasis.Determinant() < 1.0001f, "The barrel axis and forward axis should be perpendicular and unit length.");
        QuaternionEx.CreateFromRotationMatrix(barrelPitchBasis, out var barrelPitchBasisQuaternion);
        QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
        QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
        tank.BarrelServoDescription = new TwistServo
        {
            LocalBasisA = turretLocalBarrelBasis,
            LocalBasisB = barrelLocalBarrelBasis,
            SpringSettings = description.BarrelSpring,
            ServoSettings = description.BarrelServo
        };
        tank.BarrelServo = simulation.Solver.Add(tank.Turret, tank.Barrel, tank.BarrelServoDescription);
        constraints.AllocateUnsafely() = tank.BarrelServo;

        QuaternionEx.TransformUnitY(description.WheelOrientation, out var wheelAxis);
        QuaternionEx.TransformUnitZ(description.WheelOrientation, out var treadDirection);
        var treadStart = description.TreadSpacing * (description.WheelCountPerTread - 1) * -0.5f;
        BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
        for (int i = 0; i < description.WheelCountPerTread; ++i)
        {
            var wheelOffsetFromTread = treadDirection * (treadStart + i * description.TreadSpacing);
            var rightWheelHandle = CreateWheel(simulation, filters, frictions, pose, description.Body.Pose,
                description.WheelShape, description.WheelInertia, description.WheelFriction, tank.Body, ref filters[tank.Body],
                description.RightTreadOffset + wheelOffsetFromTread - description.Body.Pose.Position,
                description.SuspensionLength, description.SuspensionSettings, description.WheelOrientation,
                ref wheelHandles, ref constraints, ref rightMotors);
            var leftWheelHandle = CreateWheel(simulation, filters, frictions, pose, description.Body.Pose,
                description.WheelShape, description.WheelInertia, description.WheelFriction, tank.Body, ref filters[tank.Body],
                description.LeftTreadOffset + wheelOffsetFromTread - description.Body.Pose.Position,
                description.SuspensionLength, description.SuspensionSettings, description.WheelOrientation,
                ref wheelHandles, ref constraints, ref leftMotors);

            if (i >= 1)
            {
                //Connect wheels in a tread to each other to distribute the drive forces.
                //The motor will always just target 0 velocity. The wheel orientations will be allowed to drift from each other.
                //(If you didn't want to allow drift, you could use an AngularServo or TwistServo. AngularServo constrains all 3 degrees of freedom, but for these purposes, that'd be fine.)
                var motorDescription = new AngularAxisMotor { LocalAxisA = new Vector3(0, 1, 0), Settings = new MotorSettings(float.MaxValue, 1e-4f) };
                constraints.AllocateUnsafely() = simulation.Solver.Add(previousLeftWheelHandle, leftWheelHandle, motorDescription);
                constraints.AllocateUnsafely() = simulation.Solver.Add(previousRightWheelHandle, rightWheelHandle, motorDescription);
            }
            previousLeftWheelHandle = leftWheelHandle;
            previousRightWheelHandle = rightWheelHandle;

        }

        tank.WheelHandles = wheelHandles.Span.Slice(wheelHandles.Count);
        tank.Constraints = constraints.Span.Slice(constraints.Count);
        tank.LeftMotors = leftMotors.Span.Slice(leftMotors.Count);
        tank.RightMotors = rightMotors.Span.Slice(rightMotors.Count);

        //To aim, we transform the aim direction into the turret basis. 
        //aimDirectionInTurretBasis = worldAimDirection * inverse(body.Pose.Orientation) * description.Body.Pose.Orientation * inverse(description.TurretBasis), so we precompute and cache:
        //FromBodyLocalToTurretBasisLocal = description.Body.Pose.Orientation * inverse(description.TurretBasis)
        QuaternionEx.ConcatenateWithoutOverlap(description.Body.Pose.Orientation, QuaternionEx.Conjugate(description.TurretBasis), out tank.FromBodyLocalToTurretBasisLocal);
        tank.BodyLocalOrientation = description.Body.Pose.Orientation;
        tank.BarrelLocalProjectileSpawn = description.BarrelLocalProjectileSpawn;
        QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out tank.BarrelLocalDirection);
        tank.ProjectileInertia = description.ProjectileInertia;
        tank.ProjectileShape = description.ProjectileShape;
        tank.ProjectileSpeed = description.ProjectileSpeed;
        return tank;
    }

    void ClearBodyProperties(ref SubgroupCollisionFilter filter)
    {
        //After blowing up, all tank parts will collide with each other, and we should no longer flag the pieces as part of a living tank.
        filter = new SubgroupCollisionFilter(filter.GroupId);
    }

    public void Explode(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions, BufferPool pool)
    {
        //When the tank explodes, we just remove all the binding constraints and let it fall apart and reset body properties.
        for (int i = 0; i < WheelHandles.Length; ++i)
        {
            ClearBodyProperties(ref filters[WheelHandles[i]]);
        }
        ClearBodyProperties(ref filters[Body]);
        ClearBodyProperties(ref filters[Turret]);
        ClearBodyProperties(ref filters[Barrel]);
        var turret = simulation.Bodies.GetBodyReference(Turret);
        turret.Awake = true;
        turret.Velocity.Linear += new Vector3(0, 10, 0);
        for (int i = 0; i < Constraints.Length; ++i)
        {
            simulation.Solver.Remove(Constraints[i]);
        }
        pool.Return(ref WheelHandles);
        pool.Return(ref Constraints);
        pool.Return(ref LeftMotors);
        pool.Return(ref RightMotors);
    }

}
/// <summary>
/// Applies control inputs to a tank instance.
/// </summary>
public struct TankController
{
    public Tank Tank;

    //While the Tank instance contains references to all the simulation-contained stuff, none of it actually defines how fast or strong the tank is.
    //We store that here in the controller so it can be modified on the fly.
    public float Speed;
    public float Force;
    public float ZoomMultiplier;
    public float IdleForce;
    public float BrakeForce;

    //Track the previous state to force wakeups if the constraint targets have changed.
    private float previousLeftTargetSpeed;
    private float previousLeftForce;
    private float previousRightTargetSpeed;
    private float previousRightForce;
    private float previousTurretSwivel;
    private float previousBarrelPitch;

    public TankController(Tank tank,
        float speed, float force, float zoomMultiplier, float idleForce, float brakeForce) : this()
    {
        Tank = tank;
        Speed = speed;
        Force = force;
        ZoomMultiplier = zoomMultiplier;
        IdleForce = idleForce;
        BrakeForce = brakeForce;
    }

    /// <summary>
    /// Updates constraint targets for an input state.
    /// </summary>
    /// <param name="simulation">Simulation containing the tank.</param>
    /// <param name="leftTargetSpeedFraction">Target speed fraction of the maximum speed for the left tread.</param>
    /// <param name="rightTargetSpeedFraction">Target speed fraction of the maximum speed for the right tread.</param>
    /// <param name="zoom">Whether or not to use the boost mulitplier.</param>
    /// <param name="brakeLeft">Whether the left tread should brake.</param>
    /// <param name="brakeRight">Whether the right tread should brake.</param>
    /// <param name="aimDirection">Direction that the tank's barrel should point.</param>
    public void UpdateMovementAndAim(Simulation simulation, float leftTargetSpeedFraction, float rightTargetSpeedFraction, bool zoom, bool brakeLeft, bool brakeRight, in Vector3 aimDirection)
    {
        var leftTargetSpeed = brakeLeft ? 0 : leftTargetSpeedFraction * Speed;
        var rightTargetSpeed = brakeRight ? 0 : rightTargetSpeedFraction * Speed;
        if (zoom)
        {
            leftTargetSpeed *= ZoomMultiplier;
            rightTargetSpeed *= ZoomMultiplier;
        }
        var leftForce = brakeLeft ? BrakeForce : leftTargetSpeedFraction == 0 ? IdleForce : Force;
        var rightForce = brakeRight ? BrakeForce : rightTargetSpeedFraction == 0 ? IdleForce : Force;

        var (targetSwivelAngle, targetPitchAngle) = Tank.ComputeTurretAngles(simulation, aimDirection);

        if (leftTargetSpeed != previousLeftTargetSpeed || rightTargetSpeed != previousRightTargetSpeed ||
            leftForce != previousLeftForce || rightForce != previousRightForce ||
            targetSwivelAngle != previousTurretSwivel || targetPitchAngle != previousBarrelPitch)
        {
            //By guarding the constraint modifications behind a state test, we avoid waking up the tank every single frame.
            //(We could have also used the ApplyDescriptionWithoutWaking function and then explicitly woke the tank up when changes occur.)
            Tank.SetSpeed(simulation, Tank.LeftMotors, leftTargetSpeed, leftForce);
            Tank.SetSpeed(simulation, Tank.RightMotors, rightTargetSpeed, rightForce);
            previousLeftTargetSpeed = leftTargetSpeed;
            previousRightTargetSpeed = rightTargetSpeed;
            previousLeftForce = leftForce;
            previousRightForce = rightForce;
            Tank.SetAim(simulation, targetSwivelAngle, targetPitchAngle);
            previousTurretSwivel = targetSwivelAngle;
            previousBarrelPitch = targetPitchAngle;
        }


    }
}

struct TankAI
{
    public TankController Controller;
    public float NextMovementChangeTime;
    public Vector2 MovementTarget;
    public float NextAimTargetChangeTime;
    public Vector3 AimTarget;
    public float NextFireTime;

    public bool MachineGunTooter;

    public void Update(Scene scene, Random random, float accumulatedTime, Vector2 movementMin, Vector2 movementSpan, Vector2 movementRejectionMin, Vector2 movementRejectionMax, Vector3 aimMin, Vector3 aimSpan,
        CollidableProperty<SubgroupCollisionFilter> filters, CollidableProperty<float> frictions)
    {
        var body = scene.Simulation.Bodies.GetBodyReference(Controller.Tank.Body);
        ref var pose = ref body.Pose;
        QuaternionEx.TransformUnitY(pose.Orientation, out var carUp);
        if (carUp.Y < -0.1f)
        {
            //Alas, tank fall over go nini.
            Controller.UpdateMovementAndAim(scene.Simulation, 0, 0, false, false, false, default);
        }
        else
        {
            if (accumulatedTime >= NextMovementChangeTime)
            {
                do
                {
                    MovementTarget = movementMin + new Vector2((float)random.NextDouble(), (float)random.NextDouble()) * movementSpan;
                } while (Vector2.Max(Vector2.Min(MovementTarget, movementRejectionMax), movementRejectionMin) == MovementTarget);
                NextMovementChangeTime = accumulatedTime + 4f + (float)random.NextDouble() * 2;
            }
            if (accumulatedTime >= NextAimTargetChangeTime)
            {
                AimTarget = aimMin + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * aimSpan;
                NextAimTargetChangeTime = accumulatedTime + (MachineGunTooter ? 0.25f : 1f + (float)random.NextDouble() * 2);
                NextFireTime = MathF.Max(accumulatedTime + .1f, NextFireTime);
            }
        }


        var offset = new Vector3(MovementTarget.X - pose.Position.X, 0, MovementTarget.Y - pose.Position.Y);
        QuaternionEx.Transform(offset, QuaternionEx.Concatenate(QuaternionEx.Conjugate(pose.Orientation), Controller.Tank.BodyLocalOrientation), out var localMovementOffset);

        var targetHorizontalMovementDirection = new Vector2(localMovementOffset.X, -localMovementOffset.Z);
        var targetDirectionLength = targetHorizontalMovementDirection.Length();
        targetHorizontalMovementDirection = targetDirectionLength > 1e-10f ? targetHorizontalMovementDirection / targetDirectionLength : new Vector2(0, 1);
        var turnWeight = targetHorizontalMovementDirection.Y > 0 ? targetHorizontalMovementDirection.X : targetHorizontalMovementDirection.X > 0 ? 1f : -1f;
        //Set the leftTrack to 1 at turnWeight >= 0. At turnWeight -1, leftTrack should be -1.
        var leftTrack = MathF.Min(1f, 2f * turnWeight + 1f);
        //rightTrack = 1 at turnWeight <= 0, rightTrack = -1 at turnWeight = 1.
        var rightTrack = MathF.Min(1f, -2f * turnWeight + 1f);

        //If we're close to the target, slow down a bit.
        var speedMultiplier = Math.Min(targetDirectionLength * 0.05f, 1f);
        leftTrack *= speedMultiplier;
        rightTrack *= speedMultiplier;
        //If we're far away from the target but are pointing in the right direction, zoom.
        var zoom = targetDirectionLength > 50 && targetHorizontalMovementDirection.Y > 0.8f;
        //We're not going to compute an optimal firing solution- just aim directly at the middle of the other tank. Pretty poor choice, but that's fine.
        ref var barrelPosition = ref scene.Simulation.Bodies.GetBodyReference(Controller.Tank.Barrel).Pose.Position;
        var barrelToTarget = AimTarget - barrelPosition;
        var barrelToTargetLength = barrelToTarget.Length();
        barrelToTarget = barrelToTargetLength > 1e-10f ? barrelToTarget / barrelToTargetLength : new Vector3(0, 1, 0);
        Controller.UpdateMovementAndAim(scene.Simulation, leftTrack, rightTrack, zoom, false, false, barrelToTarget);
        if (accumulatedTime > NextFireTime)
        {
            //Are we aiming reasonably close to the target?
            if (barrelToTargetLength > 1e-10f)
            {
                Controller.Tank.ComputeBarrelDirection(scene.Simulation, out var barrelDirection);
                var dot = Vector3.Dot(barrelDirection, barrelToTarget);
                if (dot > 0.98f)
                {
                    Controller.Tank.Fire(scene.Simulation, filters, frictions);
                    NextFireTime = MachineGunTooter ? accumulatedTime : accumulatedTime + 1f + (float)random.NextDouble() * 1f;
                }
            }
        }
    }
}

public class Tanks : IAction
{
    float targetTime;


    QuickList<TankAI> tanks;
    CollidableProperty<SubgroupCollisionFilter> filters;
    CollidableProperty<float> frictions;
    public void Initialize(Random random, Scene scene)
    {
        var tankCount = random.Next(8, 64);

        tanks = new QuickList<TankAI>(tankCount, scene.BufferPool);


        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;

        ref var filters = ref (scene.Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.Filters;
        ref var frictions = ref (scene.Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.Friction;
        filters = new CollidableProperty<SubgroupCollisionFilter>(scene.Simulation);
        frictions = new CollidableProperty<float>(scene.Simulation);
        this.filters = filters;
        this.frictions = frictions;
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

        var builder = new CompoundBuilder(scene.BufferPool, scene.Simulation.Shapes, 2);
        builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10);
        builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 0.5f);
        builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
        builder.Dispose();
        var bodyShape = new Compound(children);
        var bodyShapeIndex = scene.Simulation.Shapes.Add(bodyShape);
        var wheelShape = new Cylinder(0.4f, .18f);
        var wheelShapeIndex = scene.Simulation.Shapes.Add(wheelShape);
        var projectileShape = new Sphere(0.2f);
        var projectileShapeIndex = scene.Simulation.Shapes.Add(projectileShape);

        Span<Vector3> previousPositions = stackalloc Vector3[tankCount];

        var interestCenter = (scene.RegionOfInterest.Min + scene.RegionOfInterest.Max) * 0.5f;
        var expansion = new Vector3(20);
        var rejectionRegionMin = scene.RegionOfInterest.Min - expansion;
        var rejectionRegionMax = scene.RegionOfInterest.Max + expansion;
        var rejectionRegionSpan = rejectionRegionMax - rejectionRegionMin;
        var spawnRegionMin = (rejectionRegionMin - interestCenter) * 1.2f - expansion * 2 + interestCenter;
        var spawnRegionMax = (rejectionRegionMax - interestCenter) * 1.2f + expansion * 2 + interestCenter;
        var spawnRegionSpan = spawnRegionMax - spawnRegionMin;

        for (int i = 0; i < tankCount; ++i)
        {
            Vector3 start;
            Quaternion orientation;
            bool blocked;
            do
            {

                var target = scene.RegionOfInterest.Min * new Vector3(1, 0, 1) + span * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start = spawnRegionMin + spawnRegionSpan * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
                start.Y = 2;

                //new Vector3(0, 2, 0) + target + carStartDistance * new Vector3(MathF.Sin(theta), 0, MathF.Cos(theta));
                orientation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, MathF.Atan2(start.X - target.X, start.Z - target.Z));
                blocked = false;

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

            } while (blocked);

            previousPositions[i] = start;

            var supertooter = random.NextDouble() < 0.05;
            //Don't scale up the projectile mass on a supertooter.
            const float baseMassMultiplier = 10f;
            float massMultiplier = supertooter ? baseMassMultiplier * 8 : baseMassMultiplier;
            var wheelInertia = wheelShape.ComputeInertia(0.25f * massMultiplier);
            var projectileInertia = projectileShape.ComputeInertia(0.2f * baseMassMultiplier);
            var tankDescription = new TankDescription
            {
                Body = TankPartDescription.Create(10 * massMultiplier, new Box(4f, 1, 5), RigidPose.Identity, 0.5f, scene.Simulation.Shapes),
                Turret = TankPartDescription.Create(1 * massMultiplier,
                    supertooter ? new Box(2.5f, 0.7f, 3f) : new Box(1.5f, 0.7f, 2f), new RigidPose(new Vector3(0, 0.85f, 0.4f)), 0.5f, scene.Simulation.Shapes),
                Barrel = TankPartDescription.Create(0.5f * massMultiplier,
                    supertooter ? new Box(0.4f, 0.4f, 4f) : new Box(0.2f, 0.2f, 3f), supertooter ? new RigidPose(new Vector3(0, 0.85f, 0.4f - 1.5f - 2f)) : new RigidPose(new Vector3(0, 0.85f, 0.4f - 1.5f - 1f)), 0.5f, scene.Simulation.Shapes),
                TurretAnchor = new Vector3(0f, 0.5f, 0.4f),
                BarrelAnchor = new Vector3(0, 0.5f + 0.35f, 0.4f - 1f),
                TurretBasis = Quaternion.Identity,
                TurretServo = new ServoSettings(1f, 0f, 40f * massMultiplier),
                TurretSpring = new SpringSettings(10f, 1f),
                BarrelServo = new ServoSettings(1f, 0f, 40f * massMultiplier),
                BarrelSpring = new SpringSettings(10f, 1f),

                ProjectileShape = projectileShapeIndex,
                ProjectileSpeed = 100f,
                BarrelLocalProjectileSpawn = new Vector3(0, 0, -1.5f),
                ProjectileInertia = projectileInertia,

                LeftTreadOffset = new Vector3(-1.9f, 0f, 0),
                RightTreadOffset = new Vector3(1.9f, 0f, 0),
                SuspensionLength = 1f,
                SuspensionSettings = new SpringSettings(2.5f, 1.5f),
                WheelShape = wheelShapeIndex,
                WheelInertia = wheelInertia,
                WheelFriction = 2f,
                TreadSpacing = 1f,
                WheelCountPerTread = 5,
                WheelOrientation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * -0.5f),
            };

            tanks.AllocateUnsafely() = new TankAI
            {
                Controller = new TankController(Tank.Create(scene.Simulation, filters, frictions, scene.BufferPool, new RigidPose(start, orientation), tankDescription), 20, 5 * massMultiplier, 2, 1 * massMultiplier, 3.5f * massMultiplier),
                MachineGunTooter = supertooter
            };

        }

        targetTime = 36 + 16 * (float)random.NextDouble();
    }


    public bool Update(Scene scene, Random random, float accumulatedTime)
    {
        var aimMin = scene.RegionOfInterest.Min;
        var aimSpan = scene.RegionOfInterest.Max - aimMin;
        var movementRejectionMin = new Vector2(scene.RegionOfInterest.Min.X, scene.RegionOfInterest.Min.Z);
        var movementRejectionMax = new Vector2(scene.RegionOfInterest.Max.X, scene.RegionOfInterest.Max.Z);
        var movementMin = movementRejectionMin - new Vector2(20);
        var movementMax = movementRejectionMax + new Vector2(20);
        var movementSpan = movementMax - movementMin;
        for (int i = 0; i < tanks.Count; ++i)
        {
            tanks[i].Update(scene, random, accumulatedTime, movementMin, movementSpan, movementRejectionMin, movementRejectionMax, aimMin, aimSpan, filters, frictions);
        }
        return accumulatedTime < targetTime;
    }
}

