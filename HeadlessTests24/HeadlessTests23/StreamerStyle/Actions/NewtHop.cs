using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using System.Numerics;

namespace HeadlessTests23.StreamerStyle.Actions;

public struct HoppyNewt
{
    public BodyHandle BodyHandle;
    public HoppyNewt(Simulation simulation, TypedIndex shape, float height, in Vector2 rejectMin, in Vector2 rejectMax, in Vector2 arenaMin, in Vector2 arenaMax, Random random) : this()
    {
        var arenaSpan = arenaMax - arenaMin;

        Vector2 position;
        do
        {
            position = arenaMin + arenaSpan * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
        } while (Vector2.Max(rejectMin, Vector2.Min(rejectMax, position)) == position);
        var offset = (rejectMin + rejectMax) * 0.5f - position;
        var angle = MathF.Atan2(offset.X, offset.Y) + MathF.PI;
        BodyHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(new Vector3(position.X, height, position.Y), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle)), new (shape, float.MaxValue), new (-1)));
    }

    //The newt hops between predetermined points unstoppably, waiting a moment in between jumps.
    double nextAllowedJump;
    Vector2 jumpStart, jumpEnd;
    //The orientation will interpolate while jumping.
    Vector2 forwardAtJumpStart, forwardAtJumpEnd;
    double jumpStartTime, jumpEndTime;

    public void Update(Simulation simulation, double time, float height, in Vector2 arenaMin, in Vector2 arenaMax, Random random, float inverseDt)
    {
        const float jumpDuration = 1;
        var body = simulation.Bodies.GetBodyReference(BodyHandle);
        if (time >= nextAllowedJump)
        {
            //Choose a jump location. It should be within the arena, and generally somewhere ahead of the newt.
            QuaternionEx.TransformUnitZ(body.Pose.Orientation, out var backward);
            jumpStart = new Vector2(body.Pose.Position.X, body.Pose.Position.Z);
            jumpEnd = jumpStart + new Vector2(backward.X + (float)random.NextDouble() * 1.4f - 0.7f, backward.Z + (float)random.NextDouble() * 1.4f - 0.7f) * -20;
            jumpEnd -= jumpStart * 0.05f;
            jumpEnd = Vector2.Max(arenaMin, Vector2.Min(arenaMax, jumpEnd));
            jumpStartTime = time;
            jumpEndTime = time + jumpDuration;
            forwardAtJumpStart = -new Vector2(backward.X, backward.Z);
            forwardAtJumpEnd = jumpEnd - jumpStart;
            var newForwardLengthSquared = forwardAtJumpEnd.LengthSquared();
            forwardAtJumpEnd = newForwardLengthSquared < 1e-10f ? forwardAtJumpStart : forwardAtJumpEnd / MathF.Sqrt(newForwardLengthSquared);
            nextAllowedJump = jumpEndTime + (1 + random.NextDouble() * 2.5f);
        }
        Vector3 targetPosition;
        Vector2 targetForward;
        if (time >= jumpStartTime && time <= jumpEndTime)
        {
            //The newt's in the middle of a jump. Choose a target position/orientation by interpolation.
            const float maximumJumpHeight = 5;
            var jumpProgress = (float)(time - jumpStartTime) / jumpDuration;
            var targetPosition2D = (float)jumpProgress * (jumpEnd - jumpStart) + jumpStart;
            var parabolaTerm = (2 * jumpProgress - 1);
            var currentHeight = height + (1 - parabolaTerm * parabolaTerm) * maximumJumpHeight;
            targetPosition = new Vector3(targetPosition2D.X, currentHeight, targetPosition2D.Y);
            targetForward = jumpProgress * (forwardAtJumpEnd - forwardAtJumpStart) + forwardAtJumpStart;
            var targetForwardLengthSquared = targetForward.LengthSquared();
            if (targetForwardLengthSquared < 1e-10f)
            {
                QuaternionEx.TransformUnitZ(body.Pose.Orientation, out var backward);
                targetForward = -new Vector2(backward.X, backward.Z);
            }
            else
            {
                targetForward /= MathF.Sqrt(targetForwardLengthSquared);
            }
        }
        else
        {
            //The target pose is just wherever the previous jump ended.
            targetPosition = new Vector3(jumpEnd.X, height, jumpEnd.Y);
            targetForward = forwardAtJumpEnd;
        }

        //Since it's a kinematic body, we'll compute the current pose error, and then the velocity to correct that error within a single frame.
        body.Velocity.Linear = (targetPosition - body.Pose.Position) * inverseDt;
        Matrix3x3 targetOrientationBasis;
        targetOrientationBasis.X = new Vector3(-targetForward.Y, 0, targetForward.X);
        targetOrientationBasis.Y = Vector3.UnitY;
        targetOrientationBasis.Z = -new Vector3(targetForward.X, 0, targetForward.Y);
        QuaternionEx.CreateFromRotationMatrix(targetOrientationBasis, out var targetOrientation);
        QuaternionEx.GetRelativeRotationWithoutOverlap(body.Pose.Orientation, targetOrientation, out var orientationError);
        QuaternionEx.GetAxisAngleFromQuaternion(orientationError, out var errorAxis, out var errorAngle);
        body.Velocity.Angular = errorAxis * (errorAngle * inverseDt);
    }
}
public class NewtHop : IAction
{
    float targetTime;

    QuickList<HoppyNewt> newts;

    public void Initialize(Random random, Scene scene)
    {
        var newtCount = random.Next(1, 6);

        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        Mesh templateMesh;
        using (var stream = File.Open(@"Content\newt.obj", FileMode.Open))
        {
            templateMesh = MeshLoader.LoadMesh(stream, Vector3.One, scene.BufferPool);
        }
        newts = new QuickList<HoppyNewt>(newtCount, scene.Simulation.BufferPool);

        Span<float> scales = stackalloc float[newtCount];
        var newtBoundsMin = Vector3.Min(templateMesh.Tree.Nodes[0].A.Min, templateMesh.Tree.Nodes[0].B.Min);
        var newtBoundsMax = Vector3.Max(templateMesh.Tree.Nodes[0].A.Max, templateMesh.Tree.Nodes[0].B.Max);
        var templateNewtSize = Vector3.Distance(newtBoundsMin, newtBoundsMax);

        for (int i = 0; i < newtCount; ++i)
        {
            var mesh = templateMesh;
            //sometimes, a very large newt.
            scales[i] = random.NextDouble() < 0.02 ? 50 + 100 * (float)random.NextDouble() : 5 + 10 * (float)random.NextDouble();
            mesh.Scale = new Vector3(-1, 1, -1) * new Vector3(scales[i]);

            var newtSize = scales[i] * templateNewtSize;
            var rejectMin = new Vector2(scene.RegionOfInterest.Min.X, scene.RegionOfInterest.Min.Z) - new Vector2(newtSize);
            var rejectMax = new Vector2(scene.RegionOfInterest.Max.X, scene.RegionOfInterest.Max.Z) + new Vector2(newtSize);
            var startMin = rejectMin * 1.2f;
            var startMax = rejectMax * 1.2f;

            newts.AllocateUnsafely() = new HoppyNewt(scene.Simulation, scene.Simulation.Shapes.Add(mesh), 0, rejectMin, rejectMax, startMin, startMax, random);
        }

        targetTime = 30 + (float)random.NextDouble() * 10;
    }


    public bool Update(Scene scene, Random random, float accumulatedTime)
    {
        var inverseDt = 1f / scene.TimestepDuration;
        var arenaMin = new Vector2(scene.RegionOfInterest.Min.X, scene.RegionOfInterest.Min.Z);
        var arenaMax = new Vector2(scene.RegionOfInterest.Max.X, scene.RegionOfInterest.Max.Z);
        for (int i = 0; i < newts.Count; ++i)
        {
            newts[i].Update(scene.Simulation, accumulatedTime, 0, arenaMin, arenaMax, random, inverseDt);
        }
        return accumulatedTime < targetTime;
    }
}

