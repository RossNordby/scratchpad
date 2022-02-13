using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace HeadlessTests23;

public static class TestHelpers
{
    /// <summary>
    /// Gets a value roughly representing the amount of energy in the simulation. This is occasionally handy for debug purposes.
    /// </summary>
    public static float GetBodyEnergyHeuristic(Bodies bodies)
    {
        float accumulated = 0;
        for (int index = 0; index < bodies.ActiveSet.Count; ++index)
        {
            ref var velocity = ref bodies.ActiveSet.Velocities[index];
            accumulated += Vector3.Dot(velocity.Linear, velocity.Linear);
            accumulated += Vector3.Dot(velocity.Angular, velocity.Angular);
        }
        return accumulated;
    }
    public static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
    {
        RigidPose pose;
        var span = positionBounds.Max - positionBounds.Min;

        pose.Position = positionBounds.Min + span * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
        var axis = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
        var length = axis.Length();
        if (length > 0)
            axis /= length;
        else
            axis = new Vector3(0, 1, 0);
        pose.Orientation = BepuUtilities.QuaternionEx.CreateFromAxisAngle(axis, 1203f * random.NextSingle());
        return pose;
    }


    public static void CreateDeformedPlane(int width, int height, Func<int, int, Vector3> deformer, Vector3 scaling, BufferPool pool, out Mesh mesh)
    {
        pool.Take<Vector3>(width * height, out var vertices);
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                vertices[width * j + i] = deformer(i, j);
            }
        }

        var quadWidth = width - 1;
        var quadHeight = height - 1;
        var triangleCount = quadWidth * quadHeight * 2;
        pool.Take<Triangle>(triangleCount, out var triangles);

        for (int i = 0; i < quadWidth; ++i)
        {
            for (int j = 0; j < quadHeight; ++j)
            {
                var triangleIndex = (j * quadWidth + i) * 2;
                ref var triangle0 = ref triangles[triangleIndex];
                ref var v00 = ref vertices[width * j + i];
                ref var v01 = ref vertices[width * j + i + 1];
                ref var v10 = ref vertices[width * (j + 1) + i];
                ref var v11 = ref vertices[width * (j + 1) + i + 1];
                triangle0.A = v00;
                triangle0.B = v01;
                triangle0.C = v10;
                ref var triangle1 = ref triangles[triangleIndex + 1];
                triangle1.A = v01;
                triangle1.B = v11;
                triangle1.C = v10;
            }
        }
        pool.Return(ref vertices);
        mesh = new Mesh(triangles, scaling, pool);
    }

}
