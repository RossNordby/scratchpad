using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;

namespace HeadlessTests23.DemoStyle.Characters;

/// <summary>
/// Convenience structure that wraps a CharacterController reference and its associated body.
/// </summary>
/// <remarks>
/// <para>This should be treated as an example- nothing here is intended to suggest how you *must* handle characters. 
/// On the contrary, this does some fairly inefficient stuff if you're dealing with hundreds of characters in a predictable way.
/// It's just a fairly convenient interface for demos usage.</para>
/// <para>Note that all characters are dynamic and respond to constraints and forces in the simulation.</para>
/// </remarks>
public struct CharacterInput
{
    BodyHandle bodyHandle;
    CharacterControllers characters;
    float speed;
    Capsule shape;

    public BodyHandle BodyHandle { get { return bodyHandle; } }

    public CharacterInput(CharacterControllers characters, Vector3 initialPosition, Capsule shape,
        float minimumSpeculativeMargin, float mass, float maximumHorizontalForce, float maximumVerticalGlueForce,
        float jumpVelocity, float speed, float maximumSlope = MathF.PI * 0.25f)
    {
        this.characters = characters;
        var shapeIndex = characters.Simulation.Shapes.Add(shape);

        //Because characters are dynamic, they require a defined BodyInertia. For the purposes of the demos, we don't want them to rotate or fall over, so the inverse inertia tensor is left at its default value of all zeroes.
        //This is effectively equivalent to giving it an infinite inertia tensor- in other words, no torque will cause it to rotate.
        bodyHandle = characters.Simulation.Bodies.Add(
            BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1f / mass },
            new CollidableDescription(shapeIndex, float.MaxValue, ContinuousDetectionSettings.Passive), new (shape.Radius * 0.02f)));
        ref var character = ref characters.AllocateCharacter(bodyHandle);
        character.LocalUp = new Vector3(0, 1, 0);
        character.CosMaximumSlope = MathF.Cos(maximumSlope);
        character.JumpVelocity = jumpVelocity;
        character.MaximumVerticalForce = maximumVerticalGlueForce;
        character.MaximumHorizontalForce = maximumHorizontalForce;
        character.MinimumSupportDepth = shape.Radius * -0.01f;
        character.MinimumSupportContinuationDepth = -minimumSpeculativeMargin;
        this.speed = speed;
        this.shape = shape;
    }



    /// <summary>
    /// Removes the character's body from the simulation and the character from the associated characters set.
    /// </summary>
    public void Dispose()
    {
        characters.Simulation.Shapes.Remove(new BodyReference(bodyHandle, characters.Simulation.Bodies).Collidable.Shape);
        characters.Simulation.Bodies.Remove(bodyHandle);
        characters.RemoveCharacterByBodyHandle(bodyHandle);
    }
}


