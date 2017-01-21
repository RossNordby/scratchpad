using BEPUutilities2;
using SolverPrototype;
using System.Numerics;

namespace SolverPrototypeTests
{
    static class BodyStackBuilder
    {
        public static Bodies BuildStackOfBodiesOnGround(int bodyCount, out int[] handleIndices)
        {
            Bodies bodies = new Bodies();
            handleIndices = new int[bodyCount];
            //Body 0 is a stationary kinematic acting as the ground.
            {
                var description = new BodyDescription();
                var handleIndex = bodies.Add(ref description);
                handleIndices[0] = handleIndex;
            }

            //The remaining bodies form an extremely tall stack. 
            for (int i = 1; i < bodyCount; ++i)
            {
                var description = new BodyDescription
                {
                    LocalInertia = new BodyInertia
                    {
                        //InverseInertiaTensor = new Matrix3x3
                        //{
                        //    X = new Vector3(1, 0, 0),
                        //    Y = new Vector3(0, 1, 0),
                        //    Z = new Vector3(0, 0, 1),
                        //},
                        InverseMass = 1
                    },
                };
                var handleIndex = bodies.Add(ref description);
                handleIndices[i] = handleIndex;

            }
            return bodies;
        }
    }
}

