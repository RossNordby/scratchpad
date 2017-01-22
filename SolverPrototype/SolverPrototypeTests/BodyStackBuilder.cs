using BEPUutilities2;
using SolverPrototype;
using System;
using System.Numerics;

namespace SolverPrototypeTests
{
    static class BodyStackBuilder
    {
        public static Bodies BuildStackOfBodiesOnGround(int bodyCount, bool scrambled, out int[] handleIndices)
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
            if(scrambled)
            {
                //Having every single body in order is pretty unrealistic. In a real application, churn and general lack of care will result in 
                //scrambled body versus constraint memory access patterns. That's a big increase in cache misses.
                //Scrambling the body array simulates this.
                //Given a sufficiently large added overhead, it would benefit the engine to include runtime cache optimization.
                //That is, move the memory location of bodies (and constraints, within type batches) to maximize the number of accesses to already-cached bodies.

                Random random = new Random(5);
                for (int i = bodies.BodyCount - 1; i >= 0; --i)
                {
                    bodies.Swap(i, random.Next(i));
                }

            }
            return bodies;
        }
    }
}

