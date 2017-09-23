using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    public static class CollidablePairComparerTests
    {
        public static void Test()
        {
            var random = new Random(5);
            var comparer = new CollidablePairComparer();
            for (int i = 0; i < 10000; ++i)
            {
                var a = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                var b = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                var pair1 = new CollidablePair(a, b);
                var pair2 = new CollidablePair(b, a);
                Debug.Assert(comparer.Hash(ref pair1) == comparer.Hash(ref pair2));
                Debug.Assert(comparer.Equals(ref pair1, ref pair2));
            }
            for (int i = 0; i < 10000; ++i)
            {
                var a = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                var b = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                var pair1 = new CollidablePair(a, b);
                CollidablePair pair2;
                do
                {
                    var a2 = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                    var b2 = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
                    pair2 = new CollidablePair(a2, b2);
                } while (
                (pair2.A.Packed == pair1.A.Packed && pair2.B.Packed == pair1.B.Packed) ||
                (pair2.B.Packed == pair1.A.Packed && pair2.A.Packed == pair1.B.Packed));
                Debug.Assert(!comparer.Equals(ref pair1, ref pair2));
            }
        }
    }
}
