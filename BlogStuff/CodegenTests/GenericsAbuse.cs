using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace CodegenTests
{
    public static class GenericsAbuse
    {
        public static double Time(Action action, int runCount)
        {
            action();
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < runCount; ++i)
            {
                action();
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / ((double)runCount * Stopwatch.Frequency);
        }

        public struct Body
        {
            public float X;
            public float Y;
            public float Otherness;
            public float Ineffability;
        }

        interface IConstraint
        {
            int A { get; }
            int B { get; }
            void Apply(ref Body a, ref Body b);
        }

        struct IneffabilityConstraint : IConstraint
        {
            public int A { get; set; }

            public int B { get; set; }
            
            public void Apply(ref Body a, ref Body b)
            {
                a.Ineffability = a.Ineffability + b.Otherness * (b.X * b.X + b.Y * b.Y);
            }
        }

        class IneffabilityReferenceConstraint : IConstraint
        {
            public int A { get; set; }

            public int B { get; set; }

            public void Apply(ref Body a, ref Body b)
            {
                a.Ineffability = a.Ineffability + b.Otherness * (b.X * b.X + b.Y * b.Y);
            }
        }

        static void ApplyConstraintsThroughInterface(Span<Body> bodies, Span<IConstraint> constraints)
        {
            for (int i = 0; i < constraints.Length; ++i)
            {
                var constraint = constraints[i];
                constraint.Apply(ref bodies[constraint.A], ref bodies[constraint.B]);
            }
        }                      

        static void ApplyConstraintsWithGenericsAbuse<TConstraint>(Span<Body> bodies, Span<TConstraint> constraints) where TConstraint : IConstraint
        {
            for (int i = 0; i < constraints.Length; ++i)
            {
                ref var constraint = ref constraints[i];
                constraint.Apply(ref bodies[constraint.A], ref bodies[constraint.B]);
            }
        }

        public static double TestOopy()
        {
            var bodies = new Body[128];
            for (int i = 0; i < bodies.Length; ++i)
            {
                bodies[i] = new Body { X = i, Y = i * 2, Ineffability = 4, Otherness = 0.1f * MathF.Pow(i, 0.25f) };
            }

            var random = new Random(5);
            var constraints = new IConstraint[256];
            for (int i = 0; i < constraints.Length; ++i)
            {
                constraints[i] = new IneffabilityConstraint { A = random.Next(bodies.Length), B = random.Next(bodies.Length) };
            }

            return Time(() => ApplyConstraintsThroughInterface(bodies, constraints), 1<<20);
        }

        public static double TestAbuse()
        {
            var bodies = new Body[128];
            for (int i = 0; i < bodies.Length; ++i)
            {
                bodies[i] = new Body { X = i, Y = i * 2, Ineffability = 4, Otherness = 0.1f * MathF.Pow(i, 0.25f) };
            }

            var random = new Random(5);
            var constraints = new IneffabilityConstraint[256];
            for (int i = 0; i < constraints.Length; ++i)
            {
                constraints[i] = new IneffabilityConstraint { A = random.Next(bodies.Length), B = random.Next(bodies.Length) };
            }

            return Time(() => ApplyConstraintsWithGenericsAbuse<IneffabilityConstraint>(bodies, constraints), 1 << 20);
        }

        public static double TestAbuseWithReference()
        {
            var bodies = new Body[128];
            for (int i = 0; i < bodies.Length; ++i)
            {
                bodies[i] = new Body { X = i, Y = i * 2, Ineffability = 4, Otherness = 0.1f * MathF.Pow(i, 0.25f) };
            }

            var random = new Random(5);
            var constraints = new IneffabilityReferenceConstraint[256];
            for (int i = 0; i < constraints.Length; ++i)
            {
                constraints[i] = new IneffabilityReferenceConstraint { A = random.Next(bodies.Length), B = random.Next(bodies.Length) };
            }

            return Time(() => ApplyConstraintsWithGenericsAbuse<IneffabilityReferenceConstraint>(bodies, constraints), 1 << 20);
        }

    }
}
