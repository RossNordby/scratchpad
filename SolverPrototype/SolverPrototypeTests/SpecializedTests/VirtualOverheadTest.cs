using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    public static class VirtualOverheadTest
    {
        abstract class Superclass
        {
            public abstract void Do(ref int i);
        }
        class Increment : Superclass
        {
            public override void Do(ref int i)
            {
                ++i;
            }
        }
        class Decrement : Superclass
        {
            public override void Do(ref int i)
            {
                --i;
            }
        }
        class LShift : Superclass
        {
            public override void Do(ref int i)
            {
                i <<= 1;
            }
        }
        class RShift : Superclass
        {
            public override void Do(ref int i)
            {
                i >>= 1;
            }
        }

        enum ExecutionType
        {
            Increment = 0, Decrement = 1, LShift = 2, RShift = 3
        }
        struct Switch
        {
            public ExecutionType Type;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do(ref int i)
            {
                switch (Type)
                {
                    case ExecutionType.Increment:
                        Increment(ref i);
                        break;
                    case ExecutionType.Decrement:
                        Decrement(ref i);
                        break;
                    case ExecutionType.LShift:
                        LShift(ref i);
                        break;
                    case ExecutionType.RShift:
                        RShift(ref i);
                        break;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Increment(ref int i)
            {
                ++i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Decrement(ref int i)
            {
                --i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void LShift(ref int i)
            {
                i <<= 1;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void RShift(ref int i)
            {
                i >>= 1;
            }
        }
        
        public static void Test()
        {
            const int invocationTestCount = 256;
            const int iterations = 10000;
            var switches = new Switch[invocationTestCount];
            var virtuals = new Superclass[invocationTestCount];
            Random random = new Random();
            for (int i = 0; i < invocationTestCount; ++i)
            {
                var executionType = (ExecutionType)random.Next(4);
                switches[i].Type = executionType;
                switch (executionType)
                {
                    case ExecutionType.Increment:
                        virtuals[i] = new Increment();
                        break;
                    case ExecutionType.Decrement:
                        virtuals[i] = new Decrement();
                        break;
                    case ExecutionType.LShift:
                        virtuals[i] = new LShift();
                        break;
                    case ExecutionType.RShift:
                        virtuals[i] = new RShift();
                        break;

                }
                switches[i] = new Switch();
            }
            int switchValue = 0;
            int virtualValue = 0;
            //Warmup.
            for (int i = 0; i < invocationTestCount; ++i)
            {
                switches[i].Do(ref switchValue);
                virtuals[i].Do(ref virtualValue);
            }
            var switchStart = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < invocationTestCount; ++j)
                {
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);

                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                }
            }
            var virtualStart = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < invocationTestCount; ++j)
                {
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);

                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                }
            }
            var virtualEnd = Stopwatch.GetTimestamp();
            Console.WriteLine($"Switch time (ns): {1e9 * (virtualStart - switchStart) / (Stopwatch.Frequency * invocationTestCount * iterations)}");
            Console.WriteLine($"Virtual time (ns): {1e9 * (virtualEnd - virtualStart) / (Stopwatch.Frequency * invocationTestCount * iterations)}");
            Console.WriteLine($"Switch accumulator: {switchValue}, virtual accumulator: {virtualValue}");
        }
    }
}
