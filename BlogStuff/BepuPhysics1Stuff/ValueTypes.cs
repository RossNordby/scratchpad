using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics1Stuff
{
    public static class ValueTypes
    {
        struct Snoot
        {
            public int A;
            public float B;
            public long C;
        }

        public static void Boop()
        {
            var snoots = new Snoot[4];
            for (int i = 0; i < snoots.Length; ++i)
            {
                ref var snoot = ref snoots[i];
                var value = i + 1;
                snoot.A = value;
                snoot.B = value;
                snoot.C = value;
            }

            ref var snootBytes = ref Unsafe.As<Snoot, byte>(ref snoots[0]);
            for (int i = 0; i < snoots.Length; ++i)
            {
                Console.Write($"Snoot {i} bytes: ");
                for (int j = 0; j < Unsafe.SizeOf<Snoot>(); ++j)
                {
                    Console.Write($"{Unsafe.Add(ref snootBytes, i * Unsafe.SizeOf<Snoot>() + j)}, ");
                }
                Console.WriteLine();
            }
        }
    }
}
