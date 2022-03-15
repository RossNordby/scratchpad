using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace AbominationInterop
{
    internal static class Entrypoints
    {
        [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = "Goingtr")]
        public static void Greetings()
        {
            Console.WriteLine("Proing");
        }
    }
}
