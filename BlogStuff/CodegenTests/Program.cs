using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace CodegenTests
{
    class Program
    {      
        unsafe static void Main(string[] args)
        {
            var oopyTime = GenericsAbuse.TestOopy();
            Console.WriteLine($"Oopy time (us): {oopyTime * 1e6}");
            var abuseTime = GenericsAbuse.TestAbuse();
            Console.WriteLine($"abuse time (us): {abuseTime * 1e6}, speedup vs oopy: {oopyTime / abuseTime}");
            var abuseReferenceTime = GenericsAbuse.TestAbuseWithReference();
            Console.WriteLine($"abuse with references time (us): {abuseReferenceTime * 1e6}, speedup vs oopy: {oopyTime / abuseReferenceTime}");
        }
    }
}
