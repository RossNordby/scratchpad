
//using BepuPhysics;
//using BepuUtilities.Memory;
//using System.Reflection;
//using System.Runtime.InteropServices;

//var generator = new Generator("bepu", CallingConvention.Cdecl);
//generator.RegisterInstanceDirectory<BufferPool>();
//generator.RegisterInstanceDirectory<Simulation>();

//var simulation = typeof(Simulation);
//foreach (var method in simulation.GetMethods(BindingFlags.Public | BindingFlags.Instance))
//{
//    generator.AddFunction("Simulation", method);
//}

//generator.WriteCSharp(Console.OpenStandardOutput(), "crabs", "Crablord");

using GeneratorTests;

CPPEntrypoints.Emit();