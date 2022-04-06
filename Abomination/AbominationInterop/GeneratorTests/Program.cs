
using BepuPhysics;
using BepuUtilities.Memory;
using System.Reflection;


var generator = new Generator("bepu");
generator.RegisterInstanceDirectory<BufferPool>();
generator.RegisterInstanceDirectory<Simulation>();

var simulation = typeof(Simulation);
foreach (var method in simulation.GetMethods(BindingFlags.Public | BindingFlags.Instance))
{
    generator.AddFunction(method);
}