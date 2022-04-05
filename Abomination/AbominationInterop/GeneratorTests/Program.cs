
using BepuPhysics;
using System.Reflection;


var type = typeof(Solver);

//static void ExposeField()

var fields = type.GetFields(BindingFlags.Public | BindingFlags.Instance);

foreach (var field in fields)
{
    Console.WriteLine(field);
}
