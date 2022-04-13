using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

public class Generator
{
    string prefix;
    string callingConvention;
    public Generator(string prefix, CallingConvention convention)
    {
        //toot.
        callingConvention = convention switch
        {
            CallingConvention.Cdecl => nameof(CallConvCdecl),
            CallingConvention.StdCall => nameof(CallConvStdcall),
            _ => throw new NotSupportedException("Only CDecl and Stdcall calling conventions are currently supported.")
        };
        this.prefix = prefix;
    }

    List<Type> typesRequiringDirectories = new List<Type>();
    public void RegisterInstanceDirectory<T>() where T : class
    {
        typesRequiringDirectories.Add(typeof(T));
    }

    public void AddFunction(MethodInfo function)
    {
        function.
    }

    private void Write(StreamWriter writer, Span<string> lines)
    {
        foreach (var line in lines)
        {
            if (line == null)
                writer.WriteLine();
            else
                writer.WriteLine(line);
        }
    }

    private void WriteFunction(string indent, MethodInfo methodInfo, List<string> lines)
    {
        var exposedFunctionName = prefix + methodInfo.Name;
        lines.Add($"{indent}[UnmanagedCallersOnly(CallConvs = new[] {{ { callingConvention } }}, EntryPoint = \"{exposedFunctionName}\")]");
    }
    public void WriteCSharp(Stream csharpStream, string entryPointsNamespace, string entryPointsClassName)
    {
        List<string> lines = new();
        lines.AddRange(new[] {
            "using System.Runtime.CompilerServices;",
            "using System.Runtime.InteropServices;",
            "using APIGenerator.InstanceDirectory;",
            $"namespace {entryPointsNamespace};",
            $"public static class {entryPointsClassName}",
            "{"});
        const string indent = "    ";
        for (int typeIndex = 0; typeIndex < typesRequiringDirectories.Count; ++typeIndex)
        {
            var type = typesRequiringDirectories[typeIndex];
            lines.Add($"{indent}public static InstanceDirectory<{type.Name}> instancesOf{type.Name} = new InstanceDirectory<{type.Name}>({typeIndex});");
        }
        lines.Add("}");
        var v = new StreamWriter(csharpStream);
        Write(v, CollectionsMarshal.AsSpan(lines));
    }
}
