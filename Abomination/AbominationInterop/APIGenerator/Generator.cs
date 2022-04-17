using Microsoft.CSharp;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

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

    struct Function
    {
        public MethodInfo MethodInfo;
        public string TypePrefix;
    }

    List<Function> functions = new List<Function>();
    public void AddFunction(string typePrefix, MethodInfo function)
    {
        functions.Add(new() { TypePrefix = typePrefix, MethodInfo = function });
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

    string GetFriendlyCSharpTypeName(Type type)
    {
        return new CSharpCodeProvider().GetTypeOutput(new System.CodeDom.CodeTypeReference(type));
    }

    string GetExposedFunctionName(Function function)
    {
        ReadOnlySpan<char> name = function.MethodInfo.Name;
        if (function.MethodInfo.IsSpecialName)
        {
            //This is hacky, but it works given that no functions in bepuphysics2 violate the required assumptions.
            if (name.StartsWith("get_"))
            {
                return string.Concat(prefix, function.TypePrefix, "Get", name[4..]);
            }
            if (name.StartsWith("set_"))
            {
                return string.Concat(prefix, function.TypePrefix, "Get", name[4..]);
            }
        }
        return string.Concat(prefix, function.TypePrefix, name);
    }

    private void WriteCSharpFunction(string indent, Function function, List<string> lines)
    {
        string exposedFunctionName = GetExposedFunctionName(function);

        lines.Add($"{indent}[UnmanagedCallersOnly(CallConvs = new[] {{ typeof({callingConvention}) }}, EntryPoint = \"{exposedFunctionName}\")]");
        StringBuilder builder = new StringBuilder();
        builder.Append(indent).Append("public static ").Append(GetFriendlyCSharpTypeName(function.MethodInfo.ReturnType)).Append(' ').Append(exposedFunctionName).Append('(');
        var parameters = function.MethodInfo.GetParameters();
        for (int i = 0; i < parameters.Length; ++i)
        {
            var parameter = parameters[i];
            builder.Append(GetFriendlyCSharpTypeName(parameter.ParameterType)).Append(' ').Append(parameter.Name);
            if (i < parameters.Length - 1)
                builder.Append(", ");
        }
        builder.Append(')');
        lines.Add(builder.ToString());
        lines.Add($"{indent}{{");

        lines.Add($"{indent}}}");
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
        foreach (var function in functions)
        {
            WriteCSharpFunction(indent, function, lines);
        }
        lines.Add("}");
        var v = new StreamWriter(csharpStream);
        Write(v, CollectionsMarshal.AsSpan(lines));
    }
}
