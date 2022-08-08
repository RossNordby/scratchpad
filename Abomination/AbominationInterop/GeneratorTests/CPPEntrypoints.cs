
using AbominationInterop;
using System.CodeDom;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace GeneratorTests;
public static class CPPEntrypoints
{
    static string MapTypeNamesCPP(Type type)
    {
        return type switch
        {
            Type when type == typeof(int) => "int32_t",
            Type when type == typeof(uint) => "uint32_t",
            Type when type == typeof(float) => "float",
            Type when type == typeof(void) => "void",
            Type when type == typeof(bool) => "bool",
            _ => type.Name
        };
    }

    static string GetReturnTypeName(Type type, object[] attributes = null)
    {
        if (attributes != null)
        {
            foreach (var attribute in attributes)
            {
                if (attribute is TypeNameAttribute typeNameAttribute)
                {
                    return typeNameAttribute.TypeName;
                }
            }
        }
        return MapTypeNamesCPP(type);
    }

    static string GetParameterTypeName(Type type, IEnumerable<CustomAttributeData> attributes = null)
    {
        if (attributes != null)
        {
            foreach (var attribute in attributes)
            {
                if (attribute.AttributeType == typeof(TypeNameAttribute))
                {
                    return (string)attribute.ConstructorArguments[0].Value;
                }
            }
        }
        return MapTypeNamesCPP(type);
    }

    static void AccumulateFunctionDocumentation(string path, Dictionary<string, List<string>> functionDocumentation)
    {
        using var reader = new StreamReader(path);
        string? line;
        //the wonders of being able to do hideously slow things and it doesn't matter at all
        List<string> lines = new List<string>();
        while ((line = reader.ReadLine()) != null)
        {
            lines.Add(line.TrimStart());
        }
        for (int i = 0; i < lines.Count; ++i)
        {
            line = lines[i];
            //The entrypoints files have a fairly regular structure, so we can do some hackystuff.
            if (line.StartsWith("[UnmanagedCallersOnly"))
            {
                //This should be one of the interop functions.
                //Iterate forward to find the function name.
                string? functionName = null;
                for (int j = 1; j <= 2; ++j)
                {
                    var candidateNameLine = lines[i + j];
                    if (candidateNameLine.StartsWith("public"))
                    {
                        var exclusiveEndIndex = candidateNameLine.IndexOf('(');
                        var inclusiveStartIndex = candidateNameLine.LastIndexOf(' ', exclusiveEndIndex) + 1;
                        functionName = candidateNameLine.Substring(inclusiveStartIndex, exclusiveEndIndex - inclusiveStartIndex);
                        break;
                    }
                }
                if (functionName == null)
                {
                    Debug.Fail("Hey! your assumptions about comment formatting in the entrypoints are wrong!");
                    continue;
                }
                //Iterate backward to find all comments.
                int offset = 0;
                while (true)
                {
                    if (lines[i - offset - 1].StartsWith("///"))
                        ++offset;
                    else
                        break;
                }
                var commentLines = new List<string>();
                for (int j = i - offset; j < i; ++j)
                {
                    commentLines.Add(lines[j]);
                }

                functionDocumentation.Add(functionName, commentLines);
            }
        }
    }

    static string HackilyFindSolutionDirectory()
    {
        var current = new DirectoryInfo(Directory.GetCurrentDirectory());
        while (current != null && current.GetFiles("*.sln").Length == 0)
        {
            current = current.Parent;
        }
        if (current == null)
            throw new InvalidOperationException("Hacky solution finder assumptions violated!");
        return current.FullName;
    }

    public static void Emit()
    {
        var entrypointsDirectory = HackilyFindSolutionDirectory() + "/AbominationInterop/";
        Dictionary<string, List<string>> functionComments = new();
        AccumulateFunctionDocumentation(entrypointsDirectory + "Entrypoints.cs", functionComments);
        AccumulateFunctionDocumentation(entrypointsDirectory + "Entrypoints_Shapes.cs", functionComments);

        var methods = typeof(Entrypoints).GetMethods();

        var builder = new StringBuilder(32768);

        foreach (var method in methods)
        {
            string name = null;
            foreach (var attribute in method.CustomAttributes)
            {
                if (attribute.AttributeType == typeof(UnmanagedCallersOnlyAttribute))
                {
                    foreach (var argument in attribute.NamedArguments)
                    {
                        if (argument.MemberName == "EntryPoint")
                        {
                            name = (string)argument.TypedValue.Value;
                        }
                    }
                    break;
                }
            }
            if (name != null)
            {
                var signature = $"extern \"C\" {GetReturnTypeName(method.ReturnType, method.ReturnTypeCustomAttributes.GetCustomAttributes(true))} {name}(";
                var parameters = method.GetParameters();
                for (int i = 0; i < parameters.Length; ++i)
                {
                    var parameter = parameters[i];
                    signature += $"{GetParameterTypeName(parameter.ParameterType, parameter.CustomAttributes)} {parameter.Name}";
                    if (parameter.HasDefaultValue)
                    {
                        signature += parameter.DefaultValue switch
                        {
                            int value => $" = {value}",
                            float value => $" = {value}f",
                            _ => ""
                        };
                    }
                    if (i < parameters.Length - 1)
                        signature += ", ";
                }
                signature += ");";

                var comments = functionComments[method.Name];
                foreach (var comment in comments)
                {
                    builder.AppendLine(comment);
                }

                builder.AppendLine(signature);
            }
        }

        var built = builder.ToString();
        Console.WriteLine(built);
        using (var writer = new StreamWriter("cpp.txt"))
        {
            writer.Write(built);
        }
    }
}
