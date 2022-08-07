
using AbominationInterop;
using System.CodeDom;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

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



    public static void Emit()
    {
        var methods = typeof(Entrypoints).GetMethods();
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

                Console.WriteLine(signature);
            }
        }
    }
}
