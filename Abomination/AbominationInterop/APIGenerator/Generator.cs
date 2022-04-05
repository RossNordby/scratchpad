public class Generator
{
    List<Type> typesRequiringDirectories = new List<Type>();
    public void RegisterInstanceDirectory<T>() where T : class
    {
        typesRequiringDirectories.Add(typeof(T));
    }
}
