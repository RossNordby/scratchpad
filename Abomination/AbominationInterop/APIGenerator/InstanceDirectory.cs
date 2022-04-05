using BepuUtilities.Memory;
using System.Diagnostics;
/// <summary>
/// Registers handles for GC-tracked types to be exposed through the C API.
/// </summary>
/// <remarks>GC handles are not directly exposed; instead, an integer representation is used as a handle and is used to look up the GC-managed instance.</remarks>
public class InstanceDirectory<T> where T : class
{
    struct DirectoryEntry
    {
        public T Instance;
        public int Version;
    }
    DirectoryEntry[] instances;
    ManagedIdPool pool;
    int typeIndex;

    public T this[InstanceHandle handle]
    {
        get
        {
            //Just for the sake of performance, the verification will be kept in debug mode.
            //This does imply that the C API will be shipped with a debug version...
            Debug.Assert(handle.TypeIndex == typeIndex, "Handle type must match the type of this instance directory. Did a bad handle get passed in?");
            Debug.Assert(handle.Index >= 0 && handle.Index < instances.Length, "Handle index must point to a slot within the directory. Did a bad handle get passed in?");
            Debug.Assert(handle.Version == instances[handle.TypeIndex].Version, "Handle version must match directory contained version. Did a handle get used after being removed?");
            Debug.Assert(instances[handle.Index].Instance != null, "There must be an instance associated with a handle.");
            return instances[handle.Index].Instance;
        }
    }

    public InstanceDirectory(int typeIndex, int initialCapacity = 32)
    {
        this.typeIndex = typeIndex;
        instances = new DirectoryEntry[initialCapacity];
        pool = new ManagedIdPool(initialCapacity);
    }

    /// <summary>
    /// Adds an instance to the directory, returning a handle.
    /// </summary>
    /// <param name="instance">Instance to add.</param>
    /// <returns>Handle to the instance.</returns>
    public InstanceHandle Add(T instance)
    {
        var index = pool.Take();
        ref var slot = ref instances[index];
        slot.Instance = instance;
        return new InstanceHandle(index, slot.Version++, typeIndex);
    }
    /// <summary>
    /// Removes the instance associated with the given handle.
    /// </summary>
    /// <param name="handle">Handle of the instance to remove.</param>
    public void Remove(InstanceHandle handle)
    {
        if (handle.TypeIndex != typeIndex)
            throw new ArgumentException("Handle does not match the type of this instance directory.");
        if (handle.Index < 0 || handle.Index > instances.Length)
            throw new ArgumentOutOfRangeException("Handle points to an index outside of the instance directory.");
        if (handle.Version != instances[handle.TypeIndex].Version)
            throw new ArgumentException("Handle is out of date. Is a handle being used after being removed?");
        if (instances[handle.Index].Instance == null)
            throw new ArgumentException("There is no instance associated with this handle.");
        instances[handle.Index].Instance = null;
        pool.Return(handle.Index);
    }
}
