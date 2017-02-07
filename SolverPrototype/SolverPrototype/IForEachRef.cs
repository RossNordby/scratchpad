namespace SolverPrototype
{
    //Who's up for some super idiomatic C#?
    //These types are typically implemented by value types and passed into various performance sensitive functions.
    //By using generic functions with value types, type specialization is forced and inlining becomes possible.
    //Further, in many cases the creation of a temporary array can be avoided.

    /// <summary>
    /// Defines a loop body function able to handle the equivalent of a foreach's body. Takes a ref parameter for efficiency when dealing with large value types.
    /// </summary>
    /// <typeparam name="T">Type to enumerate over.</typeparam>
    public interface IForEachRef<T>
    {
        void LoopBody(ref T i);
    }
    /// <summary>
    /// Defines a loop body function able to handle the equivalent of a foreach's body.
    /// </summary>
    /// <typeparam name="T">Type to enumerate over.</typeparam>
    public interface IForEach<T>
    {
        void LoopBody(T i);
    }
}
