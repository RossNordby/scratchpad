using System.Runtime.InteropServices;
//Node metadata isn't required or used during collision testing, so it is stored separately.
//This helps avoid splitting Nodes across cache lines and decreases memory bandwidth requirements during testing.
/// <summary>
/// Metadata associated with a 2-child tree node.
/// </summary>
[StructLayout(LayoutKind.Explicit)]
public unsafe struct Metanode
{
    [FieldOffset(0)]
    public int Parent;
    [FieldOffset(4)]
    public int IndexInParent;
    [FieldOffset(8)]
    public int RefineFlag;
    /// <summary>
    /// Cached change in cost of the tree starting at this node since the previous frame.
    /// The local cost change is unioned with the refine flags. They're never used simultaneously.
    /// This will be overwritten right after use, so don't expect anything meaningful here outside of refinement scheduling's scope.
    /// </summary>
    [FieldOffset(8)]
    public float LocalCostChange;

}

