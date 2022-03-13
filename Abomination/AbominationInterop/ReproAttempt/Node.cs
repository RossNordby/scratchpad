using System.Runtime.InteropServices;
//Note that the format of this node implies that we don't explicitly test against the root bounding box during normal execution.
//For almost all broad phase use cases, queries will be inside the root bounding box anyway. For non-broad phase uses, the outer bounding box will likely be stored
//elsewhere- for example, in the broad phase.

/// <summary>
/// 2-wide tree node.
/// </summary>
[StructLayout(LayoutKind.Explicit)]
public unsafe struct Node
{
    [FieldOffset(0)]
    public NodeChild A;
    [FieldOffset(32)]
    public NodeChild B;
}

