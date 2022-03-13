using System.Numerics;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Explicit)]
public struct NodeChild
{
    [FieldOffset(0)]
    public Vector3 Min;
    [FieldOffset(12)]
    public int Index;
    [FieldOffset(16)]
    public Vector3 Max;
    [FieldOffset(28)]
    public int LeafCount;
}

