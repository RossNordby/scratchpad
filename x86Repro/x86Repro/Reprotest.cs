using System;
using System.Numerics;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace x86Repro;

unsafe class Reprotest
{
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
    [StructLayout(LayoutKind.Explicit)]
    public struct Node
    {
        [FieldOffset(0)]
        public NodeChild A;
        [FieldOffset(32)]
        public NodeChild B;
    }

    public struct Buffer<T> where T : unmanaged
    {
        public T* Memory;
        public int Length;
        public Buffer(T* memory, int length)
        {
            Memory = memory;
            Length = length;
        }

        public ref T this[int index]
        {
            get
            {
                Debug.Assert(index >= 0 && index < Length, "hey that's no good");
                return ref Memory[index];
            }
        }
    }

    public struct TreeRay
    {
        public Vector3 OriginOverDirection;
        public float MaximumT;
        public Vector3 InverseDirection;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFrom(Vector3 origin, Vector3 direction, float maximumT, out TreeRay treeRay)
        {
            treeRay.InverseDirection = new Vector3(direction.X < 0 ? -1 : 1, direction.Y < 0 ? -1 : 1, direction.Z < 0 ? -1 : 1) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(direction));
            treeRay.MaximumT = maximumT;
            treeRay.OriginOverDirection = origin * treeRay.InverseDirection;
        }
    }

    public struct Reprotree
    {
        public int LeafCount;
        public int NodeCount;
        public Buffer<Node> Nodes;

        //NOTE: Removing aggressive inlining avoids the issue.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool Intersects(Vector3 min, Vector3 max, TreeRay* ray, out float t)
        {
            var t0 = min * ray->InverseDirection - ray->OriginOverDirection;
            var t1 = max * ray->InverseDirection - ray->OriginOverDirection;
            var tExit = Vector3.Max(t0, t1);
            var tEntry = Vector3.Min(t0, t1);
            var earliestExit = Vector4.Min(Vector4.Min(new Vector4(ray->MaximumT), new Vector4(tExit.X)), Vector4.Min(new Vector4(tExit.Y), new Vector4(tExit.Z))).X;
            t = Vector4.Max(Vector4.Max(new Vector4(tEntry.X), Vector4.Zero), Vector4.Max(new Vector4(tEntry.Y), new Vector4(tEntry.Z))).X;
            return t <= earliestExit;
        }

        public static int Encode(int index)
        {
            return -1 - index;
        }

        //NOTE: Disabling optimization avoids the issue.
        //[MethodImpl(MethodImplOptions.NoOptimization)]
        public readonly unsafe void Sweep(int nodeIndex, Vector3 expansion, TreeRay* treeRay, Buffer<int> stack)
        {
            Debug.Assert((nodeIndex >= 0 && nodeIndex < NodeCount) || (Encode(nodeIndex) >= 0 && Encode(nodeIndex) < LeafCount));
            Debug.Assert(LeafCount >= 2, "This implementation assumes all nodes are filled.");

            int stackEnd = 0;
            while (true)
            {
                if (nodeIndex < 0)
                {
                    Console.WriteLine($"Entering leaf.");
                    //This is actually a leaf node.
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        return;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    //if (nodeIndex < 0 || nodeIndex >= NodeCount)
                    //{
                    //    Console.WriteLine($"BAD index {nodeIndex} versus {NodeCount}");
                    //    Console.ReadLine();
                    //}
                    ref var node = ref Nodes[nodeIndex];
                    Console.WriteLine($"Entering node. ChildA, B: {node.A.Index}, {node.B.Index}");
                    var minA = node.A.Min - expansion;
                    var maxA = node.A.Max + expansion;
                    var aIntersected = Intersects(minA, maxA, treeRay, out var tA);
                    var minB = node.B.Min - expansion;
                    var maxB = node.B.Max + expansion;
                    var bIntersected = Intersects(minB, maxB, treeRay, out var tB);

                    if (aIntersected)
                    {
                        if (bIntersected)
                        {
                            //Visit the earlier AABB intersection first.
                            if (tA < tB)
                            {
                                //Console.WriteLine($"Using inline node index A: {node.A.Index}");
                                nodeIndex = node.A.Index;
                                stack[stackEnd++] = node.B.Index;
                            }
                            else
                            {
                                //Console.WriteLine($"Using inline node index B: {node.B.Index}");
                                nodeIndex = node.B.Index;
                                stack[stackEnd++] = node.A.Index;
                            }
                        }
                        else
                        {
                            //Single intersection cases don't require an explicit stack entry.
                            //Console.WriteLine($"Using inline node index A: {node.A.Index}");
                            nodeIndex = node.A.Index;
                        }
                    }
                    else if (bIntersected)
                    {
                        //Console.WriteLine($"Using inline node index B: {node.B.Index}");
                        nodeIndex = node.B.Index;
                    }
                    else
                    {
                        //No intersection. Need to pull from the stack to get a new target.
                        //Console.WriteLine($"need to yoink from stack.");
                        if (stackEnd == 0)
                            return;
                        nodeIndex = stack[--stackEnd];
                        //Console.WriteLine($"Popped new node index: {nodeIndex}");
                    }
                }
            }
        }
    }

    public static void Test()
    {
        const int stackSize = 256;
        var stackMemory = stackalloc int[stackSize];
        var stack = new Buffer<int>(stackMemory, stackSize);

        var tree = new Reprotree();
        var node = new Node();
        node.A = new NodeChild { LeafCount = 1, Index = -1, Min = new Vector3(-5), Max = new Vector3(5) };
        node.B = new NodeChild { LeafCount = 1, Index = -2, Min = new Vector3(-5), Max = new Vector3(5) };
        tree.Nodes = new Buffer<Node>(&node, 1);
        tree.NodeCount = 1;
        tree.LeafCount = 2;
        TreeRay.CreateFrom(Vector3.Zero, new Vector3(0, 1, 0), 1, out var treeRay);
        tree.Sweep(0, new Vector3(5), &treeRay, stack);

    }
}
