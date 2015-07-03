//#define OUTPUT

using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;



namespace SIMDPrototyping.Trees
{

    public interface IBounded
    {
        void GetBoundingBox(out BoundingBox box);
    }
    public class Tree<T> where T : IBounded
    {
        struct Level
        {
            //Consider using a pointer to avoid pointless range checking.
            //Requires fixing. Use debug conditional stuff to ensure safety;
            //may want to use the array itself in debug mode too.
            //Doesn't change the other syntax really.
            public Node[] Nodes;
            public int Count;

            public int Add(ref Node node)
            {
                if (Count == Nodes.Length)
                {
                    var newNodes = new Node[Nodes.Length * 2];
                    Array.Copy(Nodes, newNodes, Nodes.Length);
                    Nodes = newNodes;
                }
                Nodes[Count] = node;
                return Count++;
            }
        }
        Level[] Levels;


        struct Leaf
        {
            public T Bounded;
            /// <summary>
            /// Which level the leaf is in.
            /// </summary>
            public int LevelIndex;
            /// <summary>
            /// Which node within the level the leaf is in.
            /// </summary>
            public int NodeIndex;
            /// <summary>
            /// Which child within the node the leaf is in.
            /// </summary>
            public int ChildIndex;
        }
        Leaf[] leaves;
        int leafCount;

        public int LeafCount
        {
            get
            {
                return leafCount;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(T leaf, int levelIndex, int nodeIndex, int childIndex)
        {
            if (leafCount == leaves.Length)
            {
                var newLeaves = new Leaf[leafCount * 2];
                Array.Copy(leaves, newLeaves, leafCount);
                leaves = newLeaves;
            }
            leaves[leafCount].Bounded = leaf;
            leaves[leafCount].LevelIndex = levelIndex;
            leaves[leafCount].NodeIndex = nodeIndex;
            leaves[LeafCount].ChildIndex = childIndex;
            return leafCount++;
        }

        private void EnsureLevel(int nextLevel)
        {
            if (nextLevel >= Levels.Length)
            {
                var newLevels = new Level[Levels.Length * 2];
                Array.Copy(Levels, newLevels, Levels.Length);
                for (int i = Levels.Length; i < newLevels.Length; ++i)
                {
                    //Assume the same size for subsequent levels. While they could grow exponentially,
                    //typically the tree will not be perfectly balanced.
                    newLevels[i] = new Level { Nodes = new Node[Levels[Levels.Length - 1].Nodes.Length] };
                }
                Levels = newLevels;
            }
            if (nextLevel > maximumDepth)
                maximumDepth = nextLevel;

        }

        int vectorSizeMask;
        Vector<int>[] singleMasks;



        int maximumDepth;
        /// <summary>
        /// Gets the index of the deepest tree layer that contains any nodes.
        /// </summary>
        public int MaximumDepth
        {
            get
            {
                return maximumDepth;
            }
        }

        public Tree(int initialLeafCapacity = 4096, int initialTreeDepth = 24)
        {
            if (initialTreeDepth <= 0)
                throw new ArgumentException("Initial tree depth must be positive.");
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");
            singleMasks = new Vector<int>[Vector<int>.Count];
            var buffer = BufferPools<int>.Locking.Take(Vector<int>.Count);
            unchecked
            {
                buffer[0] = (int)0xffffffff;
                singleMasks[0] = new Vector<int>(buffer);
                for (int i = 1; i < singleMasks.Length; ++i)
                {
                    buffer[i - 1] = 0;
                    buffer[i] = (int)0xffffffff;
                    singleMasks[i] = new Vector<int>(buffer);
                }
            }
            Array.Clear(buffer, 0, buffer.Length);
            BufferPools<int>.Locking.GiveBack(buffer);

            Levels = new Level[initialTreeDepth];
            var maximumNodeCount = (int)Math.Ceiling(initialTreeDepth / (double)Vector<float>.Count);
            for (int i = 0; i < Levels.Length; ++i)
            {
                Levels[i] = new Level { Nodes = new Node[Math.Min(initialLeafCapacity, (long)Math.Pow(4, Math.Min(25, i)))] };
            }
            InitializeNode(out Levels[0].Nodes[0]);
            Levels[0].Count = 1;

            leaves = new Leaf[initialLeafCapacity];

            vectorSizeMask = Vector<float>.Count - 1;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InitializeNode(out Node node)
        {
            //could load a premade one instead.
            BoundingBox empty = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            node.BoundingBoxes = new BoundingBoxWide(ref empty);
            node.Children = new Vector<int>(-1);
            //'no child' is encoded as -1. 
            //Leaf nodes are encoded as -(leafIndex + 2).
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int Encode(int index)
        {
            return -(index + 2);
        }

        string GetChoiceList(List<int> choices)
        {
            StringBuilder builder = new StringBuilder(choices.Count * 2);
            for (int i = 0; i < choices.Count; ++i)
            {
                builder.Append(choices[i]);
                if (i != choices.Count - 1)
                    builder.Append(", ");
            }
            return builder.ToString();
        }

        public unsafe void Insert(T leaf)
        {
            int levelIndex = 0;
            int nodeIndex = 0;
            BoundingBox aosBox;
            leaf.GetBoundingBox(out aosBox);
            var box = new BoundingBoxWide(ref aosBox);
#if OUTPUT
            List<int> choices = new List<int>();
#endif
            while (true)
            {
                var level = Levels[levelIndex];
                //Which child should the leaf belong to?
                Vector<float> originalVolumes;
                BoundingBoxWide.ComputeVolume(ref level.Nodes[nodeIndex].BoundingBoxes, out originalVolumes);
                originalVolumes = Vector.Max(originalVolumes, Vector<float>.Zero);
                BoundingBoxWide merged;
                BoundingBoxWide.Merge(ref level.Nodes[nodeIndex].BoundingBoxes, ref box, out merged);
                Vector<float> mergedVolumes;
                BoundingBoxWide.ComputeVolume(ref merged, out mergedVolumes);

                //Give the leaf to whichever node had the least volume change.
                var volumeIncreases = mergedVolumes - originalVolumes;
                int minimumIndex = 0;
                var minimum = volumeIncreases[0];
                for (int i = 1; i < Vector<float>.Count; ++i)
                {
                    if (volumeIncreases[i] < minimum)
                    {
                        minimumIndex = i;
                        minimum = volumeIncreases[i];
                    }
                }
#if OUTPUT
                Console.WriteLine($"Minimum index: {minimumIndex}, minimum volume increase: {minimum}");
                choices.Add(minimumIndex);
#endif

                var childIndex = level.Nodes[nodeIndex].Children[minimumIndex];

                if (childIndex < -1)
                {
                    //It's a leaf node.
                    //Create a new internal node with the new leaf and the old leaf as children.
                    var nextLevel = levelIndex + 1;
                    //this is the only place where a new level could potentially be created.
                    EnsureLevel(nextLevel);
                    Node newNode;
                    InitializeNode(out newNode);
                    //The first child of the new node is the old leaf. Insert its bounding box.
                    //Since we don't have a great way of shuffling yet, just let it be in the same index.
                    BoundingBoxWide.ConditionalSelect(
                        ref singleMasks[minimumIndex],
                        ref level.Nodes[nodeIndex].BoundingBoxes,
                        ref newNode.BoundingBoxes,
                        out newNode.BoundingBoxes);
                    newNode.Children = Vector.ConditionalSelect(singleMasks[minimumIndex], level.Nodes[nodeIndex].Children, newNode.Children);

                    //Insert the new leaf into the second child slot.
                    //Just put it in the next slot over from the minimum.
                    var newLeafChildIndex = (minimumIndex + 1) & vectorSizeMask;
                    BoundingBoxWide.ConditionalSelect(
                      ref singleMasks[newLeafChildIndex],
                      ref box,
                      ref newNode.BoundingBoxes,
                      out newNode.BoundingBoxes);
                    var newNodeIndex = Levels[nextLevel].Add(ref newNode);
                    var leafIndex = AddLeaf(leaf, nextLevel, newNodeIndex, newLeafChildIndex);
                    var leafIndexVector = new Vector<int>(Encode(leafIndex));
                    Levels[nextLevel].Nodes[newNodeIndex].Children = Vector.ConditionalSelect(singleMasks[newLeafChildIndex], leafIndexVector, newNode.Children);

                    //Update the old leaf node with the new index information.
                    var oldLeafIndex = Encode(childIndex);
                    leaves[oldLeafIndex].LevelIndex = nextLevel;
                    leaves[oldLeafIndex].NodeIndex = newNodeIndex;
                    //Since we inserted it into the same slot of the new node as it was in in the old node, there is no change to the child index.

                    //Update the original node's child pointer and bounding box.
                    var newNodeIndexVector = new Vector<int>(newNodeIndex);
                    level.Nodes[nodeIndex].Children = Vector.ConditionalSelect(singleMasks[minimumIndex], newNodeIndexVector, level.Nodes[nodeIndex].Children);
                    BoundingBoxWide.ConditionalSelect(ref singleMasks[minimumIndex], ref merged, ref level.Nodes[nodeIndex].BoundingBoxes, out level.Nodes[nodeIndex].BoundingBoxes);

#if OUTPUT
                    Console.WriteLine($"Leaf {leafIndex} merged with existing leaf.");// New Node Children: {newNode.Children}, Old Node children: {level.Nodes[nodeIndex].Children}");
                    Console.WriteLine($"Choices: {GetChoiceList(choices)}");
#endif

                    break;
                }
                if (childIndex == -1)
                {
                    //There is no child at all.
                    //Put the new leaf here.
                    var leafIndex = AddLeaf(leaf, levelIndex, nodeIndex, minimumIndex);
                    var leafIndexVector = new Vector<int>(Encode(leafIndex));
                    level.Nodes[nodeIndex].Children = Vector.ConditionalSelect(singleMasks[minimumIndex], leafIndexVector, level.Nodes[nodeIndex].Children);
                    BoundingBoxWide.ConditionalSelect(ref singleMasks[minimumIndex], ref merged, ref level.Nodes[nodeIndex].BoundingBoxes, out level.Nodes[nodeIndex].BoundingBoxes);

#if OUTPUT
                    Console.WriteLine($"Leaf {leafIndex} inserted in empty slot.");
                    Console.WriteLine($"Choices: {GetChoiceList(choices)}");
#endif
                    break;
                }
                //It's an internal node. Traverse to the next node.
                BoundingBoxWide.ConditionalSelect(ref singleMasks[minimumIndex], ref merged, ref level.Nodes[nodeIndex].BoundingBoxes, out level.Nodes[nodeIndex].BoundingBoxes);
                nodeIndex = level.Nodes[nodeIndex].Children[minimumIndex];
                ++levelIndex;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ComputeBoundingBox(ref BoundingBoxWide boundingBoxes, out BoundingBoxWide mergedWide)
        {
            //YIKES transposition
            BoundingBox merged;
            merged.Min = new Vector3(float.MaxValue);
            merged.Max = new Vector3(-float.MaxValue);
            for (int childIndex = 0; childIndex < Vector<int>.Count; ++childIndex)
            {
                var childMin = new Vector3(
                    boundingBoxes.Min.X[childIndex],
                    boundingBoxes.Min.Y[childIndex],
                    boundingBoxes.Min.Z[childIndex]);
                var childMax = new Vector3(
                    boundingBoxes.Max.X[childIndex],
                    boundingBoxes.Max.Y[childIndex],
                    boundingBoxes.Max.Z[childIndex]);
                merged.Min = Vector3.Min(merged.Min, childMin);
                merged.Max = Vector3.Max(merged.Max, childMax);
            }
            mergedWide = new BoundingBoxWide(ref merged);
        }

        public unsafe void Refit()
        {
            //Update the bounding boxes of every leaf-owner.
            //Note the scalar-ness of this. It seems like there should exist some way to vectorize it properly, though it may require changing things around.
            for (int i = 0; i < leafCount; ++i)
            {
                BoundingBox box;
                leaves[i].Bounded.GetBoundingBox(out box);
                BoundingBoxWide wideBox = new BoundingBoxWide(ref box);
                //Console.WriteLine($"index reached: {i}, child index: {leaves[i].ChildIndex}, level: {leaves[i].LevelIndex}, node: { leaves[i].NodeIndex}");

                BoundingBoxWide.ConditionalSelect(ref singleMasks[leaves[i].ChildIndex],
                    ref wideBox,
                    ref Levels[leaves[i].LevelIndex].Nodes[leaves[i].NodeIndex].BoundingBoxes,
                    out Levels[leaves[i].LevelIndex].Nodes[leaves[i].NodeIndex].BoundingBoxes);
                //Console.WriteLine($"comp");

            }
            //Go through each level, refitting as you go.
            //Note that the deepest level is skipped. It does not need to be tested; it's all leaves that were already updated.
            for (int levelIndex = maximumDepth - 1; levelIndex >= 0; --levelIndex)
            {
                //consider testing caching Levels[levelIndex]. It may have a minor effect.
                for (int nodeIndex = 0; nodeIndex < Levels[levelIndex].Count; ++nodeIndex)
                {
                    for (int childIndex = 0; childIndex < Vector<int>.Count; ++childIndex)
                    {
                        var childNodeIndex = Levels[levelIndex].Nodes[nodeIndex].Children[childIndex];
                        if (childNodeIndex >= 0)
                        {
                            BoundingBoxWide merged;
                            ComputeBoundingBox(ref Levels[levelIndex + 1].Nodes[childNodeIndex].BoundingBoxes, out merged);
                            BoundingBoxWide.ConditionalSelect(ref singleMasks[childIndex],
                                ref merged,
                                ref Levels[levelIndex].Nodes[nodeIndex].BoundingBoxes,
                                out Levels[levelIndex].Nodes[nodeIndex].BoundingBoxes);
                        }

                    }
                }
            }
        }


        unsafe struct TraversalStack
        {
            public TraversalTarget* Stack;
            public int Count;
            public void Initialize(TraversalTarget* stack)
            {
                this.Stack = stack;
                this.Count = 0;
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Push(int levelIndex, int nodeIndex)
            {
                Stack[Count] = new TraversalTarget { Level = levelIndex, Node = nodeIndex };
                Count++;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Pop(out TraversalTarget target)
            {
                if (Count > 0)
                {
                    --Count;
                    target = Stack[Count];
                    return true;
                }
                target = new TraversalTarget();
                return false;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Test<TResultList>(TraversalTarget* stack, ref int count, int stackCapacity, int level,
            ref BoundingBoxWide query, ref Node node,
            ref TResultList results) where TResultList : IList<T>
        {
            Vector<int> intersectionMask;
            BoundingBoxWide.Intersects(ref node.BoundingBoxes, ref query, out intersectionMask);
            //Console.WriteLine($"Intersection mask: {intersectionMask}");
            //Console.WriteLine(node.BoundingBoxes);
            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                if (intersectionMask[i] < 0)
                {
                    if (node.Children[i] >= 0)
                    {
                        Debug.Assert(count < stackCapacity);
                        stack[count++] = new TraversalTarget { Level = level + 1, Node = node.Children[i] };
                    }
                    else if (node.Children[i] < -1)
                    {
                        results.Add(leaves[Encode(node.Children[i])].Bounded);
                    }
                }
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Query<TResultList>(ref BoundingBox boundingBox, ref TResultList results) where TResultList : IList<T>
        {
            //TODO: could optimize this by keeping the next target out of the stack.
            var stackCapacity = (Vector<int>.Count - 1) * maximumDepth + 1;
            var stack = stackalloc TraversalTarget[stackCapacity];
            int count = 0;

            var boundingBoxWide = new BoundingBoxWide(ref boundingBox);
            Test(stack, ref count, stackCapacity, 0, ref boundingBoxWide, ref Levels[0].Nodes[0], ref results);

            while (count > 0)
            {
                --count;
                var target = stack[count];

                Test(stack, ref count, stackCapacity, target.Level, ref boundingBoxWide, ref Levels[target.Level].Nodes[target.Node], ref results);
            }
        }


        unsafe void TestRecursive<TResultList>(int level, int nodeIndex,
            ref BoundingBoxWide query,
            ref TResultList results) where TResultList : IList<T>
        {
            Vector<int> intersectionMask;
            BoundingBoxWide.Intersects(ref Levels[level].Nodes[nodeIndex].BoundingBoxes, ref query, out intersectionMask);
            //Console.WriteLine($"Intersection mask: {intersectionMask}");
            //Console.WriteLine(node.BoundingBoxes);
            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                if (intersectionMask[i] < 0)
                {
                    if (Levels[level].Nodes[nodeIndex].Children[i] >= 0)
                    {
                        TestRecursive(level + 1, Levels[level].Nodes[nodeIndex].Children[i], ref query, ref results);
                    }
                    else if (Levels[level].Nodes[nodeIndex].Children[i] < -1)
                    {
                        results.Add(leaves[Encode(Levels[level].Nodes[nodeIndex].Children[i])].Bounded);
                    }
                }
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void QueryRecursive<TResultList>(ref BoundingBox boundingBox, ref TResultList results) where TResultList : IList<T>
        {
            var boundingBoxWide = new BoundingBoxWide(ref boundingBox);
            TestRecursive(0, 0, ref boundingBoxWide, ref results);
        }

    }

    struct TraversalTarget
    {
        public int Level;
        public int Node;
    }
}
