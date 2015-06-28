//#define OUTPUT

using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
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

        int leafCount;

        public int LeafCount
        {
            get
            {
                return leafCount;
            }
        }

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

        public Tree(int initialLeafCapacity = 4096, int initialTreeDepth = 16)
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
                Levels[i] = new Level { Nodes = new Node[Math.Min(initialLeafCapacity, (int)Math.Pow(4, i))] };
            }
            InitializeNode(out Levels[0].Nodes[0]);
            Levels[0].Count = 1;

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
                    var oldLeafIndex = Encode(childIndex);
                    var nextLevel = levelIndex + 1;
                    //this is the only place where a new level could potentially be created.
                    EnsureLevel(nextLevel);
                    Node newNode;
                    InitializeNode(out newNode);
                    //The first child of the new node is the old leaf. Insert its bounding box.
                    //Since we don't have a great way of shuffling yet, just let it be in the same index.
                    BoundingBoxWide.ConditionalSelect(
                        ref singleMasks[minimumIndex],
                        ref newNode.BoundingBoxes,
                        ref level.Nodes[nodeIndex].BoundingBoxes,
                        out newNode.BoundingBoxes);

                    newNode.Children = Vector.ConditionalSelect(singleMasks[minimumIndex], level.Nodes[nodeIndex].Children, newNode.Children);
                    //Insert the new leaf into the second child slot.
                    //Just put it in the next slot over from the minimum.
                    var maskIndex = (minimumIndex + 1) & vectorSizeMask;
                    BoundingBoxWide.ConditionalSelect(
                      ref singleMasks[maskIndex],
                      ref newNode.BoundingBoxes,
                      ref box,
                      out newNode.BoundingBoxes);
                    var leafIndex = leafCount++;
                    var leafIndexVector = new Vector<int>(Encode(leafIndex));
                    newNode.Children = Vector.ConditionalSelect(singleMasks[maskIndex], leafIndexVector, newNode.Children);

                    //Update the original node's child pointer and bounding box.
                    var newNodeIndexVector = new Vector<int>(Levels[nextLevel].Add(ref newNode));
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
                    var leafIndex = leafCount++;
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

    }
}
