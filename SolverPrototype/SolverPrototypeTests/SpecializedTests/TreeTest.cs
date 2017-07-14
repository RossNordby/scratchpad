using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    public static class TreeTest
    {
        public static void AddRemove()
        {
            var pool = new BufferPool();
            var tree = new Tree(pool, 128);

            const int leafCountAlongAxis = 12;
            var leafCount = leafCountAlongAxis * leafCountAlongAxis * leafCountAlongAxis;
            var leafBounds = new BoundingBox[leafCount];
            var handleToLeafIndex = new int[leafCount];
            var leafIndexToHandle = new int[leafCount];

            const float boundsSpan = 2;
            const float spanRange = 2;
            const float boundsSpacing = 3;
            var random = new Random(5);
            int nextLeafSlotIndex = 0;
            for (int i = 0; i < leafCountAlongAxis; ++i)
            {
                for (int j = 0; j < leafCountAlongAxis; ++j)
                {
                    for (int k = 0; k < leafCountAlongAxis; ++k)
                    {
                        var index = leafCountAlongAxis * leafCountAlongAxis * i + leafCountAlongAxis * j + k;
                        leafBounds[index].Min = new Vector3(i, j, k) * boundsSpacing;
                        leafBounds[index].Max = leafBounds[index].Min + new Vector3(boundsSpan) +
                            spanRange * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                        var handleIndex = nextLeafSlotIndex++;
                        handleToLeafIndex[handleIndex] = tree.Add(ref leafBounds[index]);
                        leafIndexToHandle[handleToLeafIndex[handleIndex]] = handleIndex;
                    }
                }
            }
            tree.Validate();

            const int iterations = 100000;
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), leafCount, out var removedLeafHandles);
            for (int i = 0; i < iterations; ++i)
            {
                var addedFraction = tree.LeafCount / (float)leafCount;
                
                if (random.NextDouble() < addedFraction)
                {
                    //Remove a leaf.
                    var leafIndexToRemove = random.Next(tree.LeafCount);
                    var handleToRemove = leafIndexToHandle[leafIndexToRemove];
                    var movedLeafIndex = tree.RemoveAt(leafIndexToRemove);
                    if (movedLeafIndex >= 0)
                    {
                        //A leaf was moved from the end into the removed leaf's slot.
                        var movedHandle = leafIndexToHandle[movedLeafIndex];
                        handleToLeafIndex[movedHandle] = leafIndexToRemove;
                        leafIndexToHandle[leafIndexToRemove] = movedHandle;
                        leafIndexToHandle[movedLeafIndex] = -1;
                    }
                    else
                    {
                        //The removed leaf was the last one. This leaf index is no longer associated with any existing leaf.
                        leafIndexToHandle[leafIndexToRemove] = -1;
                    }
                    handleToLeafIndex[handleToRemove] = -1;

                    removedLeafHandles.AddUnsafely(handleToRemove);

                    tree.Validate();
                }
                else
                {
                    //Add a leaf.
                    var indexInRemovedList = random.Next(removedLeafHandles.Count);
                    var handleToAdd = removedLeafHandles[indexInRemovedList];
                    removedLeafHandles.FastRemoveAt(indexInRemovedList);
                    var leafIndex = tree.Add(ref leafBounds[handleToAdd]);
                    leafIndexToHandle[leafIndex] = handleToAdd;
                    handleToLeafIndex[handleToAdd] = leafIndex;

                    tree.Validate();
                }
            }


        }
    }
}
