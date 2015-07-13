using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace SIMDPrototyping.Trees.Baseline
{
    partial class Tree<T>
    {
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

        const float leafCost = 1;
        const float internalNodeCost = 1;
        unsafe float MeasureCostHeuristic(int levelIndex, int nodeIndex, float parentBoundsHeuristic)
        {
            var node = Levels[levelIndex].Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            int nextLevel = levelIndex + 1;
            float cost = 0;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] < -1)
                {
                    //It's a leaf node. They cost one by convention.
                    cost += leafCost;
                }
                else
                {
                    //Internal node.
                    var childBoundsHeuristic = ComputeBoundsHeuristic(ref bounds[i]);
                    cost += MeasureCostHeuristic(nextLevel, children[i], childBoundsHeuristic) * childBoundsHeuristic;
                }
            }
            return cost / parentBoundsHeuristic + internalNodeCost;
        }

        public unsafe float MeasureCostHeuristic()
        {
            //Could flatten this a lot. Unnecessary recursion.
            var node = Levels[0].Nodes;
            var bounds = &node->A;
            var children = &node->ChildA;

            float cost = 0;
            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < node->ChildCount; ++i)
            {
                BoundingBox.Merge(ref bounds[i], ref merged, out merged);
            }
            float rootHeuristic = ComputeBoundsHeuristic(ref merged);
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] < -1)
                {
                    //It's a leaf node. They cost one by convention.
                    cost += leafCost;
                }
                else
                {
                    //Internal node.
                    var childBoundsHeuristic = ComputeBoundsHeuristic(ref bounds[i]);
                    cost += MeasureCostHeuristic(1, children[i], childBoundsHeuristic) * childBoundsHeuristic;
                }
            }

            return cost / rootHeuristic;
        }

        public unsafe float MeasureCostHeuristic2()
        {
            var rootNode = Levels[0].Nodes;
            var rootBounds = &rootNode->A;

            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < rootNode->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref merged, out merged);
            }
            float rootHeuristic = ComputeBoundsHeuristic(ref merged);

            float totalCost = 0;
            for (int levelIndex = 0; levelIndex <= maximumDepth; ++levelIndex)
            {
                var level = Levels[levelIndex];
                for (int nodeIndex = 0; nodeIndex < level.Count; ++nodeIndex)
                {
                    var node = level.Nodes + nodeIndex;
                    var children = &node->ChildA;
                    var bounds = &node->A;
                    for (int childIndex = 0; childIndex < level.Nodes[nodeIndex].ChildCount; ++childIndex)
                    {
                        if (children[childIndex] >= 0)
                        {
                            //Internal node.
                            totalCost += internalNodeCost * ComputeBoundsHeuristic(ref bounds[childIndex]);
                        }
                        else
                        {
                            //Leaf node.
                            totalCost += leafCost * ComputeBoundsHeuristic(ref bounds[childIndex]);
                        }

                    }
                }
            }
            return totalCost / rootHeuristic;

        }

    }
}
