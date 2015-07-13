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



        public unsafe float MeasureCostHeuristic()
        {
            var rootNode = Levels[0].Nodes;
            var rootBounds = &rootNode->A;

            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < rootNode->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref merged, out merged);
            }
            float rootHeuristic = ComputeBoundsHeuristic(ref merged);

            const float leafCost = 1;
            const float internalNodeCost = 1;

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
