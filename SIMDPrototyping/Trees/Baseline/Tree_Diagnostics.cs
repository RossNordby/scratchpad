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
                    cost += 1;
                }
                else
                {
                    //Internal node.
                    var childBoundsHeuristic = ComputeBoundsHeuristic(ref bounds[i]);
                    cost += MeasureCostHeuristic(nextLevel, children[i], childBoundsHeuristic) * childBoundsHeuristic;
                }
            }
            return cost / parentBoundsHeuristic + 1;
        }

        public unsafe float MeasureCostHeuristic()
        {
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
                    cost += 1;
                }
                else
                {
                    //Internal node.
                    var childBoundsHeuristic = ComputeBoundsHeuristic(ref bounds[i]);
                    cost += MeasureCostHeuristic(1, children[i], childBoundsHeuristic) * childBoundsHeuristic;
                }
            }

            return cost / rootHeuristic + 1;
        }

    }
}
