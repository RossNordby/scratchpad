//#define NODE8


using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

#if NODE32
using Node = SIMDPrototyping.Trees.Baseline.Node32;
#elif NODE16
using Node = SIMDPrototyping.Trees.Baseline.Node16;
#elif NODE8
using Node = SIMDPrototyping.Trees.Baseline.Node8;
#elif NODE4
using Node = SIMDPrototyping.Trees.Baseline.Node4;
#elif NODE2
using Node = SIMDPrototyping.Trees.Baseline.Node2;
#endif

namespace SIMDPrototyping.Trees.Baseline
{
    partial class Tree<T>
    {
        struct Subtree
        {
            public int Level;
            public int Node;
        }

        unsafe void CollectSubtrees(int levelIndex, int nodeIndex, int depth, List<Subtree> subtrees)
        {
            //TODO: could replace recursion with direct BFS queue collection 
            var childNode = Levels[levelIndex].Nodes + nodeIndex;
            var children = &childNode->ChildA;
            var nextLevel = levelIndex + 1;
            if (depth > 1)
            {
                var nextDepth = depth - 1;
                for (int i = 0; i < childNode->ChildCount; ++i)
                {
                    if (children[i] >= 0)
                    {
                        CollectSubtrees(nextLevel, children[i], nextDepth, subtrees);
                    }
                    else
                    {
                        //It's a leaf node; can't recurse.
                        subtrees.Add(new Subtree { Level = nextLevel, Node = children[i] });
                    }
                }
            }
            else
            {
                //Add all the nodes at this level as subtrees, whether they're leaves or internal nodes.
                for (int i = 0; i < childNode->ChildCount; ++i)
                {
                    subtrees.Add(new Subtree { Level = nextLevel, Node = children[i] });
                }
            }
        }

        unsafe void RefineNode(int levelIndex, int nodeIndex, int depth)
        {
            var node = Levels[levelIndex].Nodes + nodeIndex;
            //For each subtree below this node, check for alternative locations.
            //A subtree is represented by the node (internal or leaf) which is has the depth levelIndex + depth, or leaves which are encountered at a higher depth.
            //So, for a 16-ary tree with a depth of 2, you have 256 potential subtrees.
            List<Subtree> subtrees = new List<Subtree>();
            CollectSubtrees(levelIndex, nodeIndex, depth, subtrees);
        }

        public void Refine()
        {
            //How many levels above the last one should we start?
            //This offset avoids wasting time examining leaf node rooted treelets.
            const int depth =
#if NODE2
            8;
#elif NODE4
            4;
#elif NODE8
            3; //This is not exactly equivalent to the others: 512 instead of 256 options.
#elif NODE16
            2;
#endif

            var startingLevel = maximumDepth - depth;
            for (int nodeIndex = 0; nodeIndex < Levels[startingLevel].Count; ++nodeIndex)
            {
                RefineNode(startingLevel, nodeIndex, depth);
            }

        }
    }
}
