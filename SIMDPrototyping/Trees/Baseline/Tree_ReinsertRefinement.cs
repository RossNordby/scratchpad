//#define NODE8


using BEPUutilities.DataStructures;
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
        void ComputeBestCostChange(int levelIndex, int nodeIndex, ref QuickList<int> childrenIndices)
        {
        }

        public void InsertGlobal(T leaf)
        {
            BoundingBox leafBox;
            leaf.GetBoundingBox(out leafBox);
        }
    }
}
