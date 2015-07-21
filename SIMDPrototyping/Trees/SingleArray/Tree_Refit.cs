using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        unsafe void Refit(int nodeIndex, out BoundingBox boundingBox)
        {
            var node = Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            var childCount = node->ChildCount;
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            if (children[0] >= 0)
            {
                Refit(children[0], out boundingBox);
            }
            else
            {
                //This is a leaf node. It requires no refitting.
                boundingBox = bounds[0];
            }
            for (int i = 1; i < childCount; ++i)
            {
                if (children[i] >= 0)
                {
                    BoundingBox childBox;
                    Refit(children[i], out childBox);
                    BoundingBox.Merge(ref childBox, ref boundingBox, out boundingBox);
                }
                else
                {
                    //This is a leaf node. Merge the existing bounding box in.
                    BoundingBox.Merge(ref bounds[i], ref boundingBox, out boundingBox);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void UpdateLeafBoundingBox(int leafIndex, ref BoundingBox boundingBox)
        {
            (&Nodes[Leaves[leafIndex].NodeIndex].A)[Leaves[leafIndex].ChildIndex] = boundingBox;
        }

        public unsafe void Refit()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootChildren = &Nodes->ChildA;
            var rootBounds = &Nodes->A;
            for (int i = 0; i < Nodes->ChildCount; ++i)
            {
                if (rootChildren[i] >= 0)
                {
                    Refit(rootChildren[i], out rootBounds[i]);
                }
            }
        }

    }
}
