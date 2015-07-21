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
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            if (node->ChildA >= 0)
            {
                Refit(node->ChildA, out node->A);
            }
            if (node->ChildB >= 0)
            {
                Refit(node->ChildB, out node->B);
            }
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            for (int i = 2; i < node->ChildCount; ++i)
            {
                if ((&node->ChildA)[i] >= 0)
                {
                    Refit((&node->ChildA)[i], out (&node->A)[i]);
                }
                BoundingBox.Merge(ref (&node->A)[i], ref boundingBox, out boundingBox);
            }
        }

        unsafe void RefitCached(int nodeIndex, out BoundingBox boundingBox)
        {
            var node = Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA; //try move down?
            var childCount = node->ChildCount;
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            if (node->ChildA >= 0)
            {
                RefitCached(node->ChildA, out node->A);
            }
            if (node->ChildB >= 0)
            {
                RefitCached(node->ChildB, out node->B);
            }
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            for (int i = 2; i < childCount; ++i)
            {
                if (children[i] >= 0)
                {
                    RefitCached(children[i], out bounds[i]);
                }
                BoundingBox.Merge(ref bounds[i], ref boundingBox, out boundingBox);
            }
        }

#if NODE4
        unsafe void Refit4(int nodeIndex, out BoundingBox boundingBox)
        {
            var node = Nodes + nodeIndex;
            var childCount = node->ChildCount;
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);


            if (node->ChildA >= 0)
            {
                Refit4(node->ChildA, out node->A);
            }
            if (node->ChildB >= 0)
            {
                Refit4(node->ChildB, out node->B);
            }
            if (node->ChildCount < 3)
                goto Merge;
            if (node->ChildC >= 0)
            {
                Refit4(node->ChildC, out node->C);
            }
            if (node->ChildCount < 4)
                goto Merge;
            if (node->ChildD >= 0)
            {
                Refit4(node->ChildD, out node->D);
            }
            Merge:
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            if (node->ChildCount < 3)
                return;
            BoundingBox.Merge(ref node->C, ref boundingBox, out boundingBox);
            if (node->ChildCount < 4)
                return;
            BoundingBox.Merge(ref node->D, ref boundingBox, out boundingBox);

        }

        unsafe void RefitSwitch4(int nodeIndex, out BoundingBox boundingBox)
        {
            var node = Nodes + nodeIndex;
            var childCount = node->ChildCount;
            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);


            switch (node->ChildCount)
            {
                case 1:
                    {
                        //Must be a leaf.
                        Debug.Assert(node->ChildA < 0);
                        boundingBox = node->A;
                    }
                    break;
                case 2:
                    {
                        if (node->ChildA >= 0)
                        {
                            RefitSwitch4(node->ChildA, out node->A);
                        }
                        if (node->ChildB >= 0)
                        {
                            RefitSwitch4(node->ChildB, out node->B);
                        }
                        BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
                    }
                    break;
                case 3:
                    {
                        if (node->ChildA >= 0)
                        {
                            RefitSwitch4(node->ChildA, out node->A);
                        }
                        if (node->ChildB >= 0)
                        {
                            RefitSwitch4(node->ChildB, out node->B);
                        }
                        if (node->ChildCount < 3)
                            goto Merge;
                        if (node->ChildC >= 0)
                        {
                            RefitSwitch4(node->ChildC, out node->C);
                        }
                        Merge:
                        BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
                        if (node->ChildCount < 3)
                            return;
                        BoundingBox.Merge(ref node->C, ref boundingBox, out boundingBox);
                    }
                    break;
                case 4:
                default:

                    {
                        if (node->ChildA >= 0)
                        {
                            RefitSwitch4(node->ChildA, out node->A);
                        }
                        if (node->ChildB >= 0)
                        {
                            RefitSwitch4(node->ChildB, out node->B);
                        }
                        if (node->ChildCount < 3)
                            goto Merge;
                        if (node->ChildC >= 0)
                        {
                            RefitSwitch4(node->ChildC, out node->C);
                        }
                        if (node->ChildCount < 4)
                            goto Merge;
                        if (node->ChildD >= 0)
                        {
                            RefitSwitch4(node->ChildD, out node->D);
                        }
                        Merge:
                        BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
                        if (node->ChildCount < 3)
                            return;
                        BoundingBox.Merge(ref node->C, ref boundingBox, out boundingBox);
                        if (node->ChildCount < 4)
                            return;
                        BoundingBox.Merge(ref node->D, ref boundingBox, out boundingBox);
                    }
                    break;
            }
        }
#endif

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
