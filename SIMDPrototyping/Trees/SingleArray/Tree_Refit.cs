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
            var node = nodes + nodeIndex;
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
            var node = nodes + nodeIndex;
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
            var node = nodes + nodeIndex;
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
            var node = nodes + nodeIndex;
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
        unsafe void UpdateLeafBoundingBox(int leafIndex, ref BoundingBox boundingBox)
        {
            (&nodes[leaves[leafIndex].NodeIndex].A)[leaves[leafIndex].ChildIndex] = boundingBox;
        }

        public unsafe void Refit()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootChildren = &nodes->ChildA;
            var rootBounds = &nodes->A;
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
                if (rootChildren[i] >= 0)
                {
                    Refit(rootChildren[i], out rootBounds[i]);
                }
            }
        }




        unsafe struct StackElement
        {
            public int NodeIndex;
            public BoundingBox* ParentBounds;
        }

        public unsafe void RefitNonrecursive2()
        {
            if (nodes->ChildCount < 2)
            {
                Debug.Assert(nodes->ChildCount == 0 || (nodes->ChildCount == 1 && nodes->ChildA < 0), "If there is only one child in a node, it should be a leaf- otherwise this node is a waste.");
                //Nothing to refit.
                return;
            }
            var stack = stackalloc StackElement[512];
            int count = 1;
            BoundingBox standin;
            stack[0].NodeIndex = 0;
            stack[0].ParentBounds = &standin;
            while (count > 0)
            {
                var endOfStack = count - 1;
                var stackElement = stack[endOfStack];
                var node = nodes + stackElement.NodeIndex;

                var bounds = &node->A;
                var children = &node->ChildA;
                //Note backwards order to ensure proper prefix traversal.
                var lastIndex = node->ChildCount - 1;
                for (int i = lastIndex; i >= 0; --i)
                {
                    var index = count++;
                    stack[index].NodeIndex = children[i];
                    stack[index].ParentBounds = bounds + i;
                }

                //todo: this doesnt work- should do this on pop, but we incorrectly pop early
                BoundingBox.Merge(ref node->A, ref node->B, out *stackElement.ParentBounds);

            }
        }

        unsafe struct StackElement2
        {
            public int NodeIndex;
            public int NextChildIndexToVisit;
            public BoundingBox* ParentBounds;
        }

        public unsafe void RefitNonrecursive22()
        {
            if (nodes->ChildCount < 2)
            {
                Debug.Assert(nodes->ChildCount == 0 || (nodes->ChildCount == 1 && nodes->ChildA < 0), "If there is only one child in a node, it should be a leaf- otherwise this node is a waste.");
                //Nothing to refit.
                return;
            }
            var stack = stackalloc StackElement2[64];
            int count = 1;
            BoundingBox standin;
            stack[0].NodeIndex = 0;
            stack[0].NextChildIndexToVisit = -1;
            stack[0].ParentBounds = &standin;
            while (count > 0)
            {
                var endOfStack = count - 1;
                var stackElement = stack + endOfStack;


                var node = nodes + stackElement->NodeIndex;

                var children = &node->ChildA;

                while (true)
                {
                    ++stackElement->NextChildIndexToVisit;
                    if (stackElement->NextChildIndexToVisit >= node->ChildCount)
                    {
                        //No more children.   
                        //TODO: could use the node's parent pointers to access the bounds slot directly...
                        BoundingBox.Merge(ref node->A, ref node->B, out *stackElement->ParentBounds);
                        count = endOfStack;
                        break;
                    }
                    var childIndex = children[stackElement->NextChildIndexToVisit];
                    //There is another child to visit. Is it an internal node?
                    if (childIndex >= 0)
                    {
                        //It's an internal node, so enqueue the child.
                        var newStackElement = stack + count;
                        newStackElement->NextChildIndexToVisit = -1;
                        newStackElement->NodeIndex = childIndex;
                        newStackElement->ParentBounds = &(&node->A)[stackElement->NextChildIndexToVisit];
                        ++count;
                        break;
                    }

                }







            }
        }


    }
}
