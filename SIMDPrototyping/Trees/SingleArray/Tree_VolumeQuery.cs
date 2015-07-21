using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

#if NODE32
using Node = SIMDPrototyping.Trees.SingleArray.Node32;
#elif NODE16
using Node = SIMDPrototyping.Trees.SingleArray.Node16;
#elif NODE8
using Node = SIMDPrototyping.Trees.SingleArray.Node8;
#elif NODE4
using Node = SIMDPrototyping.Trees.SingleArray.Node4;
#elif NODE2
using Node = SIMDPrototyping.Trees.SingleArray.Node2;
#endif


namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Test<TResultList>(int* stack, ref int count, ref BoundingBox query, Node* node,
            ref TResultList results) where TResultList : IList<int>
        {
            var boundingBoxes = &node->A;
            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (BoundingBox.Intersects(ref query, ref boundingBoxes[i]))
                {
                    if (children[i] >= 0)
                    {
                        stack[count++] = children[i];
                    }
                    else
                    {
                        results.Add(Encode(children[i]));
                    }
                }

            }
        }



        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Obsolete]
        public unsafe void Query<TResultList>(ref BoundingBox boundingBox, ref TResultList results) where TResultList : IList<int>
        {
            //TODO: could optimize this by keeping the next target out of the stack.
            const int maximumDepth = 128; //TODO: This is technically a bug. Trees CAN exceed this size.
            var stackCapacity = (ChildrenCapacity - 1) * maximumDepth + 1;
            var stack = stackalloc int[stackCapacity];
            int count = 0;

            var boundingBoxWide = new BoundingBoxWide(ref boundingBox);
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            Test(stack, ref count, ref boundingBox, Nodes, ref results);

            while (count > 0)
            {
                --count;
                var nodeIndex = stack[count];

                Test(stack, ref count, ref boundingBox, Nodes + nodeIndex, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestRecursive<TResultList>(int nodeIndex,
            ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = Nodes + nodeIndex;
            var boundingBoxes = &node->A;
            var children = &node->ChildA;
            var childCount = node->ChildCount;
            for (int i = 0; i < childCount; ++i)
            {
                if (BoundingBox.Intersects(ref query, ref boundingBoxes[i]))
                {
                    if (children[i] >= 0)
                    {
                        TestRecursive(children[i], ref query, ref results);
                    }
                    else
                    {
                        results.Add(Encode(children[i]));
                    }
                }
            }
        }

        

#if NODE4
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestRecursive4<TResultList>(int nodeIndex, ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = (Nodes + nodeIndex);
            var childCount = node->ChildCount;

            Debug.Assert(childCount >= 1);

            bool a, b, c, d;
            a = BoundingBox.Intersects(ref query, ref node->A);
            b = BoundingBox.Intersects(ref query, ref node->B);
            c = BoundingBox.Intersects(ref query, ref node->C);


            if (a)
            {
                if (node->ChildA >= 0)
                {
                    TestRecursive4(node->ChildA, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildA));
                }
            }
            if (childCount < 2)
                return;
            d = BoundingBox.Intersects(ref query, ref node->D);
            if (b)
            {
                if (node->ChildB >= 0)
                {
                    TestRecursive4(node->ChildB, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildB));
                }
            }
            if (childCount < 3)
                return;
            if (c)
            {
                if (node->ChildC >= 0)
                {
                    TestRecursive4(node->ChildC, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildC));
                }
            }
            if (childCount < 4)
                return;
            if (d)
            {
                if (node->ChildD >= 0)
                {
                    TestRecursive4(node->ChildD, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildD));
                }
            }


        }
#endif

#if NODE4
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestRecursive4Switch<TResultList>(int nodeIndex,
            ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = Nodes + nodeIndex;

            switch (node->ChildCount)
            {
                case 1:
                    {
                        bool a = BoundingBox.Intersects(ref query, ref node->A);
                        
                        if (a)
                        {
                            if (node->ChildA >= 0)
                            {
                                TestRecursive4Switch(node->ChildA, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildA));
                            }
                        }
                    }
                    break;
                case 2:
                    {
                        bool a, b;
                        a = BoundingBox.Intersects(ref query, ref node->A);
                        b = BoundingBox.Intersects(ref query, ref node->B);
                        
                        if (a)
                        {
                            if (node->ChildA >= 0)
                            {
                                TestRecursive4Switch(node->ChildA, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildA));
                            }
                        }
                        if (b)
                        {
                            if (node->ChildB >= 0)
                            {
                                TestRecursive4Switch( node->ChildB, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildB));
                            }
                        }
                    }
                    break;
                case 3:
                    {
                        bool a, b, c;
                        a = BoundingBox.Intersects(ref query, ref node->A);
                        b = BoundingBox.Intersects(ref query, ref node->B);
                        c = BoundingBox.Intersects(ref query, ref node->C);
                        
                        if (a)
                        {
                            if (node->ChildA >= 0)
                            {
                                TestRecursive4Switch( node->ChildA, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildA));
                            }
                        }
                        if (b)
                        {
                            if (node->ChildB >= 0)
                            {
                                TestRecursive4Switch( node->ChildB, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildB));
                            }
                        }
                        if (c)
                        {
                            if (node->ChildC >= 0)
                            {
                                TestRecursive4Switch( node->ChildC, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildC));
                            }
                        }
                    }
                    break;
                case 4:
                    {
                        bool a, b, c, d;
                        a = BoundingBox.Intersects(ref query, ref node->A);
                        b = BoundingBox.Intersects(ref query, ref node->B);
                        c = BoundingBox.Intersects(ref query, ref node->C);
                        d = BoundingBox.Intersects(ref query, ref node->D);
                        
                        if (a)
                        {
                            if (node->ChildA >= 0)
                            {
                                TestRecursive4Switch(node->ChildA, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildA));
                            }
                        }
                        if (b)
                        {
                            if (node->ChildB >= 0)
                            {
                                TestRecursive4Switch(node->ChildB, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildB));
                            }
                        }
                        if (c)
                        {
                            if (node->ChildC >= 0)
                            {
                                TestRecursive4Switch(node->ChildC, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildC));
                            }
                        }
                        if (d)
                        {
                            if (node->ChildD >= 0)
                            {
                                TestRecursive4Switch(node->ChildD, ref query, ref results);
                            }
                            else
                            {
                                results.Add(Encode(node->ChildD));
                            }
                        }
                    }
                    break;
            }




        }
#endif
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void QueryRecursive<TResultList>(ref BoundingBox boundingBox, ref TResultList results) where TResultList : IList<int>
        {
            //Assumption: root is always zero.
#if NODE4
            
            TestRecursive4(0, ref boundingBox, ref results);
#endif
        }
    }
}
