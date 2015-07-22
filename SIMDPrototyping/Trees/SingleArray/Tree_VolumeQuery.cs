﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;



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


#if NODE16
        unsafe void TestRecursive16<TResultList>(int nodeIndex, ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = (Nodes + nodeIndex);
            var childCount = node->ChildCount;

            Debug.Assert(childCount >= 1);

            var a = BoundingBox.Intersects(ref query, ref node->A);
            var b = BoundingBox.Intersects(ref query, ref node->B);
            var c = BoundingBox.Intersects(ref query, ref node->C);


            if (a)
            {
                if (node->ChildA >= 0)
                {
                    TestRecursive16(node->ChildA, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildA));
                }
            }
            if (childCount < 2)
                return;
            var d = BoundingBox.Intersects(ref query, ref node->D);
            if (b)
            {
                if (node->ChildB >= 0)
                {
                    TestRecursive16(node->ChildB, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildB));
                }
            }
            if (childCount < 3)
                return;
            var e = BoundingBox.Intersects(ref query, ref node->E);
            if (c)
            {
                if (node->ChildC >= 0)
                {
                    TestRecursive16(node->ChildC, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildC));
                }
            }
            if (childCount < 4)
                return;
            var f = BoundingBox.Intersects(ref query, ref node->F);
            if (d)
            {
                if (node->ChildD >= 0)
                {
                    TestRecursive16(node->ChildD, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildD));
                }
            }
            if (childCount < 5)
                return;
            var g = BoundingBox.Intersects(ref query, ref node->G);
            if (e)
            {
                if (node->ChildE >= 0)
                {
                    TestRecursive16(node->ChildE, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildE));
                }
            }
            if (childCount < 6)
                return;
            var h = BoundingBox.Intersects(ref query, ref node->H);
            if (f)
            {
                if (node->ChildF >= 0)
                {
                    TestRecursive16(node->ChildF, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildF));
                }
            }
            if (childCount < 7)
                return;
            var i = BoundingBox.Intersects(ref query, ref node->H);
            if (g)
            {
                if (node->ChildG >= 0)
                {
                    TestRecursive16(node->ChildG, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildG));
                }
            }
            if (childCount < 8)
                return;
            var j = BoundingBox.Intersects(ref query, ref node->H);
            if (h)
            {
                if (node->ChildH >= 0)
                {
                    TestRecursive16(node->ChildH, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildH));
                }
            }
            if (childCount < 9)
                return;
            var k = BoundingBox.Intersects(ref query, ref node->H);
            if (i)
            {
                if (node->ChildA >= 0)
                {
                    TestRecursive16(node->ChildA, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildA));
                }
            }
            if (childCount < 10)
                return;
            var l = BoundingBox.Intersects(ref query, ref node->D);
            if (j)
            {
                if (node->ChildB >= 0)
                {
                    TestRecursive16(node->ChildB, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildB));
                }
            }
            if (childCount < 11)
                return;
            var m = BoundingBox.Intersects(ref query, ref node->E);
            if (k)
            {
                if (node->ChildC >= 0)
                {
                    TestRecursive16(node->ChildC, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildC));
                }
            }
            if (childCount < 12)
                return;
            var n = BoundingBox.Intersects(ref query, ref node->F);
            if (l)
            {
                if (node->ChildD >= 0)
                {
                    TestRecursive16(node->ChildD, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildD));
                }
            }
            if (childCount < 13)
                return;
            var o = BoundingBox.Intersects(ref query, ref node->G);
            if (m)
            {
                if (node->ChildE >= 0)
                {
                    TestRecursive16(node->ChildE, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildE));
                }
            }
            if (childCount < 14)
                return;
            var p = BoundingBox.Intersects(ref query, ref node->H);
            if (n)
            {
                if (node->ChildF >= 0)
                {
                    TestRecursive16(node->ChildF, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildF));
                }
            }
            if (childCount < 15)
                return;
            if (o)
            {
                if (node->ChildG >= 0)
                {
                    TestRecursive16(node->ChildG, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildG));
                }
            }
            if (childCount < 16)
                return;
            if (p)
            {
                if (node->ChildH >= 0)
                {
                    TestRecursive16(node->ChildH, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildH));
                }
            }

        }
#endif

#if NODE8
        unsafe void TestRecursive8<TResultList>(int nodeIndex, ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = (Nodes + nodeIndex);
            var childCount = node->ChildCount;

            Debug.Assert(childCount >= 1);
            
            var a = BoundingBox.Intersects(ref query, ref node->A);
            var b = BoundingBox.Intersects(ref query, ref node->B);
            var c = BoundingBox.Intersects(ref query, ref node->C);


            if (a)
            {
                if (node->ChildA >= 0)
                {
                    TestRecursive8(node->ChildA, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildA));
                }
            }
            if (childCount < 2)
                return;
            var d = BoundingBox.Intersects(ref query, ref node->D);
            if (b)
            {
                if (node->ChildB >= 0)
                {
                    TestRecursive8(node->ChildB, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildB));
                }
            }
            if (childCount < 3)
                return;
            var e = BoundingBox.Intersects(ref query, ref node->E);
            if (c)
            {
                if (node->ChildC >= 0)
                {
                    TestRecursive8(node->ChildC, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildC));
                }
            }
            if (childCount < 4)
                return;
            var f = BoundingBox.Intersects(ref query, ref node->F);
            if (d)
            {
                if (node->ChildD >= 0)
                {
                    TestRecursive8(node->ChildD, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildD));
                }
            }
            if (childCount < 5)
                return;
            var g = BoundingBox.Intersects(ref query, ref node->G);
            if (e)
            {
                if (node->ChildE >= 0)
                {
                    TestRecursive8(node->ChildE, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildE));
                }
            }
            if (childCount < 6)
                return;
            var h = BoundingBox.Intersects(ref query, ref node->H);
            if (f)
            {
                if (node->ChildF >= 0)
                {
                    TestRecursive8(node->ChildF, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildF));
                }
            }
            if (childCount < 7)
                return;
            if (g)
            {
                if (node->ChildG >= 0)
                {
                    TestRecursive8(node->ChildG, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildG));
                }
            }
            if (childCount < 8)
                return;
            if (h)
            {
                if (node->ChildH >= 0)
                {
                    TestRecursive8(node->ChildH, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildH));
                }
            }


        }
#endif

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
                                TestRecursive4Switch(node->ChildB, ref query, ref results);
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
#if NODE2
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestRecursive2<TResultList>(int nodeIndex, ref BoundingBox query,
            ref TResultList results) where TResultList : IList<int>
        {
            var node = (Nodes + nodeIndex);
            var childCount = node->ChildCount;

            Debug.Assert(childCount >= 1);
            
            var a = BoundingBox.Intersects(ref query, ref node->A);
            var b = BoundingBox.Intersects(ref query, ref node->B);
            
            if (a)
            {
                if (node->ChildA >= 0)
                {
                    TestRecursive2(node->ChildA, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildA));
                }
            }
            if (childCount < 2)
                return;
            if (b)
            {
                if (node->ChildB >= 0)
                {
                    TestRecursive2(node->ChildB, ref query, ref results);
                }
                else
                {
                    results.Add(Encode(node->ChildB));
                }
            }



        }
#endif

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void QueryRecursive<TResultList>(ref BoundingBox boundingBox, ref TResultList results) where TResultList : IList<int>
        {
            //Assumption: root is always zero.

#if NODE4
            TestRecursive4(0, ref boundingBox, ref results);
#elif NODE8
            TestRecursive8(0, ref boundingBox, ref results);
#else
            TestRecursive(0, ref boundingBox, ref results);
#endif
        }
    }
}