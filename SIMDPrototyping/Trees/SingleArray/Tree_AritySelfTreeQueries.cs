using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
#if NODE2
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForLeaf<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (nodeIndex < 0)
            {
                results.Add(new Overlap { A = leafIndex, B = Encode(nodeIndex) });
            }
            else
            {
                TestLeafAgainstNode2(leafIndex, ref leafBounds, nodeIndex, ref results);
            }
        }
        unsafe void TestLeafAgainstNode2<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            var node = nodes + nodeIndex;

            var a = BoundingBox.Intersects(ref leafBounds, ref node->A);
            var b = BoundingBox.Intersects(ref leafBounds, ref node->B);

            var nodeChildA = node->ChildA;
            var nodeChildB = node->ChildB;

            if (a)
            {
                DispatchTestForLeaf(leafIndex, ref leafBounds, nodeChildA, ref results);
            }
            if (b)
            {
                DispatchTestForLeaf(leafIndex, ref leafBounds, nodeChildB, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForNodes<TResultList>(int childA, int childB, ref BoundingBox boundsA, ref BoundingBox boundsB, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (childA >= 0)
            {
                if (childB >= 0)
                {
                    GetOverlapsBetweenDifferentNodes2(nodes + childA, nodes + childB, ref results);
                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode2(Encode(childB), ref boundsB, childA, ref results);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode2(Encode(childA), ref boundsA, childB, ref results);
            }
            else
            {
                //Two leaves.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        unsafe void GetOverlapsBetweenDifferentNodes2<TResultList>(Node* a, Node* b, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
            var bb = BoundingBox.Intersects(ref a->B, ref b->B);

            if (aa)
            {
                DispatchTestForNodes(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
            }
            if (ab)
            {
                DispatchTestForNodes(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
            }
            if (ba)
            {
                DispatchTestForNodes(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
            }
            if (bb)
            {
                DispatchTestForNodes(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
            }

        }

        unsafe void GetOverlapsInNode2<TResultList>(Node* node, ref TResultList results) where TResultList : IList<Overlap>
        {

            var nodeChildA = node->ChildA;
            var nodeChildB = node->ChildB;

            var ab = BoundingBox.Intersects(ref node->A, ref node->B);

            if (nodeChildA >= 0)
                GetOverlapsInNode2(nodes + nodeChildA, ref results);
            if (nodeChildB >= 0)
                GetOverlapsInNode2(nodes + nodeChildB, ref results);

            //Test all different nodes.
            if (ab)
            {
                DispatchTestForNodes(nodeChildA, nodeChildB, ref node->A, ref node->B, ref results);
            }

        }
        public unsafe void GetSelfOverlaps2<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            GetOverlapsInNode2(nodes, ref results);

        }


        unsafe struct PairToTest
        {
            public int A;
            public int B;
            public BoundingBox* LeafBounds;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushPair(int a, int b, PairToTest* stack, ref int nextToVisit)
        {
            //Potential microoptimizations here
            stack[++nextToVisit] = new PairToTest { A = a, B = b };

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushPair(int encodedLeafIndex, BoundingBox* leafBounds, int b, PairToTest* stack, ref int nextToVisit)
        {
            //Potential microoptimizations here
            stack[++nextToVisit] = new PairToTest { A = encodedLeafIndex, B = b, LeafBounds = leafBounds };

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestDifferentNodes<TResultList>(int childA, int childB, BoundingBox* a, BoundingBox* b,
            PairToTest* stack, ref int nextToVisit,
            ref TResultList results) where TResultList : IList<Overlap>
        {
            if (childA >= 0)
            {
                if (childB >= 0)
                {
                    PushPair(childA, childB, stack, ref nextToVisit);
                }
                else
                {
                    //leaf B versus node A.
                    PushPair(childB, b, childA, stack, ref nextToVisit);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                PushPair(childA, a, childB, stack, ref nextToVisit);
            }
            else
            {
                //Two leaves.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        public unsafe void GetSelfOverlapsExplicit<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            var stack = stackalloc PairToTest[32];
            int nextToVisit = 0;

            stack->A = 0;
            stack->B = 0;

            while (nextToVisit >= 0)
            {
                //Console.WriteLine($"stack size: {nextToVisit + 1}");
                var pairToTest = stack + nextToVisit--;

                if (pairToTest->A == pairToTest->B)
                {
                    //Same nodes
                    var node = nodes + pairToTest->A;

                    var ab = BoundingBox.Intersects(ref node->A, ref node->B);

                    if (node->ChildA >= 0)
                        PushPair(node->ChildA, node->ChildA, stack, ref nextToVisit);
                    if (node->ChildB >= 0)
                        PushPair(node->ChildB, node->ChildB, stack, ref nextToVisit);

                    if (ab)
                    {
                        TestDifferentNodes(node->ChildA, node->ChildB, &node->A, &node->B, stack, ref nextToVisit, ref results);
                    }
                }
                else
                {
                    if (pairToTest->A < 0)
                    {
                        //It's a leaf-versus-not-leaf.
                        var internalNode = nodes + pairToTest->B;
                        var a = BoundingBox.Intersects(ref *pairToTest->LeafBounds, ref internalNode->A);
                        var b = BoundingBox.Intersects(ref *pairToTest->LeafBounds, ref internalNode->B);
                        if (a)
                        {
                            if (internalNode->ChildA >= 0)
                            {
                                PushPair(pairToTest->A, pairToTest->LeafBounds, internalNode->ChildA, stack, ref nextToVisit);
                            }
                            else
                            {
                                results.Add(new Overlap { A = Encode(pairToTest->A), B = Encode(pairToTest->B) });
                            }
                        }
                        if (b)
                        {
                            if (internalNode->ChildB >= 0)
                            {
                                PushPair(pairToTest->A, pairToTest->LeafBounds, internalNode->ChildB, stack, ref nextToVisit);
                            }
                            else
                            {
                                results.Add(new Overlap { A = Encode(pairToTest->A), B = Encode(pairToTest->B) });
                            }
                        }

                    }
                    else
                    {
                        //Different nodes  
                        var a = nodes + pairToTest->A;
                        var b = nodes + pairToTest->B;
                        var aa = BoundingBox.Intersects(ref a->A, ref b->A);
                        var ab = BoundingBox.Intersects(ref a->A, ref b->B);
                        var ba = BoundingBox.Intersects(ref a->B, ref b->A);
                        var bb = BoundingBox.Intersects(ref a->B, ref b->B);


                        if (aa)
                        {
                            TestDifferentNodes(a->ChildA, b->ChildA, &a->A, &b->A, stack, ref nextToVisit, ref results);
                        }
                        if (ab)
                        {
                            TestDifferentNodes(a->ChildA, b->ChildB, &a->A, &b->B, stack, ref nextToVisit, ref results);
                        }
                        if (ba)
                        {
                            TestDifferentNodes(a->ChildB, b->ChildA, &a->B, &b->A, stack, ref nextToVisit, ref results);
                        }
                        if (bb)
                        {
                            TestDifferentNodes(a->ChildB, b->ChildB, &a->B, &b->B, stack, ref nextToVisit, ref results);
                        }
                    }
                }

            }
        }

        enum PairType
        {
            SameNode,
            LeafInternal,
            InternalInternal
        }
        [StructLayout(LayoutKind.Explicit)]
        unsafe struct TestPair
        {
            [FieldOffset(0)]
            public Node* A;
            [FieldOffset(8)]
            public Node* B;
            [FieldOffset(8)]
            public BoundingBox* LeafBounds;
            [FieldOffset(16)]
            public int EncodedLeafIndex;
            [FieldOffset(20)]
            public PairType Type;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushDifferent(Node* a, Node* b, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = a;
            element->B = b;
            element->Type = PairType.InternalInternal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushLeafInternal(Node* internalNode, BoundingBox* leafBounds, int encodedLeafIndex, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = internalNode;
            element->LeafBounds = leafBounds;
            element->EncodedLeafIndex = encodedLeafIndex;
            element->Type = PairType.LeafInternal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushSame(Node* node, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = node;
            element->Type = PairType.SameNode;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestDifferent<TResultList>(int a, int b, BoundingBox* aBounds, BoundingBox* bBounds, TestPair* stack, ref int nextToVisit, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (a >= 0)
            {
                if (b >= 0)
                {
                    //both internal nodes
                    PushDifferent(nodes + a, nodes + b, stack, ref nextToVisit);
                }
                else
                {
                    //leaf and internal
                    PushLeafInternal(nodes + a, bBounds, b, stack, ref nextToVisit);
                }
            }
            else if (b >= 0)
            {
                //leaf and internal
                PushLeafInternal(nodes + b, aBounds, a, stack, ref nextToVisit);
            }
            else
            {
                //two leaves
                results.Add(new Overlap { A = Encode(a), B = Encode(b) });
            }
        }

        public unsafe void GetSelfOverlapsExplicit2<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            var stack = stackalloc TestPair[32];

            stack->A = nodes;
            stack->Type = PairType.SameNode;
            int count = 1;

            while (count > 0)
            {
                var pairToTest = stack[--count];
                switch (pairToTest.Type)
                {
                    case PairType.SameNode:
                        {
                            var ab = BoundingBox.Intersects(ref pairToTest.A->A, ref pairToTest.A->B);

                            //TODO: note the shared conditions for childA internalness and the shared pointer arithmetic.
                            if (pairToTest.A->ChildA >= 0)
                                PushSame(nodes + pairToTest.A->ChildA, stack, ref count);
                            if (pairToTest.A->ChildB >= 0)
                                PushSame(nodes + pairToTest.A->ChildB, stack, ref count);

                            if (ab)
                            {
                                TestDifferent(pairToTest.A->ChildA, pairToTest.A->ChildB, &pairToTest.A->A, &pairToTest.A->B, stack, ref count, ref results);
                            }
                        }
                        break;
                    case PairType.InternalInternal:
                        {
                            var aa = BoundingBox.Intersects(ref pairToTest.A->A, ref pairToTest.B->A);
                            var ab = BoundingBox.Intersects(ref pairToTest.A->A, ref pairToTest.B->B);
                            var ba = BoundingBox.Intersects(ref pairToTest.A->B, ref pairToTest.B->A);
                            var bb = BoundingBox.Intersects(ref pairToTest.A->B, ref pairToTest.B->B);

                            if (aa)
                            {
                                TestDifferent(pairToTest.A->ChildA, pairToTest.B->ChildA, &pairToTest.A->A, &pairToTest.B->A, stack, ref count, ref results);
                            }
                            if (ab)
                            {
                                TestDifferent(pairToTest.A->ChildA, pairToTest.B->ChildB, &pairToTest.A->A, &pairToTest.B->B, stack, ref count, ref results);
                            }
                            if (ba)
                            {
                                TestDifferent(pairToTest.A->ChildB, pairToTest.B->ChildA, &pairToTest.A->B, &pairToTest.B->A, stack, ref count, ref results);
                            }
                            if (bb)
                            {
                                TestDifferent(pairToTest.A->ChildB, pairToTest.B->ChildB, &pairToTest.A->B, &pairToTest.B->B, stack, ref count, ref results);
                            }
                        }
                        break;
                    case PairType.LeafInternal:
                        {
                            var a = BoundingBox.Intersects(ref *pairToTest.LeafBounds, ref pairToTest.A->A);
                            var b = BoundingBox.Intersects(ref *pairToTest.LeafBounds, ref pairToTest.A->B);
                            if (a)
                            {
                                if (pairToTest.A->ChildA >= 0)
                                {
                                    PushLeafInternal(nodes + pairToTest.A->ChildA, pairToTest.LeafBounds, pairToTest.EncodedLeafIndex, stack, ref count);
                                }
                                else
                                {
                                    results.Add(new Overlap { A = Encode(pairToTest.EncodedLeafIndex), B = Encode(pairToTest.A->ChildA) });
                                }
                            }
                            if (b)
                            {
                                if (pairToTest.A->ChildB >= 0)
                                {
                                    PushLeafInternal(nodes + pairToTest.A->ChildB, pairToTest.LeafBounds, pairToTest.EncodedLeafIndex, stack, ref count);
                                }
                                else
                                {
                                    results.Add(new Overlap { A = Encode(pairToTest.EncodedLeafIndex), B = Encode(pairToTest.A->ChildB) });
                                }
                            }
                        }
                        break;
                }

            }
        }

#endif


        public unsafe void GetSelfOverlapsArityDedicated<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
#if NODE2
            GetOverlapsInNode2(nodes, ref results);
#endif

        }

    }
}
