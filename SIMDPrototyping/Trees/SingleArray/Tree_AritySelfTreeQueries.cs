using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
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


        struct PairToTest
        {
            public int A;
            public int B;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushPair(int a, int b, PairToTest* stack, ref int nextToVisit)
        {
            //Potential microoptimizations here
            stack[++nextToVisit] = new PairToTest { A = a, B = b };

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestDifferentNodes<TResultList>(int childA, int childB, ref BoundingBox a, ref BoundingBox b, PairToTest* stack, ref int nextToVisit, ref TResultList results) where TResultList : IList<Overlap>
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
                    TestLeafAgainstNode2(Encode(childB), ref b, childA, ref results);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode2(Encode(childA), ref a, childB, ref results);
            }
            else
            {
                //Two leaves.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        public unsafe void GetSelfOverlapsExplicit<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            var stack = stackalloc PairToTest[128];
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
                        TestDifferentNodes(node->ChildA, node->ChildB, ref node->A, ref node->B, stack, ref nextToVisit, ref results);
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
                        TestDifferentNodes(a->ChildA, b->ChildA, ref a->A, ref b->A, stack, ref nextToVisit, ref results);
                    }
                    if (ab)
                    {
                        TestDifferentNodes(a->ChildA, b->ChildB, ref a->A, ref b->B, stack, ref nextToVisit, ref results);
                    }
                    if (ba)
                    {
                        TestDifferentNodes(a->ChildB, b->ChildA, ref a->B, ref b->A, stack, ref nextToVisit, ref results);
                    }
                    if (bb)
                    {
                        TestDifferentNodes(a->ChildB, b->ChildB, ref a->B, ref b->B, stack, ref nextToVisit, ref results);
                    }
                }

            }





        }


    }
}
