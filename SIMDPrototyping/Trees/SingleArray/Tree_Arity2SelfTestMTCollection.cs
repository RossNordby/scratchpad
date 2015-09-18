using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushDifferent(int a, int b, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = nodes + a;
            element->B = nodes + b;
            element->Type = PairType.InternalInternal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushLeafInternal(int internalNode, BoundingBox* leafBounds, int encodedLeafIndex, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = nodes + internalNode;
            element->LeafBounds = leafBounds;
            element->EncodedLeafIndex = encodedLeafIndex;
            element->Type = PairType.LeafInternal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushSame(int node, TestPair* stack, ref int count)
        {
            var element = stack + count++;
            element->A = nodes + node;
            element->Type = PairType.SameNode;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestDifferentNodes<TResultList>(int a, int b, BoundingBox* aBounds, BoundingBox* bBounds, TestPair* stack, ref int nextToVisit, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (a >= 0)
            {
                if (b >= 0)
                {
                    //both internal nodes
                    PushDifferent(a, b, stack, ref nextToVisit);
                }
                else
                {
                    //leaf and internal
                    PushLeafInternal(a, bBounds, b, stack, ref nextToVisit);
                }
            }
            else if (b >= 0)
            {
                //leaf and internal
                PushLeafInternal(b, aBounds, a, stack, ref nextToVisit);
            }
            else
            {
                //two leaves
                results.Add(new Overlap { A = Encode(a), B = Encode(b) });
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushSame(int index, int leafCount, ref PriorityQueue queue, ref QuickList<TestPair> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, leafCount);
            pairsToTest.Add(new TestPair { A = nodes + index, Type = PairType.SameNode });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushDifferent(int a, int b, int leafCountA, int leafCountB, ref PriorityQueue queue, ref QuickList<TestPair> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, Math.Max(leafCountA, leafCountB));
            pairsToTest.Add(new TestPair { A = nodes + a, B = nodes + b, Type = PairType.InternalInternal });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushInternalLeaf(int encodedLeafIndex, BoundingBox* leafBounds, int internalIndex, int leafCount, ref PriorityQueue queue, ref QuickList<TestPair> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, (float)Math.Log(leafCount));
            pairsToTest.Add(new TestPair { A = nodes + internalIndex, LeafBounds = leafBounds, EncodedLeafIndex = encodedLeafIndex, Type = PairType.LeafInternal });
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

        public unsafe void CollectTestPairs<TResultList>(int targetPairCount, ref QuickList<Overlap> testPairs, ref TResultList results) where TResultList : IList<Overlap>
        {
            PriorityQueue.Entry* entries = stackalloc PriorityQueue.Entry[targetPairCount];
            PriorityQueue queue = new PriorityQueue(entries);


            QuickList<TestPair> pairsToTest = new QuickList<TestPair>(BufferPools<TestPair>.Locking, BufferPool<TestPair>.GetPoolIndex(targetPairCount * 2));
            PushSame(0, leafCount, ref queue, ref pairsToTest);
            while (queue.Count < targetPairCount)
            {
                PriorityQueue.Entry entry;
                queue.PopMax(out entry);
                var pairToTest = pairsToTest[entry.Id];
                switch (pairToTest.Type)
                {
                    case PairType.SameNode:
                        {
                            var ab = BoundingBox.Intersects(ref pairToTest.A->A, ref pairToTest.A->B);

                            //TODO: note the shared conditions for childA internalness and the shared pointer arithmetic.
                            if (pairToTest.A->ChildA >= 0)
                                PushSame(pairToTest.A->ChildA, stack, ref count);
                            if (pairToTest.A->ChildB >= 0)
                                PushSame(pairToTest.A->ChildB, stack, ref count);

                            if (ab)
                            {
                                TestDifferentNodes(pairToTest.A->ChildA, pairToTest.A->ChildB, &pairToTest.A->A, &pairToTest.A->B, stack, ref count, ref results);
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
                                TestDifferentNodes(pairToTest.A->ChildA, pairToTest.B->ChildA, &pairToTest.A->A, &pairToTest.B->A, stack, ref count, ref results);
                            }
                            if (ab)
                            {
                                TestDifferentNodes(pairToTest.A->ChildA, pairToTest.B->ChildB, &pairToTest.A->A, &pairToTest.B->B, stack, ref count, ref results);
                            }
                            if (ba)
                            {
                                TestDifferentNodes(pairToTest.A->ChildB, pairToTest.B->ChildA, &pairToTest.A->B, &pairToTest.B->A, stack, ref count, ref results);
                            }
                            if (bb)
                            {
                                TestDifferentNodes(pairToTest.A->ChildB, pairToTest.B->ChildB, &pairToTest.A->B, &pairToTest.B->B, stack, ref count, ref results);
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
                                    PushLeafInternal(pairToTest.A->ChildA, pairToTest.LeafBounds, pairToTest.EncodedLeafIndex, stack, ref count);
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
                                    PushLeafInternal(pairToTest.A->ChildB, pairToTest.LeafBounds, pairToTest.EncodedLeafIndex, stack, ref count);
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
            pairsToTest.Count = 0;
            pairsToTest.Dispose();
        }
    }
}
