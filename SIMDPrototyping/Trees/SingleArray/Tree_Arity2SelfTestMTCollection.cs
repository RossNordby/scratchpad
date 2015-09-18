using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        [StructLayout(LayoutKind.Explicit)]
        unsafe struct TestPair2
        {
            [FieldOffset(0)]
            public int A;
            [FieldOffset(4)]
            public int B;
            [FieldOffset(8)]
            public BoundingBox* LeafBounds;
            [FieldOffset(16)]
            public PairType Type;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushSame(int index, int leafCount, ref PriorityQueue queue, ref QuickList<TestPair2> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, (float)(Math.Log(leafCount) * leafCount));
            pairsToTest.Add(new TestPair2 { A = index, Type = PairType.SameNode });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushDifferent(int a, int b, int leafCountA, int leafCountB, ref PriorityQueue queue, ref QuickList<TestPair2> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, (float)(Math.Log(leafCountA) * leafCountA + Math.Log(leafCountB) * leafCountB));
            pairsToTest.Add(new TestPair2 { A = a, B = b, Type = PairType.InternalInternal });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void PushLeafInternal(int internalIndex, int leafCount, BoundingBox* leafBounds, int encodedLeafIndex, ref PriorityQueue queue, ref QuickList<TestPair2> pairsToTest)
        {
            queue.Insert(pairsToTest.Count, (float)Math.Log(leafCount));
            pairsToTest.Add(new TestPair2 { A = internalIndex, LeafBounds = leafBounds, B = encodedLeafIndex, Type = PairType.LeafInternal });
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void TestDifferentNodes<TResultList>(BoundingBox* a, BoundingBox* b, int childA, int childB, int leafCountA, int leafCountB,
             ref PriorityQueue queue, ref QuickList<TestPair2> pairsToTest,
            ref TResultList results) where TResultList : IList<Overlap>
        {
            if (childA >= 0)
            {
                if (childB >= 0)
                {
                    PushDifferent(childA, childB, leafCountA, leafCountB, ref queue, ref pairsToTest);
                }
                else
                {
                    //leaf B versus node A.
                    PushLeafInternal(childA, leafCountA, b, childB, ref queue, ref pairsToTest);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                PushLeafInternal(childB, leafCountB, a, childA, ref queue, ref pairsToTest);
            }
            else
            {
                //Two leaves.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        public unsafe void CollectNodePairs2<TResultList>(int targetPairCount, ref QuickList<Overlap> testPairs, ref TResultList results) where TResultList : IList<Overlap>
        {
            //For a 2-ary tree, the maximum number of added node pairs is 4.
            //That will only potentially occur if the tree is not yet at the target pair count (i.e. max of targetPairCount - 1), and popping a node will reduce it one further.
            PriorityQueue.Entry* entries = stackalloc PriorityQueue.Entry[targetPairCount + 2];
            PriorityQueue queue = new PriorityQueue(entries);


            QuickList<TestPair2> pairsToTest = new QuickList<TestPair2>(BufferPools<TestPair2>.Locking, BufferPool<TestPair2>.GetPoolIndex(targetPairCount * 2));
            PushSame(0, leafCount, ref queue, ref pairsToTest);
            while (queue.Count < targetPairCount && queue.Count > 0)
            {
                PriorityQueue.Entry entry;
                queue.PopMax(out entry);
                var pairToTest = pairsToTest[entry.Id];
                switch (pairToTest.Type)
                {
                    case PairType.SameNode:
                        {
                            var node = nodes + pairToTest.A;
                            var ab = BoundingBox.Intersects(ref node->A, ref node->B);

                            //TODO: note the shared conditions for childA internalness and the shared pointer arithmetic.
                            if (node->ChildA >= 0)
                                PushSame(node->ChildA, node->LeafCountA, ref queue, ref pairsToTest);
                            if (node->ChildB >= 0)
                                PushSame(node->ChildB, node->LeafCountB, ref queue, ref pairsToTest);

                            if (ab)
                            {
                                TestDifferentNodes(
                                    &node->A, &node->B,
                                    node->ChildA, node->ChildB,
                                    node->LeafCountA, node->LeafCountB,
                                    ref queue, ref pairsToTest, ref results);
                            }
                        }
                        break;
                    case PairType.InternalInternal:
                        {
                            var a = nodes + pairToTest.A;
                            var b = nodes + pairToTest.B;
                            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
                            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
                            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
                            var bb = BoundingBox.Intersects(ref a->B, ref b->B);

                            if (aa)
                            {
                                TestDifferentNodes(
                                    &a->A, &b->A,
                                    a->ChildA, b->ChildA,
                                    a->LeafCountA, b->LeafCountA,
                                    ref queue, ref pairsToTest, ref results);
                            }
                            if (ab)
                            {
                                TestDifferentNodes(
                                    &a->A, &b->B,
                                    a->ChildA, b->ChildB,
                                    a->LeafCountA, b->LeafCountB,
                                    ref queue, ref pairsToTest, ref results);
                            }
                            if (ba)
                            {
                                TestDifferentNodes(
                                    &a->B, &b->A,
                                    a->ChildB, b->ChildA,
                                    a->LeafCountB, b->LeafCountA,
                                    ref queue, ref pairsToTest, ref results);
                            }
                            if (bb)
                            {
                                TestDifferentNodes(
                                    &a->B, &b->B,
                                    a->ChildB, b->ChildB,
                                    a->LeafCountB, b->LeafCountB,
                                    ref queue, ref pairsToTest, ref results);
                            }
                        }
                        break;
                    case PairType.LeafInternal:
                        {
                            var internalNode = nodes + pairToTest.A;
                            var a = BoundingBox.Intersects(ref *pairToTest.LeafBounds, ref internalNode->A);
                            var b = BoundingBox.Intersects(ref *pairToTest.LeafBounds, ref internalNode->B);
                            if (a)
                            {
                                if (internalNode->ChildA >= 0)
                                {
                                    PushLeafInternal(internalNode->ChildA, internalNode->LeafCountA, pairToTest.LeafBounds, pairToTest.B, ref queue, ref pairsToTest);
                                }
                                else
                                {
                                    results.Add(new Overlap { A = Encode(pairToTest.B), B = Encode(internalNode->ChildA) });
                                }
                            }
                            if (b)
                            {
                                if (internalNode->ChildB >= 0)
                                {
                                    PushLeafInternal(internalNode->ChildB, internalNode->LeafCountB, pairToTest.LeafBounds, pairToTest.B, ref queue, ref pairsToTest);
                                }
                                else
                                {
                                    results.Add(new Overlap { A = Encode(pairToTest.B), B = Encode(internalNode->ChildB) });
                                }
                            }
                        }
                        break;
                }

            }

            for (int i = 0; i < queue.Count; ++i)
            {
                var pair = pairsToTest[queue.Entries[i].Id];
                switch (pair.Type)
                {
                    case PairType.SameNode:
                        testPairs.Add(new Overlap { A = pair.A, B = pair.A });
                        break;
                    case PairType.InternalInternal:
                    case PairType.LeafInternal:
                        testPairs.Add(new Overlap { A = pair.A, B = pair.B });
                        break;

                }
            }

            pairsToTest.Count = 0;
            pairsToTest.Dispose();
        }
    }
}
