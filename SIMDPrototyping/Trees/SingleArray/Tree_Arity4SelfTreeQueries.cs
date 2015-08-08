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
#if NODE4
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForLeaf4<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (nodeIndex < 0)
            {
                results.Add(new Overlap { A = leafIndex, B = Encode(nodeIndex) });
            }
            else
            {
                TestLeafAgainstNode4(leafIndex, ref leafBounds, nodeIndex, ref results);
            }
        }
        unsafe void TestLeafAgainstNode4<TResultList>(int leafIndex, ref BoundingBox leafBounds, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            var node = nodes + nodeIndex;


            //TODO: did you realize this trick on the other queries?
            var a = BoundingBox.Intersects(ref leafBounds, ref node->A);
            var b = BoundingBox.Intersects(ref leafBounds, ref node->B);
            //TODO: use binary or early out? or swap to ifs so first condition catches second one?
            //var c = node->ChildCount > 2 && BoundingBox.Intersects(ref leafBounds, ref node->C);
            //var d = node->ChildCount > 3 && BoundingBox.Intersects(ref leafBounds, ref node->D);
            bool c, d;
            if (node->ChildCount > 2)
            {
                c = BoundingBox.Intersects(ref leafBounds, ref node->C);
                if (node->ChildCount > 3)
                    d = BoundingBox.Intersects(ref leafBounds, ref node->D);
                else
                    d = false;
            }
            else
            {
                c = false;
                d = false;
            }

            //TODO: cache or no cache?
            var nodeChildA = node->ChildA;
            var nodeChildB = node->ChildB;
            var nodeChildC = node->ChildC;
            var nodeChildD = node->ChildD;

            if (a)
            {
                DispatchTestForLeaf4(leafIndex, ref leafBounds, nodeChildA, ref results);
            }
            if (b)
            {
                DispatchTestForLeaf4(leafIndex, ref leafBounds, nodeChildB, ref results);
            }
            if (c)
            {
                DispatchTestForLeaf4(leafIndex, ref leafBounds, nodeChildC, ref results);
            }
            if (d)
            {
                DispatchTestForLeaf4(leafIndex, ref leafBounds, nodeChildD, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForNodes4<TResultList>(int childA, int childB, ref BoundingBox boundsA, ref BoundingBox boundsB, ref TResultList results) where TResultList : IList<Overlap>
        {
            if (childA >= 0)
            {
                if (childB >= 0)
                {
                    GetOverlapsBetweenDifferentNodes4If(nodes + childA, nodes + childB, ref results);
                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode4(Encode(childB), ref boundsB, childA, ref results);
                }
            }
            else if (childB >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode4(Encode(childA), ref boundsA, childB, ref results);
            }
            else
            {
                //Two leaves.
                results.Add(new Overlap { A = Encode(childA), B = Encode(childB) });
            }
        }

        unsafe void GetOverlapsBetweenDifferentNodes4Switch<TResultList>(Node* a, Node* b, ref TResultList results) where TResultList : IList<Overlap>
        {
            //All of these are guaranteed to be needed.
            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
            var bb = BoundingBox.Intersects(ref a->B, ref b->B);

            switch (a->ChildCount)
            {

                case 4:
                    {
                        var ca = BoundingBox.Intersects(ref a->C, ref b->A);
                        var cb = BoundingBox.Intersects(ref a->C, ref b->B);
                        var da = BoundingBox.Intersects(ref a->D, ref b->A);
                        var db = BoundingBox.Intersects(ref a->D, ref b->B);

                        switch (b->ChildCount)
                        {
                            case 4:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);
                                    var cc = BoundingBox.Intersects(ref a->C, ref b->C);
                                    var dc = BoundingBox.Intersects(ref a->D, ref b->C);
                                    var ad = BoundingBox.Intersects(ref a->A, ref b->D);
                                    var bd = BoundingBox.Intersects(ref a->B, ref b->D);
                                    var cd = BoundingBox.Intersects(ref a->C, ref b->D);
                                    var dd = BoundingBox.Intersects(ref a->D, ref b->D);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ad)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildD, ref a->A, ref b->D, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                    if (bd)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildD, ref a->B, ref b->D, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                    if (cc)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
                                    }
                                    if (cd)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildD, ref a->C, ref b->D, ref results);
                                    }
                                    if (da)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildA, ref a->D, ref b->A, ref results);
                                    }
                                    if (db)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildB, ref a->D, ref b->B, ref results);
                                    }
                                    if (dc)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildC, ref a->D, ref b->C, ref results);
                                    }
                                    if (dd)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildD, ref a->D, ref b->D, ref results);
                                    }
                                }
                                break;
                            case 3:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);
                                    var cc = BoundingBox.Intersects(ref a->C, ref b->C);
                                    var dc = BoundingBox.Intersects(ref a->D, ref b->C);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                    if (cc)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
                                    }
                                    if (da)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildA, ref a->D, ref b->A, ref results);
                                    }
                                    if (db)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildB, ref a->D, ref b->B, ref results);
                                    }
                                    if (dc)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildC, ref a->D, ref b->C, ref results);
                                    }
                                }
                                break;
                            default:
                                {
                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                    if (da)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildA, ref a->D, ref b->A, ref results);
                                    }
                                    if (db)
                                    {
                                        DispatchTestForNodes4(a->ChildD, b->ChildB, ref a->D, ref b->B, ref results);
                                    }
                                }
                                break;
                        }


                    }
                    break;
                case 3:
                    {
                        var ca = BoundingBox.Intersects(ref a->C, ref b->A);
                        var cb = BoundingBox.Intersects(ref a->C, ref b->B);

                        switch (b->ChildCount)
                        {
                            case 4:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);
                                    var cc = BoundingBox.Intersects(ref a->C, ref b->C);
                                    var ad = BoundingBox.Intersects(ref a->A, ref b->D);
                                    var bd = BoundingBox.Intersects(ref a->B, ref b->D);
                                    var cd = BoundingBox.Intersects(ref a->C, ref b->D);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ad)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildD, ref a->A, ref b->D, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                    if (bd)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildD, ref a->B, ref b->D, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                    if (cc)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
                                    }
                                    if (cd)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildD, ref a->C, ref b->D, ref results);
                                    }
                                }
                                break;
                            case 3:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);
                                    var cc = BoundingBox.Intersects(ref a->C, ref b->C);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                    if (cc)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
                                    }
                                }
                                break;
                            default:
                                {
                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (ca)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                                    }
                                    if (cb)
                                    {
                                        DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                                    }
                                }
                                break;
                        }
                    }
                    break;
                default:
                    {
                        switch (b->ChildCount)
                        {
                            case 4:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);
                                    var ad = BoundingBox.Intersects(ref a->A, ref b->D);
                                    var bd = BoundingBox.Intersects(ref a->B, ref b->D);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ad)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildD, ref a->A, ref b->D, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                    if (bd)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildD, ref a->B, ref b->D, ref results);
                                    }
                                }
                                break;
                            case 3:
                                {
                                    var ac = BoundingBox.Intersects(ref a->A, ref b->C);
                                    var bc = BoundingBox.Intersects(ref a->B, ref b->C);

                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ac)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                    if (bc)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                                    }
                                }
                                break;
                            default:
                                {
                                    if (aa)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
                                    }
                                    if (ab)
                                    {
                                        DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
                                    }
                                    if (ba)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
                                    }
                                    if (bb)
                                    {
                                        DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
                                    }
                                }
                                break;
                        }

                    }
                    break;
            }


        }

        unsafe void GetOverlapsBetweenDifferentNodes4If<TResultList>(Node* a, Node* b, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
            var bb = BoundingBox.Intersects(ref a->B, ref b->B);


            var ac = BoundingBox.Intersects(ref a->A, ref b->C);
            var ad = BoundingBox.Intersects(ref a->A, ref b->D);
            var bc = BoundingBox.Intersects(ref a->B, ref b->C);
            var bd = BoundingBox.Intersects(ref a->B, ref b->D);

            if (aa)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
            }
            if (ab)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
            }
            if (b->ChildCount > 2)
            {
                if (ac)
                {
                    DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
                }
                if (b->ChildCount > 3 & ad)
                {
                    DispatchTestForNodes4(a->ChildA, b->ChildD, ref a->A, ref b->D, ref results);
                }
            }

            if (ba)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
            }
            if (bb)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
            }
            if (b->ChildCount > 2)
            {
                if (bc)
                {
                    DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
                }
                if (b->ChildCount > 3 & bd)
                {
                    DispatchTestForNodes4(a->ChildB, b->ChildD, ref a->B, ref b->D, ref results);
                }
            }

            if (a->ChildCount > 2)
            {

                var ca = BoundingBox.Intersects(ref a->C, ref b->A);
                var cb = BoundingBox.Intersects(ref a->C, ref b->B);
                var cc = BoundingBox.Intersects(ref a->C, ref b->C);
                var cd = BoundingBox.Intersects(ref a->C, ref b->D);

                if (ca)
                {
                    DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
                }
                if (cb)
                {
                    DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
                }
                if (b->ChildCount > 2)
                {
                    if (cc)
                    {
                        DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
                    }
                    if (b->ChildCount > 3 & cd)
                    {
                        DispatchTestForNodes4(a->ChildC, b->ChildD, ref a->C, ref b->D, ref results);
                    }
                }

                if (a->ChildCount > 3)
                {

                    var da = BoundingBox.Intersects(ref a->D, ref b->A);
                    var db = BoundingBox.Intersects(ref a->D, ref b->B);
                    var dc = BoundingBox.Intersects(ref a->D, ref b->C);
                    var dd = BoundingBox.Intersects(ref a->D, ref b->D);

                    if (da)
                    {
                        DispatchTestForNodes4(a->ChildD, b->ChildA, ref a->D, ref b->A, ref results);
                    }
                    if (db)
                    {
                        DispatchTestForNodes4(a->ChildD, b->ChildB, ref a->D, ref b->B, ref results);
                    }
                    if (b->ChildCount > 2)
                    {
                        if (dc)
                        {
                            DispatchTestForNodes4(a->ChildD, b->ChildC, ref a->D, ref b->C, ref results);
                        }
                        if (b->ChildCount > 3 & dd)
                        {
                            DispatchTestForNodes4(a->ChildD, b->ChildD, ref a->D, ref b->D, ref results);
                        }
                    }
                }
            }
        }

        unsafe void GetOverlapsBetweenDifferentNodes4<TResultList>(Node* a, Node* b, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var aa = BoundingBox.Intersects(ref a->A, ref b->A);
            var ab = BoundingBox.Intersects(ref a->A, ref b->B);
            var ac = b->ChildCount > 2 && BoundingBox.Intersects(ref a->A, ref b->C);
            var ad = b->ChildCount > 3 && BoundingBox.Intersects(ref a->A, ref b->D);

            var ba = BoundingBox.Intersects(ref a->B, ref b->A);
            var bb = BoundingBox.Intersects(ref a->B, ref b->B);
            var bc = b->ChildCount > 2 && BoundingBox.Intersects(ref a->B, ref b->C);
            var bd = b->ChildCount > 3 && BoundingBox.Intersects(ref a->B, ref b->D);

            var ca = a->ChildCount > 2 && BoundingBox.Intersects(ref a->C, ref b->A);
            var cb = a->ChildCount > 2 && BoundingBox.Intersects(ref a->C, ref b->B);
            var cc = a->ChildCount > 2 && b->ChildCount > 2 && BoundingBox.Intersects(ref a->C, ref b->C);
            var cd = a->ChildCount > 2 && b->ChildCount > 3 && BoundingBox.Intersects(ref a->C, ref b->D);

            var da = a->ChildCount > 3 && BoundingBox.Intersects(ref a->D, ref b->A);
            var db = a->ChildCount > 3 && BoundingBox.Intersects(ref a->D, ref b->B);
            var dc = a->ChildCount > 3 && b->ChildCount > 2 && BoundingBox.Intersects(ref a->D, ref b->C);
            var dd = a->ChildCount > 3 && b->ChildCount > 3 && BoundingBox.Intersects(ref a->D, ref b->D);


            if (aa)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildA, ref a->A, ref b->A, ref results);
            }
            if (ab)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildB, ref a->A, ref b->B, ref results);
            }


            if (ac)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildC, ref a->A, ref b->C, ref results);
            }
            if (ad)
            {
                DispatchTestForNodes4(a->ChildA, b->ChildD, ref a->A, ref b->D, ref results);
            }

            if (ba)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildA, ref a->B, ref b->A, ref results);
            }
            if (bb)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildB, ref a->B, ref b->B, ref results);
            }
            if (bc)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildC, ref a->B, ref b->C, ref results);
            }
            if (bd)
            {
                DispatchTestForNodes4(a->ChildB, b->ChildD, ref a->B, ref b->D, ref results);
            }

            if (ca)
            {
                DispatchTestForNodes4(a->ChildC, b->ChildA, ref a->C, ref b->A, ref results);
            }
            if (cb)
            {
                DispatchTestForNodes4(a->ChildC, b->ChildB, ref a->C, ref b->B, ref results);
            }
            if (cc)
            {
                DispatchTestForNodes4(a->ChildC, b->ChildC, ref a->C, ref b->C, ref results);
            }
            if (cd)
            {
                DispatchTestForNodes4(a->ChildC, b->ChildD, ref a->C, ref b->D, ref results);
            }

            if (da)
            {
                DispatchTestForNodes4(a->ChildD, b->ChildA, ref a->D, ref b->A, ref results);
            }
            if (db)
            {
                DispatchTestForNodes4(a->ChildD, b->ChildB, ref a->D, ref b->B, ref results);
            }
            if (dc)
            {
                DispatchTestForNodes4(a->ChildD, b->ChildC, ref a->D, ref b->C, ref results);
            }
            if (dd)
            {
                DispatchTestForNodes4(a->ChildD, b->ChildD, ref a->D, ref b->D, ref results);
            }
        }

        unsafe void GetOverlapsInNode4<TResultList>(Node* node, ref TResultList results) where TResultList : IList<Overlap>
        {

            var ab = BoundingBox.Intersects(ref node->A, ref node->B);

            if (node->ChildA >= 0)
                GetOverlapsInNode4(nodes + node->ChildA, ref results);
            if (node->ChildB >= 0)
                GetOverlapsInNode4(nodes + node->ChildB, ref results);

            bool ac, bc, ad, bd, cd;
            if (node->ChildCount > 2)
            {
                ac = BoundingBox.Intersects(ref node->A, ref node->C);
                bc = BoundingBox.Intersects(ref node->B, ref node->C);

                if (node->ChildC >= 0)
                    GetOverlapsInNode4(nodes + node->ChildC, ref results);

                if (node->ChildCount > 3)
                {
                    ad = BoundingBox.Intersects(ref node->A, ref node->D);
                    bd = BoundingBox.Intersects(ref node->B, ref node->D);
                    cd = BoundingBox.Intersects(ref node->C, ref node->D);

                    if (node->ChildD >= 0)
                    {
                        GetOverlapsInNode4(nodes + node->ChildD, ref results);
                    }
                }
                else
                {
                    ad = false;
                    bd = false;
                    cd = false;
                }
            }
            else
            {
                ac = false;
                bc = false;
                ad = false;
                bd = false;
                cd = false;
            }


            //Test all different nodes.
            if (ab)
            {
                DispatchTestForNodes4(node->ChildA, node->ChildB, ref node->A, ref node->B, ref results);
            }
            if (ac)
            {
                DispatchTestForNodes4(node->ChildA, node->ChildC, ref node->A, ref node->C, ref results);
            }
            if (ad)
            {
                DispatchTestForNodes4(node->ChildA, node->ChildD, ref node->A, ref node->D, ref results);
            }
            if (bc)
            {
                DispatchTestForNodes4(node->ChildB, node->ChildC, ref node->B, ref node->C, ref results);
            }
            if (bd)
            {
                DispatchTestForNodes4(node->ChildB, node->ChildD, ref node->B, ref node->D, ref results);
            }
            if (cd)
            {
                DispatchTestForNodes4(node->ChildC, node->ChildD, ref node->C, ref node->D, ref results);
            }

        }
        public unsafe void GetSelfOverlaps4<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            GetOverlapsInNode4(nodes, ref results);

        }



#endif


    }
}
