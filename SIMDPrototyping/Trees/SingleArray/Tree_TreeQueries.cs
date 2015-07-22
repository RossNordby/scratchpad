﻿using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    public struct Overlap
    {
        public int A, B;
    }
    partial class Tree
    {
        unsafe void TestLeafAgainstNode<TResultList>(int leafIndex, ref BoundingBox leafBounds, int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            var node = Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            int nextLevel = levelIndex + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (BoundingBox.Intersects(ref leafBounds, ref bounds[i]))
                {
                    if (children[i] < 0)
                    {
                        results.Add(new Overlap { A = leafIndex, B = Encode(children[i]) });
                    }
                    else
                    {
                        TestLeafAgainstNode(leafIndex, ref leafBounds, nextLevel, children[i], ref results);
                    }
                }
            }
        }


        unsafe void GetOverlapsBetweenDifferentNodes<TResultList>(int levelIndex, int aIndex, int bIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            var a = Nodes + aIndex;
            var b = Nodes + bIndex;
            var aBounds = &a->A;
            var aChildren = &a->ChildA;
            var bBounds = &b->A;
            var bChildren = &b->ChildA;
            int nextLevel = levelIndex + 1;

            //There are no shared children, so test them all.
            for (int i = 0; i < a->ChildCount; ++i)
            {
                for (int j = 0; j < b->ChildCount; ++j)
                {
                    if (BoundingBox.Intersects(ref aBounds[i], ref bBounds[j]))
                    {
                        if (aChildren[i] >= 0 & bChildren[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, aChildren[i], bChildren[j], ref results);
                        }
                        else if (aChildren[i] < 0 & bChildren[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(Encode(aChildren[i]), ref aBounds[i], nextLevel, bChildren[j], ref results);
                        }
                        else if (aChildren[i] >= 0 & bChildren[j] < 0)
                        {
                            //leaf B versus node A.
                            TestLeafAgainstNode(Encode(bChildren[j]), ref bBounds[j], nextLevel, aChildren[i], ref results);
                        }
                        else
                        {
                            //Two leaves.
                            results.Add(new Overlap { A = Encode(aChildren[i]), B = Encode(bChildren[j]) });
                        }
                    }
                }
            }
        }

        unsafe void GetOverlapsInNode<TResultList>(int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            var node = Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            int nextLevel = levelIndex + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    GetOverlapsInNode(nextLevel, children[i], ref results);
                }
            }
            //Test all different nodes.
            for (int i = 0; i < node->ChildCount; ++i)
            {
                for (int j = i + 1; j < node->ChildCount; ++j)
                {
                    if (BoundingBox.Intersects(ref bounds[i], ref bounds[j]))
                    {
                        if (children[i] >= 0 & children[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, children[i], children[j], ref results);
                        }
                        else if (children[i] < 0 & children[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(Encode(children[i]), ref bounds[i], nextLevel, children[j], ref results);
                        }
                        else if (children[i] >= 0 & children[j] < 0)
                        {
                            //leaf B versus node A.
                            TestLeafAgainstNode(Encode(children[j]), ref bounds[j], nextLevel, children[i], ref results);
                        }
                        else
                        {
                            //Two leaves.
                            results.Add(new Overlap { A = Encode(children[i]), B = Encode(children[j]) });
                        }
                    }
                }
            }

        }
        public void GetSelfOverlaps<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            GetOverlapsInNode(0, 0, ref results);

        }

        public unsafe void GetSelfOverlapsViaQueries<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            var leafQueryResults = new QuickList<int>(BufferPools<int>.Thread);

            for (int i = 0; i < leafCount; ++i)
            {
                var leaf = Leaves[i];
                var leafBoundingBox = (&Nodes[leaf.NodeIndex].A)[leaf.ChildIndex];
                QueryRecursive(ref leafBoundingBox, ref leafQueryResults);
                for (int j = 0; j < leafQueryResults.Count; ++j)
                {
                    //Only include results which which are forward in the list to avoid self tests.
                    if (i < leafQueryResults.Elements[j])
                    {
                        results.Add(new Overlap { A = leaf.Id, B = leafQueryResults.Elements[j] });
                    }
                }
                leafQueryResults.Count = 0;
            }
            leafQueryResults.Dispose();
        }

    }
}