using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Baseline
{
    public struct Overlap<T>
    {
        public T A;
        public T B;
    }
    partial class Tree<T>
    {
        unsafe void TestLeafAgainstNode<TResultList>(T leaf, ref BoundingBox leafBounds, int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap<T>>
        {
            var node = Levels[levelIndex].Nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            int nextLevel = levelIndex + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (BoundingBox.Intersects(ref leafBounds, ref bounds[i]))
                {
                    if (children[i] < 0)
                    {
                        results.Add(new Overlap<T> { A = leaf, B = leaves[Encode(children[i])].Bounded });
                    }
                    else
                    {
                        TestLeafAgainstNode(leaf, ref leafBounds, nextLevel, children[i], ref results);
                    }
                }
            }
        }


        unsafe void GetOverlapsBetweenDifferentNodes<TResultList>(int levelIndex, int aIndex, int bIndex, ref TResultList results) where TResultList : IList<Overlap<T>>
        {
            var a = Levels[levelIndex].Nodes + aIndex;
            var b = Levels[levelIndex].Nodes + bIndex;
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
                        if (aChildren[i] >= 0 && bChildren[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, aChildren[i], bChildren[j], ref results);
                        }
                        else if (aChildren[i] < 0 && bChildren[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(leaves[Encode(aChildren[i])].Bounded, ref aBounds[i], nextLevel, bChildren[j], ref results);
                        }
                        else if (aChildren[i] >= 0 && bChildren[j] < 0)
                        {
                            //leaf B versus node A.
                            TestLeafAgainstNode(leaves[Encode(bChildren[j])].Bounded, ref bBounds[j], nextLevel, aChildren[i], ref results);
                        }
                        else
                        {
                            //Two leaves.
                            results.Add(new Overlap<T> { A = leaves[Encode(aChildren[i])].Bounded, B = leaves[Encode(bChildren[j])].Bounded });
                        }
                    }
                }
            }
        }

        unsafe void GetOverlapsInNode<TResultList>(int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap<T>>
        {
            var node = Levels[levelIndex].Nodes + nodeIndex;
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
                        if (children[i] >= 0 && children[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, children[i], children[j], ref results);
                        }
                        else if (children[i] < 0 && children[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(leaves[Encode(children[i])].Bounded, ref bounds[i], nextLevel, children[j], ref results);
                        }
                        else if (children[i] >= 0 && children[j] < 0)
                        {
                            //leaf B versus node A.
                            TestLeafAgainstNode(leaves[Encode(children[j])].Bounded, ref bounds[j], nextLevel, children[i], ref results);
                        }
                        else
                        {
                            //Two leaves.
                            results.Add(new Overlap<T> { A = leaves[Encode(children[i])].Bounded, B = leaves[Encode(children[j])].Bounded });
                        }
                    }
                }
            }

        }
        public void GetSelfOverlaps<TResultList>(ref TResultList results) where TResultList : IList<Overlap<T>>
        {
            GetOverlapsInNode(0, 0, ref results);

        }

        public unsafe void GetSelfOverlapsViaQueries<TResultList>(ref TResultList results) where TResultList : IList<Overlap<T>>
        {
            var leafQueryResults = new QuickList<int>(BufferPools<int>.Thread);

            for (int i = 0; i < leafCount; ++i)
            {
                var leaf = leaves[i];
                var leafBoundingBox = (&Levels[leaf.LevelIndex].Nodes[leaf.NodeIndex].A)[leaf.ChildIndex];
                QueryRecursive(ref leafBoundingBox, ref leafQueryResults);
                for (int j = 0; j < leafQueryResults.Count; ++j)
                {
                    //Only include results which which are forward in the list to avoid self tests.
                    if (i < leafQueryResults.Elements[j])
                    {
                        results.Add(new Overlap<T> { A = leaf.Bounded, B = leaves[leafQueryResults.Elements[j]].Bounded });
                    }
                }
                leafQueryResults.Count = 0;
            }
            leafQueryResults.Dispose();
        }
    }
}
