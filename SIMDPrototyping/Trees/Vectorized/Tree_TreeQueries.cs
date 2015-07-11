using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Vectorized
{
    public struct Overlap
    {
        public int A;
        public int B;
    }
    partial class Tree<T>
    {
        unsafe void TestLeafAgainstNode<TResultList>(int leaf, ref BoundingBoxWide leafBounds, int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            int nextLevel = levelIndex + 1;
            Vector<int> intersectionMask;
            BoundingBoxWide.Intersects(ref leafBounds, ref Levels[levelIndex].Nodes[nodeIndex].BoundingBoxes, out intersectionMask);
            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                if (Levels[levelIndex].Nodes[nodeIndex].Children[i] < 0)
                {
                    results.Add(new Overlap { A = leaf, B = Encode(Levels[levelIndex].Nodes[nodeIndex].Children[i]) });
                }
                else
                {
                    TestLeafAgainstNode(leaf, ref leafBounds, nextLevel, Levels[levelIndex].Nodes[nodeIndex].Children[i], ref results);
                }
            }
        }


        unsafe void GetOverlapsBetweenNodes<TResultList>(int levelIndex, int aIndex, int bIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var a = Levels[levelIndex].Nodes[aIndex];
            var b = Levels[levelIndex].Nodes[bIndex];
            int nextLevel = levelIndex + 1;
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                //REALLY want a shuffle here. If we could just push forward, retest, push forward, retest... it would be so much faster.
                Vector<int> intersectionMask;
                BoundingBoxWide aWide;
                BoundingBoxWide.GetBoundingBox(ref Levels[levelIndex].Nodes[aIndex].BoundingBoxes, i, out aWide);
                //var aChild = new Vector<int>(a.Children[i]);
                //var less = Vector.LessThan(aChild, b.Children);
                //var bChildIsVector.LessThan(b.Children, new Vector<int>(-1));
                BoundingBoxWide.Intersects(ref aWide, ref b.BoundingBoxes, out intersectionMask);
                for (int j = 0; j < Vector<float>.Count; ++j)
                {

                    //TODO: would it be faster to compute a single combined value via simd and then use a single switch?
                    if (intersectionMask[j] < 0)
                    {
                        //TODO: try removing branches via bool ops
                        if (a.Children[i] >= 0 && b.Children[j] >= 0)
                        {
                            GetOverlapsBetweenNodes(nextLevel, a.Children[i], b.Children[j], ref results);
                        }
                        else if (a.Children[i] < -1 && b.Children[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(Encode(a.Children[i]), ref aWide, nextLevel, b.Children[j], ref results);
                        }
                        else if (a.Children[i] >= 0 && b.Children[j] < -1)
                        {
                            //leaf B versus node A.
                            BoundingBoxWide bWide;
                            BoundingBoxWide.GetBoundingBox(ref b.BoundingBoxes, j, out bWide);
                            TestLeafAgainstNode(Encode(b.Children[j]), ref bWide, nextLevel, a.Children[i], ref results);
                        }
                        else if (a.Children[i] < -1 && b.Children[j] < -1)
                        {
                            //Two leaves.
                            var leafA = Encode(a.Children[i]);
                            var leafB = Encode(b.Children[j]);
                            if (leafA < leafB)//Make sure the leaf isn't itself.
                                results.Add(new Overlap { A = leafA, B = leafB });
                        }
                    }
                }
            }
        }


        public void GetSelfOverlaps<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            GetOverlapsBetweenNodes(0, 0, 0, ref results);

        }

        public unsafe void GetSelfOverlapsViaQueries<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            var leafQueryResults = new QuickList<int>(BufferPools<int>.Thread);

            for (int i = 0; i < leafCount; ++i)
            {
                var leaf = leaves[i];
                BoundingBoxWide leafBoundingBox;
                BoundingBoxWide.GetBoundingBox(ref Levels[leaf.LevelIndex].Nodes[leaf.NodeIndex].BoundingBoxes, leaf.ChildIndex, out leafBoundingBox);
                TestRecursive(0, 0, ref leafBoundingBox, ref leafQueryResults);
                for (int j = 0; j < leafQueryResults.Count; ++j)
                {
                    //Only include results which which are forward in the list to avoid self tests.
                    if (i < leafQueryResults.Elements[j])
                    {
                        results.Add(new Overlap { A = i, B = leafQueryResults.Elements[j] });
                    }
                }
                leafQueryResults.Count = 0;
            }
            leafQueryResults.Dispose();
        }

    }
}
