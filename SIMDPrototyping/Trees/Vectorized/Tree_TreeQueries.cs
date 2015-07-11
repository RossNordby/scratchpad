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
                if (intersectionMask[i] < 0)
                {
                    if (Levels[levelIndex].Nodes[nodeIndex].Children[i] < -1)
                    {
                        var otherLeaf = Encode(Levels[levelIndex].Nodes[nodeIndex].Children[i]);
                        results.Add(new Overlap { A = leaf, B = otherLeaf });
                    }
                    else if (Levels[levelIndex].Nodes[nodeIndex].Children[i] >= 0)
                    {
                        TestLeafAgainstNode(leaf, ref leafBounds, nextLevel, Levels[levelIndex].Nodes[nodeIndex].Children[i], ref results);
                    }
                }
            }
        }

        unsafe void GetOverlapsInNode<TResultList>(int levelIndex, int nodeIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var node = Levels[levelIndex].Nodes[nodeIndex];
            int nextLevel = levelIndex + 1;

            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                if (node.Children[i] >= 0)
                {
                    GetOverlapsInNode(nextLevel, node.Children[i], ref results);
                }
            }

            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                if (node.Children[i] == -1)
                    continue;
                //REALLY want a shuffle here. If we could just push forward, retest, push forward, retest...
                //Unclear how much of a performance benefit this would actually be, since this is just a series of broadcasts.
                //... not sure it's actually compiled to a series of broadcasts, though.
                Vector<int> intersectionMask;
                BoundingBoxWide aWide;
                BoundingBoxWide.GetBoundingBox(ref node.BoundingBoxes, i, out aWide);
                //var aChild = new Vector<int>(a.Children[i]);
                //var less = Vector.LessThan(aChild, b.Children);
                //var bChildIsVector.LessThan(b.Children, new Vector<int>(-1));
                BoundingBoxWide.Intersects(ref aWide, ref node.BoundingBoxes, out intersectionMask);
                for (int j = i + 1; j < Vector<float>.Count; ++j)
                {

                    //TODO: would it be faster to compute a single combined value via simd and then use a single switch?
                    if (node.Children[j] != -1 && intersectionMask[j] < 0)
                    {
                        //TODO: try removing branches via bool ops
                        if (node.Children[i] >= 0 && node.Children[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, node.Children[i], node.Children[j], ref results);
                        }
                        else if (node.Children[i] < -1 && node.Children[j] >= 0)
                        {
                            //leaf A versus node B.
                            TestLeafAgainstNode(Encode(node.Children[i]), ref aWide, nextLevel, node.Children[j], ref results);
                        }
                        else if (node.Children[i] >= 0 && node.Children[j] < -1)
                        {
                            //leaf B versus node A.
                            BoundingBoxWide bWide;
                            BoundingBoxWide.GetBoundingBox(ref node.BoundingBoxes, j, out bWide);
                            TestLeafAgainstNode(Encode(node.Children[j]), ref bWide, nextLevel, node.Children[i], ref results);
                        }
                        else if (node.Children[i] < -1 && node.Children[j] < -1)
                        {
                            //Two leaves.
                            var leafA = Encode(node.Children[i]);
                            var leafB = Encode(node.Children[j]);
                            results.Add(new Overlap { A = leafA, B = leafB });
                        }
                    }
                }
            }
        }

        unsafe void GetOverlapsBetweenDifferentNodes<TResultList>(int levelIndex, int aIndex, int bIndex, ref TResultList results) where TResultList : IList<Overlap>
        {
            //There are no shared children, so test them all.
            var a = Levels[levelIndex].Nodes[aIndex];
            var b = Levels[levelIndex].Nodes[bIndex];
            int nextLevel = levelIndex + 1;
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                if (a.Children[i] == -1)
                    continue;
                //REALLY want a shuffle here. If we could just push forward, retest, push forward, retest...
                //Unclear how much of a performance benefit this would actually be, since this is just a series of broadcasts.
                //... not sure it's actually compiled to a series of broadcasts, though.
                Vector<int> intersectionMask;
                BoundingBoxWide aWide;
                BoundingBoxWide.GetBoundingBox(ref a.BoundingBoxes, i, out aWide);
                //var aChild = new Vector<int>(a.Children[i]);
                //var less = Vector.LessThan(aChild, b.Children);
                //var bChildIsVector.LessThan(b.Children, new Vector<int>(-1));
                BoundingBoxWide.Intersects(ref aWide, ref b.BoundingBoxes, out intersectionMask);
                for (int j = 0; j < Vector<float>.Count; ++j)
                {

                    //TODO: would it be faster to compute a single combined value via simd and then use a single switch?
                    if (b.Children[j] != -1 && intersectionMask[j] < 0)
                    {
                        //TODO: try removing branches via bool ops
                        if (a.Children[i] >= 0 && b.Children[j] >= 0)
                        {
                            GetOverlapsBetweenDifferentNodes(nextLevel, a.Children[i], b.Children[j], ref results);
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
                            results.Add(new Overlap { A = leafA, B = leafB });
                        }
                    }
                }
            }
        }


        public void GetSelfOverlaps<TResultList>(ref TResultList results) where TResultList : IList<Overlap>
        {
            GetOverlapsInNode(0, 0, ref results);
            //Console.WriteLine("Tree-based results:");
            //for (int i = 0; i < results.Count; ++i)
            //{
            //    Console.WriteLine($"{results[i].A}, {results[i].B}");
            //}

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
            //Console.WriteLine("Query-based results:");
            //for (int i = 0; i < results.Count; ++i)
            //{
            //    Console.WriteLine($"{results[i].A}, {results[i].B}");
            //}
        }

    }
}
