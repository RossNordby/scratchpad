using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{

    public interface IBounded
    {
        void GetBoundingBox(out BoundingBox box);
    }
    public class Tree<T> where T : IBounded
    {
        Node[][] Levels;

        

        public void Insert(T leaf)
        {
            int levelIndex = 0;
            int nodeIndex = 0;
            BoundingBox aosBox;
            leaf.GetBoundingBox(out aosBox);
            var box = new BoundingBoxWide(ref aosBox);
            while (true)
            {
                //Which child should the leaf belong to?
                BoundingBoxWide merged;
                BoundingBoxWide.Merge(ref Levels[levelIndex][nodeIndex].BoundingBoxes, ref box, out merged);
                Vector<float> volumes;
                BoundingBoxWide.ComputeVolume(ref merged, out volumes);
            }
        }

    }
}
