using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{

    public interface IBounded
    {
        void GetBoundingBox(out BoundingBox box);
    }
    public class Tree<T> 
    {
        Node[][] Levels;



        public void Insert(T leaf)
        {
            int levelIndex = 0;
            int nodeIndex = 0;
            while (true)
            {
                //Which child should the leaf belong to?
                Levels[levelIndex][nodeIndex].;
            }
        }

    }
}
