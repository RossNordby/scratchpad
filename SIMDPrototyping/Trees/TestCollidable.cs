using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{
    public class TestCollidable : IBounded
    {
        public BoundingBox BoundingBox;
        public void GetBoundingBox(out BoundingBox box)
        {
            box = this.BoundingBox;
        }
    }
}
