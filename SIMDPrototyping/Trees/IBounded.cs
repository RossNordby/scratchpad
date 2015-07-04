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
}
