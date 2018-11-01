using System;
using System.Collections.Generic;
using System.Text;

namespace GoingWide
{
    public abstract class Benchmark
    {
        protected ArenaPool pool;

        public const int LaneCount = 1 << 20;

        public Benchmark()
        {
            pool = new ArenaPool(minimumBlockSizeInBytes: LaneCount << 4);
        }
                
        public abstract void Execute();

        public void Reset()
        {
            pool.Reset();
        }
    }
}
