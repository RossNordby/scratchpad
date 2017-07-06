using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;

namespace DemoRenderer.Constraints
{
    public class LineExtractor
    {
        internal QuickList<LineInstance, Array<LineInstance>> lines;
        ConstraintLineExtractor constraints;
        BoundingBoxLineExtractor boundingBoxes;

        ParallelLooper looper;
        public LineExtractor(ParallelLooper looper, int initialLineCapacity = 8192)
        {
            QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), initialLineCapacity, out lines);
            constraints = new ConstraintLineExtractor();
            boundingBoxes = new BoundingBoxLineExtractor();
            this.looper = looper;
        }

        public void Extract(Simulation simulation, bool showConstraints = true, bool showContacts = false, bool showBoundingBoxes = false, IThreadDispatcher threadDispatcher = null)
        {
            looper.Dispatcher = threadDispatcher;
            if (showConstraints)
                constraints.AddInstances(simulation, showConstraints, showContacts, ref lines, looper);
            if (showBoundingBoxes)
                boundingBoxes.AddInstances(simulation, ref lines, looper);
        }
        public void ClearInstances()
        {
            lines.Count = 0;
        }

    }
}
