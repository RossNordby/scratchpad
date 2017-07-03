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
        public bool ShowConstraints { get; set; } = true;
        public bool ShowBoundingBoxes { get; set; } = true;

        public LineExtractor(int initialLineCapacity = 8192)
        {
            QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), initialLineCapacity, out lines);
            constraints = new ConstraintLineExtractor();
            boundingBoxes = new BoundingBoxLineExtractor();
        }

        public void Extract(Simulation simulation)
        {
            if (ShowConstraints)
                constraints.AddInstances(simulation, ref lines);
            if (ShowBoundingBoxes)
                boundingBoxes.AddInstances(simulation, ref lines);
        }
        public void ClearInstances()
        {
            lines.Count = 0;
        }

    }
}
