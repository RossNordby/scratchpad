using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemoRenderer.Shapes
{
    public class ShapesExtractor
    {
        //For now, we only have spheres. Later, once other shapes exist, this will be responsible for bucketing the different shape types and when necessary caching shape models.
        internal QuickList<SphereInstance, Array<SphereInstance>> spheres;

        ParallelLooper looper;
        public ShapesExtractor(ParallelLooper looper, int initialCapacityPerShapeType = 1024)
        {
            var initialSpheresSpan = new Array<SphereInstance>(new SphereInstance[initialCapacityPerShapeType]);
            spheres = new QuickList<SphereInstance, Array<SphereInstance>>(ref initialSpheresSpan);
            this.looper = looper;
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
        }

        public void AddInstances(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            spheres.EnsureCapacity(simulation.Bodies.BodyCount, new PassthroughArrayPool<SphereInstance>());
            for (int i = 0; i < simulation.Bodies.BodyCount; ++i)
            {
                simulation.Bodies.GetPoseByIndex(i, out var pose);
                SphereInstance instance;
                instance.Position = pose.Position;
                instance.Radius = 0.5f;
                instance.Orientation = pose.Orientation;
                spheres.AddUnsafely(instance);

            }
        }
    }
}
