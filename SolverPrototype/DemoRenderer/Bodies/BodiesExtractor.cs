using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemoRenderer.Bodies
{
    public class BodiesExtractor
    {
        //For now, we only have spheres. Later, once other shapes exist, this will be responsible for bucketing the different shape types and when necessary caching shape models.
        internal QuickList<SphereInstance, Array<SphereInstance>> spheres;

        public BodiesExtractor(int initialSizePerShape)
        {
            var initialSpheresSpan = new Array<SphereInstance>(new SphereInstance[initialSizePerShape]);
            spheres = new QuickList<SphereInstance, Array<SphereInstance>>(ref initialSpheresSpan);
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
        }

        public void AddInstances(Simulation simulation)
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
