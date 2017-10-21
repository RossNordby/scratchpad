﻿using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
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

        void AddShape(Simulation simulation, TypedIndex shapeIndex, ref RigidPose pose)
        {
            switch (shapeIndex.Type)
            {
                case Sphere.Id:
                    {
                        SphereInstance instance;
                        instance.Position = pose.Position;
                        instance.Radius = simulation.Shapes.GetShape<Sphere>(shapeIndex.Index).Radius;
                        instance.Orientation = pose.Orientation;
                        spheres.AddUnsafely(instance);
                    }
                    break;
            }
        }

        public void AddInstances(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            spheres.EnsureCapacity(simulation.Bodies.Count, new PassthroughArrayPool<SphereInstance>());
            for (int i = 0; i < simulation.Bodies.Count; ++i)
            {
                AddShape(simulation, simulation.Bodies.Collidables[i].Shape, ref simulation.Bodies.Poses[i]);
            }
            for (int i = 0; i < simulation.Statics.Count; ++i)
            {
                AddShape(simulation, simulation.Statics.Collidables[i].Shape, ref simulation.Statics.Poses[i]);
            }
        }
    }
}
