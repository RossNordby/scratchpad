using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Text;

namespace SolverPrototypeTests
{
    public class SimpleDemo : Demo
    {
        public override void Initialize()
        {
            Simulation = new Simulation(BufferPool);
        }
    
    }
}
