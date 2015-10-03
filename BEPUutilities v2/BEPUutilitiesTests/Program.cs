using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;

namespace BEPUutilitiesTests
{
    class Program
    {
       

        static void Main(string[] args)
        {
            Vector3Tests.Test(); 
            Matrix4x4Tests.Test();
        }
    }
}
