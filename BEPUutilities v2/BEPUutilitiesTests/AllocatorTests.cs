using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace BEPUutilitiesTests
{
    [TestClass]
    public static class AllocatorTests
    {

        [TestMethod]
        public static void TestChurnStability()
        {
            var allocator = new Allocator(2048);
            var random = new Random(5);
            ulong idCounter = 0;
            var allocatedIds = new QuickList<ulong>(BufferPools<ulong>.Locking);
            var unallocatedIds = new QuickList<ulong>(BufferPools<ulong>.Locking);
            for (int i = 0; i < 512; ++i)
            {
                long start;
                var id = idCounter++;
                //allocator.ValidatePointers();
                if (allocator.Allocate(id, 1 + random.Next(5), out start))
                {
                    allocatedIds.Add(id);
                }
                else
                {
                    unallocatedIds.Add(id);
                }
                //allocator.ValidatePointers();
            }
            for (int timestepIndex = 0; timestepIndex < 100000; ++timestepIndex)
            {
                //First add and remove a bunch randomly.
                for (int i = random.Next(Math.Min(allocatedIds.Count, 15)); i >= 0; --i)
                {
                    var indexToRemove = random.Next(allocatedIds.Count);
                    //allocator.ValidatePointers();
                    Assert.IsTrue(allocator.Deallocate(allocatedIds.Elements[indexToRemove]));
                    //allocator.ValidatePointers();
                    unallocatedIds.Add(allocatedIds.Elements[indexToRemove]);
                    allocatedIds.FastRemoveAt(indexToRemove);
                }
                for (int i = random.Next(Math.Min(unallocatedIds.Count, 15)); i >= 0; --i)
                {
                    var indexToAllocate = random.Next(unallocatedIds.Count);
                    long start;
                    //allocator.ValidatePointers();
                    if (allocator.Allocate(unallocatedIds.Elements[indexToAllocate], random.Next(3), out start))
                    {
                        //allocator.ValidatePointers();
                        allocatedIds.Add(unallocatedIds.Elements[indexToAllocate]);
                        unallocatedIds.FastRemoveAt(indexToAllocate);
                    }
                    //allocator.ValidatePointers();
                }
                //Check to ensure that everything's still coherent.
                for (int i = 0; i < allocatedIds.Count; ++i)
                {
                    Assert.IsTrue(allocator.Contains(allocatedIds.Elements[i]));
                }
                for (int i = 0; i < unallocatedIds.Count; ++i)
                {
                    Assert.IsFalse(allocator.Contains(unallocatedIds.Elements[i]));
                }
            }
            //Wind it down.
            for (int i = 0; i < allocatedIds.Count; ++i)
            {
                Assert.IsTrue(allocator.Deallocate(allocatedIds.Elements[i]));
            }
            //Confirm cleanup.
            for (int i = 0; i < allocatedIds.Count; ++i)
            {
                Assert.IsFalse(allocator.Contains(allocatedIds.Elements[i]));
            }
            for (int i = 0; i < unallocatedIds.Count; ++i)
            {
                Assert.IsFalse(allocator.Contains(unallocatedIds.Elements[i]));
            }
        }
    }
}
