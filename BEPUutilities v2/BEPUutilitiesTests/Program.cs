using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    [TestClass]
    class Program
    {
        [TestMethod]
        public static void TestQueueResizing()
        {
            Random random = new Random(5);
            UnsafeBufferPool<int> pool = new UnsafeBufferPool<int>();
            QuickQueue<int> queue = new QuickQueue<int>(pool, 2);
            Queue<int> controlQueue = new Queue<int>();

            for (int iterationIndex = 0; iterationIndex < 1000000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    queue.Enqueue(iterationIndex);
                    controlQueue.Enqueue(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    queue.Dequeue();
                    controlQueue.Dequeue();
                }
                if (iterationIndex % 1000 == 0)
                {
                    queue.EnsureCapacity(queue.Count * 3);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    queue.Compact();
                }
            }

            Assert.IsTrue(queue.Count == controlQueue.Count);
            while (queue.Count > 0)
            {
                var a = queue.Dequeue();
                var b = controlQueue.Dequeue();
                Assert.IsTrue(a == b);
                Assert.IsTrue(queue.Count == controlQueue.Count);
            }
        }

        [TestMethod]
        public static void TestListResizing()
        {
            Random random = new Random(5);
            UnsafeBufferPool<int> pool = new UnsafeBufferPool<int>();
            QuickList<int> list = new QuickList<int>(pool, 2);
            List<int> controlList = new List<int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    list.Add(iterationIndex);
                    controlList.Add(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(list.Count);
                    list.RemoveAt(indexToRemove);
                    controlList.RemoveAt(indexToRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    list.EnsureCapacity(list.Count * 3);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    list.Compact();
                }
            }

            Assert.IsTrue(list.Count == controlList.Count);
            for (int i = 0; i < list.Count; ++i)
            {
                var a = list[i];
                var b = controlList[i];
                Assert.IsTrue(a == b);
                Assert.IsTrue(list.Count == controlList.Count);
            }
        }

        [TestMethod]
        public static void TestSetResizing()
        {
            Random random = new Random(5);
            UnsafeBufferPool<int> pool = new UnsafeBufferPool<int>();
            QuickSet<int> set = new QuickSet<int>(pool, pool);
            HashSet<int> controlSet = new HashSet<int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    set.Add(iterationIndex);
                    controlSet.Add(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(set.Count);
                    var toRemove = set[indexToRemove];
                    set.FastRemove(toRemove);
                    controlSet.Remove(toRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    set.EnsureCapacity(set.Count * 3);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    set.Compact();
                }
            }

            Assert.IsTrue(set.Count == controlSet.Count);
            for (int i = 0; i < set.Count; ++i)
            {
                Assert.IsTrue(controlSet.Contains(set[i]));
            }
            foreach (var element in controlSet)
            {
                Assert.IsTrue(set.Contains(element));
            }
        }

        [TestMethod]
        public static void TestDictionaryResizing()
        {
            Random random = new Random(5);
            UnsafeBufferPool<int> pool = new UnsafeBufferPool<int>();
            QuickDictionary<int, int> dictionary = new QuickDictionary<int, int>(pool, pool, pool);
            Dictionary<int, int> controlDictionary = new Dictionary<int, int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    dictionary.Add(iterationIndex, iterationIndex);
                    controlDictionary.Add(iterationIndex, iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(dictionary.Count);
                    var toRemove = dictionary.Keys[indexToRemove];
                    dictionary.FastRemove(toRemove);
                    controlDictionary.Remove(toRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    dictionary.EnsureCapacity(dictionary.Count * 3);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    dictionary.Compact();
                }
            }

            Assert.IsTrue(dictionary.Count == controlDictionary.Count);
            for (int i = 0; i < dictionary.Count; ++i)
            {
                Assert.IsTrue(controlDictionary.ContainsKey(dictionary.Keys[i]));
            }
            foreach (var element in controlDictionary.Keys)
            {
                Assert.IsTrue(dictionary.ContainsKey(element));
            }
        }

        static void Main(string[] args)
        {
            //Could actually do a unit testing engine here. .. ... .... Or not.
            TestQueueResizing();
            TestListResizing();
            TestSetResizing();
            TestDictionaryResizing();
        }
    }
}
