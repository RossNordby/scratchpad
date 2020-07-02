using SomeLibrary;
using System;
using Xunit;
using Xunit.Abstractions;

namespace SomeTests
{
    public class UnitTest1
    {
        [Fact]
        public void Test1()
        {
            Assert.True(Tarben.Emulsify(5) == 15, "ya dingus");
        }

        [Fact]
        public void Test2()
        {
            Assert.True(Environment.ProcessorCount > 1);
            Assert.True(Environment.ProcessorCount > 40000, $"count: {Environment.ProcessorCount}");
        }
    }
}
