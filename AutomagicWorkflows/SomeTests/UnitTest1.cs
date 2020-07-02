using SomeLibrary;
using System;
using Xunit;
using Xunit.Abstractions;

namespace SomeTests
{
    public class UnitTest1
    {
        private readonly ITestOutputHelper output;
        public UnitTest1(ITestOutputHelper output)
        {
            this.output = output;
        }

        [Fact]
        public void Test1()
        {
            Assert.True(Tarben.Emulsify(5) == 15, "ya dingus");
        }

        [Fact]
        public void Test2()
        {
            output.WriteLine($"count: {Environment.ProcessorCount}");
            Assert.True(Environment.ProcessorCount > 1);
        }
    }
}
