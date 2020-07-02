using SomeLibrary;
using System;
using Xunit;

namespace SomeTests
{
    public class UnitTest1
    {
        [Fact]
        public void Test1()
        {
            Assert.True(Tarben.Emulsify(5) == 15, "ya dingus");
        }
    }
}
