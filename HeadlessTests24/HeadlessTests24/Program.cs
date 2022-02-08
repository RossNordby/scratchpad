using HeadlessTests24.DemoStyle;
using HeadlessTests24.DemoStyle.Dancers;
using System.Numerics;
using System.Text;

List<int> threadCounts = new List<int>();
const string threadCountsPath = "threadCounts.txt";
try
{
    using (var reader = new StreamReader(File.Open(threadCountsPath, FileMode.Open)))
    {
        if (int.TryParse(reader.ReadLine(), out var count))
        {
            if (count < 0)
                Console.WriteLine($"(no we're not going to use a negative thread count of {count})");
            if (count > 2048)
                Console.WriteLine($"(no we're not going to use a massive thread count of {count})");
            threadCounts.Add(count);
        }
    }
    Console.Write($"Found {threadCountsPath}; using thread counts of: ");
}
catch
{
    var threadCount = Environment.ProcessorCount;
    //These are reasonable search locations for a pow2 processor.... which isn't every processor.
    while (threadCount > 0)
    {
        threadCounts.Add(threadCount);
        threadCount >>= 1;
    }
    if (Environment.ProcessorCount >= 4)
    {
        threadCounts.Add(Environment.ProcessorCount * 3 / 4);
    }
    if (Environment.ProcessorCount >= 8)
    {
        threadCounts.Add(Environment.ProcessorCount * 3 / 8);
    }
    Console.Write($"No {threadCountsPath} detected; defaulting to thread counts of: ");
}
for (int i = 0; i < threadCounts.Count; ++i)
{
    Console.Write(i < threadCounts.Count - 1 ? $"{threadCounts[i]}, " : threadCounts[i]);
}
Console.WriteLine();

var builder = new StringBuilder(2048);



builder.Append("Demo");
for (int i = 0; i < threadCounts.Count; ++i)
{
    builder.Append(" ").Append(threadCounts[i]);
}
builder.AppendLine();
void ExecuteDemoStyle<T>(int runCount, int preframeCount, int frameCount) where T : Demo, new()
{
    string name = typeof(T).Name;
    builder.Append(name).Append(" ");
    for (int i = 0; i < threadCounts.Count; ++i)
    {
        Console.WriteLine($"@@ Testing {name} with {threadCounts[i]} threads. @@");
        var times = new List<double>();
        DemoHeadlessTest.Test<ColosseumDemo>(threadCounts[i], runCount, preframeCount, frameCount, times);
        //Just write the minimum for now.
        double minimum = times[0];
        for (int j = 1; j < times.Count; ++j)
        {
            minimum = Math.Min(minimum, times[j]);
        }
        builder.Append(minimum);
    }
    builder.AppendLine();
}

ExecuteDemoStyle<ColosseumDemo>(2, 32, 256);
ExecuteDemoStyle<DancerDemo>(2, 32, 256);

Console.WriteLine(builder.ToString());