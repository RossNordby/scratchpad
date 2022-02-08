using System.Numerics;

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
    if (threadCount >= 4)
    {
        threadCounts.Add(Environment.ProcessorCount * 3 / 4);
    }
    if (threadCount >= 8)
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