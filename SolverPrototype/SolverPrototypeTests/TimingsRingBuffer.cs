using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using DemoRenderer.UI;
using System;

namespace SolverPrototypeTests
{
    public class TimingsRingBuffer : IDataSeries
    {
        QuickQueue<double, Array<double>> queue;

        /// <summary>
        /// Gets or sets the maximum number of time measurements that can be held by the ring buffer.
        /// </summary>
        public int Capacity
        {
            get { return queue.Span.Length; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Capacity must be positive.");
                if (Capacity != value)
                {
                    var newSpan = new Array<double>(new double[value]);
                    queue.Resize(ref newSpan, out var oldSpan);
                }
            }
        }
        public TimingsRingBuffer(int maximumCapacity)
        {
            if(maximumCapacity <= 0)
                throw new ArgumentException("Capacity must be positive.");
            var initialSpan = new Array<double>(new double[maximumCapacity]);
            queue = new QuickQueue<double, Array<double>>(ref initialSpan);
        }

        public void Add(double time)
        {

            if(queue.Count == Capacity)
            {
                queue.Dequeue();
            }
            queue.EnqueueUnsafely(time);

        }


        public double this[int index] => queue[index];

        public int Start => 0;

        public int End => queue.Count;

        public TimelineStats ComputeStats()
        {
            TimelineStats stats;
            stats.Total = 0.0;
            var sumOfSquares = 0.0;
            stats.Min = double.MaxValue;
            stats.Max = double.MinValue;
            for (int i = 0; i < queue.Count; ++i)
            {
                var time = queue[i];
                stats.Total += time;
                sumOfSquares += time * time;
                if (time < stats.Min)
                    stats.Min = time;
                if (time > stats.Max)
                    stats.Max = time;
            }
            stats.Average = stats.Total / queue.Count;
            stats.StdDev = Math.Sqrt(Math.Max(0, sumOfSquares / queue.Count - stats.Average * stats.Average));
            return stats;
        }
    }
}
