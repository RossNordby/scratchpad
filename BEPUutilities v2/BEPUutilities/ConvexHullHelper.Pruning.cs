using System;
using System.Collections.Generic;
using BEPUutilities2.DataStructures;
using BEPUutilities2.ResourceManagement;
using System.Numerics;

namespace BEPUutilities2
{
    public static partial class ConvexHullHelper
    {
       
        /// <summary>
        /// Removes redundant points.  Two points are redundant if they occupy the same hash grid cell of size 0.001.
        /// </summary>
        /// <param name="points">List of points to prune.</param>
        public static void RemoveRedundantPoints(IList<Vector3> points)
        {
            RemoveRedundantPoints(points, .001);
        }

        /// <summary>
        /// Removes redundant points.  Two points are redundant if they occupy the same hash grid cell.
        /// </summary>
        /// <param name="points">List of points to prune.</param>
        /// <param name="cellSize">Size of cells to determine redundancy.</param>
        public static void RemoveRedundantPoints(IList<Vector3> points, double cellSize)
        {
            var rawPoints = new QuickList<Vector3>(BufferPools<Vector3>.Locking, BufferPool.GetPoolIndex(points.Count));
            rawPoints.AddRange(points);
            RemoveRedundantPoints(ref rawPoints, cellSize);
            points.Clear();
            for (int i = 0; i < rawPoints.Count; ++i)
            {
                points.Add(rawPoints.Elements[i]);
            }
            rawPoints.Dispose();
        }

        /// <summary>
        /// Removes redundant points.  Two points are redundant if they occupy the same hash grid cell of size 0.001.
        /// </summary>
        /// <param name="points">List of points to prune.</param>
        public static void RemoveRedundantPoints(ref QuickList<Vector3> points)
        {
            RemoveRedundantPoints(ref points, .001);
        }

        /// <summary>
        /// Removes redundant points.  Two points are redundant if they occupy the same hash grid cell.
        /// </summary>
        /// <param name="points">List of points to prune.</param>
        /// <param name="cellSize">Size of cells to determine redundancy.</param>
        public static void RemoveRedundantPoints(ref QuickList<Vector3> points, double cellSize)
        {
            var set = new QuickSet<Int3>(BufferPools<Int3>.Locking, BufferPools<int>.Locking, BufferPool.GetPoolIndex(points.Count));
            for (int i = points.Count - 1; i >= 0; --i)
            {
                var element = points.Elements[i];
                var cell = new Int3
                {
                    X = (int)Math.Floor(element.X / cellSize),
                    Y = (int)Math.Floor(element.Y / cellSize),
                    Z = (int)Math.Floor(element.Z / cellSize) 
                };
                if (set.Contains(cell))
                {
                    points.FastRemoveAt(i);
                }
                else
                {
                    set.Add(cell);
                    //TODO: Consider adding adjacent cells to guarantee that a point on the border between two cells will still detect the presence
                    //of a point on the opposite side of that border.
                }
            }
            set.Dispose();
        }

    }
}
