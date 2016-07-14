using BEPUphysics.BroadPhaseEntries;
using SIMDPrototyping.Trees.SingleArray;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Vector3B = BEPUutilities.Vector3;

namespace SIMDPrototyping.Trees.Tests
{
    public static class ReconstructPerformanceTest
    {
        delegate void VertexModifier(Random random);
        delegate void MeshRebuilder(int iterationIndex);
        delegate void MeshDisposer();
        delegate float CostMetricMeasurer();

        static void Test(VertexModifier vertexModifier, MeshRebuilder rebuilder, CostMetricMeasurer costMeasurer, string testName, int iterations)
        {
            Random random = new Random(5);
            double sum = 0;
            double fastest = double.MaxValue;
            double slowest = 0;
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                vertexModifier(random);

                var start = Stopwatch.GetTimestamp();
                rebuilder(iterationIndex);
                var end = Stopwatch.GetTimestamp();

                var time = (end - start) / (double)Stopwatch.Frequency;

                if (time < fastest) fastest = time;
                if (time > slowest) slowest = time;

                sum += time;
            }
            Console.WriteLine($"{testName}:");
            Console.WriteLine($"Average: {sum / iterations}");
            Console.WriteLine($"Fastest: {fastest}");
            Console.WriteLine($"Slowest: {slowest}");
            if (costMeasurer != null)
                Console.WriteLine($"Cost Metric: {costMeasurer()}");
        }



        static void Build14(int gridSize, out VertexModifier vertexModifier, out Dictionary<string, MeshRebuilder> rebuilders)
        {
            List<Vector3B> vertices = new List<Vector3B>();

            for (int y = 0; y < gridSize; ++y)
                for (int x = 0; x < gridSize; ++x)
                    vertices.Add(new Vector3B(2 * x, 0, 2 * y));

            List<int> triangles = new List<int>();

            for (int y = 0; y < gridSize - 1; ++y)
                for (int x = 0; x < gridSize - 1; ++x)
                {
                    triangles.Add(gridSize * (y) + (x));
                    triangles.Add(gridSize * (y) + (x + 1));
                    triangles.Add(gridSize * (y + 1) + (x + 1));

                    triangles.Add(gridSize * (y) + (x));
                    triangles.Add(gridSize * (y + 1) + (x + 1));
                    triangles.Add(gridSize * (y + 1) + (x));
                }

            var physicsMesh = new BEPUphysics.BroadPhaseEntries.StaticMesh(vertices.ToArray(), triangles.ToArray(), BEPUutilities.AffineTransform.Identity);

            vertexModifier = (Random random) =>
            {
                for (int vertexIndex = 0; vertexIndex < physicsMesh.Mesh.Data.Vertices.Length; ++vertexIndex)
                    physicsMesh.Mesh.Data.Vertices[vertexIndex].Y = (float)random.NextDouble() * 2;
            };

            rebuilders = new Dictionary<string, MeshRebuilder>();
            rebuilders.Add("v1.4.0 Reconstruct", (i) =>
            {
                physicsMesh.Mesh.Tree.Reconstruct();
                physicsMesh.UpdateBoundingBox();
            });
            rebuilders.Add("v1.4.0 Refit", (i) =>
            {
                physicsMesh.Mesh.Tree.Refit();
                physicsMesh.UpdateBoundingBox();
            });

        }



        class Mesh
        {
            public Tree Tree;

            public Vector3[] Vertices;
            int[] indices;

            //Under normal circumstances this would be redundant data, but we intentionally want to redo the initial construction.
            //(If this becomes an important use case, ideally there would be direct API support for it, bypassing the need for maintaining these arrays.)
            BoundingBox[] leafBounds;
            int[] leafIds;

            void GetBoundingBox(int triangleIndex, out BoundingBox box)
            {
                var baseIndex = triangleIndex * 3;
                var a = Vertices[indices[baseIndex]];
                var b = Vertices[indices[baseIndex + 1]];
                var c = Vertices[indices[baseIndex + 2]];
                box.Min = Vector3.Min(Vector3.Min(a, b), c);
                box.Max = Vector3.Max(Vector3.Max(a, b), c);
            }

            public Mesh(Vector3[] vertices, int[] indices)
            {
                this.Vertices = vertices;
                this.indices = indices;
                var leafCount = indices.Length / 3;
                leafBounds = new BoundingBox[leafCount];
                leafIds = new int[leafCount];
                Tree = new Tree(leafCount);
                for (int i = 0; i < leafCount; ++i)
                {
                    leafIds[i] = i;
                    GetBoundingBox(i, out leafBounds[i]);
                }

                Tree.SweepBuild(leafIds, leafBounds);
            }

            public void RebuildSweep()
            {
                for (int i = 0; i < Tree.LeafCount; ++i)
                {
                    GetBoundingBox(i, out leafBounds[i]);
                }
                //SweepBuild expects the tree to be fresh.
                Tree.Reset();
                Tree.SweepBuild(leafIds, leafBounds);
            }
            public void RebuildIncremental()
            {
                for (int i = 0; i < Tree.LeafCount; ++i)
                {
                    GetBoundingBox(i, out leafBounds[i]);
                }
                Tree.Reset();
                for (int i = 0; i < leafBounds.Length; ++i)
                {
                    //Roughly permute the leaves for a random-ish insertion order. Improves quality.
                    //Could do a better permutation.
                    var leafIndex = (int)((982451653L * i) % leafBounds.Length);
                    Tree.Add(leafIds[leafIndex], ref leafBounds[leafIndex]);
                }
            }

            public void Refit()
            {
                Tree.Refit();
            }

            public void RefitWithIncrementalRefine(int frameIndex)
            {
                Tree.RefitAndRefine(frameIndex);
            }

        }

        static void Build2(int gridSize, out VertexModifier vertexModifier, out Dictionary<string, MeshRebuilder> rebuilders, out CostMetricMeasurer costMeasurer, out MeshDisposer disposer)
        {
            List<Vector3> vertices = new List<Vector3>();

            for (int y = 0; y < gridSize; ++y)
                for (int x = 0; x < gridSize; ++x)
                    vertices.Add(new Vector3(2 * x, 0, 2 * y));

            List<int> triangles = new List<int>();

            for (int y = 0; y < gridSize - 1; ++y)
                for (int x = 0; x < gridSize - 1; ++x)
                {
                    triangles.Add(gridSize * (y) + (x));
                    triangles.Add(gridSize * (y) + (x + 1));
                    triangles.Add(gridSize * (y + 1) + (x + 1));

                    triangles.Add(gridSize * (y) + (x));
                    triangles.Add(gridSize * (y + 1) + (x + 1));
                    triangles.Add(gridSize * (y + 1) + (x));
                }

            var physicsMesh = new Mesh(vertices.ToArray(), triangles.ToArray());

            vertexModifier = (Random random) =>
            {
                for (int vertexIndex = 0; vertexIndex < physicsMesh.Vertices.Length; ++vertexIndex)
                    physicsMesh.Vertices[vertexIndex].Y = (float)random.NextDouble() * 2;
            };

            rebuilders = new Dictionary<string, MeshRebuilder>();
            //Note incremental first, sweep second. Starts the refine off with a better tree to begin with so it doesn't to do an unrealistic amount of optimization to begin with.
            rebuilders.Add("v2 Reconstruct Incremental", (i) => physicsMesh.RebuildIncremental());
            rebuilders.Add("v2 Reconstruct Sweep", (i) => physicsMesh.RebuildSweep());
            rebuilders.Add("v2 Refit And Refine", (i) => physicsMesh.RefitWithIncrementalRefine(i));
            rebuilders.Add("v2 Refit", (i) => physicsMesh.Refit());

            disposer = () => { physicsMesh.Tree.Dispose(); };

            costMeasurer = () => { return physicsMesh.Tree.MeasureCostMetric(); };
        }

        public static void Test()
        {
            const int gridSize = 256;
            const int iterationCount = 100;
            VertexModifier modifier;
            Dictionary<string, MeshRebuilder> rebuilders;


            Build14(gridSize, out modifier, out rebuilders);
            foreach (var rebuilder in rebuilders)
            {
                Test(modifier, rebuilder.Value, null, rebuilder.Key + " Warmup", 1);
            }
            foreach (var rebuilder in rebuilders)
            {
                GC.Collect();
                Test(modifier, rebuilder.Value, null, rebuilder.Key, iterationCount);
            }

            MeshDisposer disposer;
            CostMetricMeasurer costMeasurer;
            Build2(gridSize, out modifier, out rebuilders, out costMeasurer, out disposer);
            foreach (var rebuilder in rebuilders)
            {
                Test(modifier, rebuilder.Value, costMeasurer, rebuilder.Key + " Warmup", 1);
            }
            foreach (var rebuilder in rebuilders)
            {
                GC.Collect();
                Test(modifier, rebuilder.Value, costMeasurer, rebuilder.Key, iterationCount);
            }
            disposer();
        }
    }
}
