using DemoContentLoader;
using Microsoft.Build.Evaluation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;

namespace DemoContentBuilder
{
    static class ProjectBuilder
    {
        static string GetRelativePathFromDirectory(string path, string baseDirectory)
        {
            //(Borrowed this from a ye olde Marc Gravell post on SO.)
            var pathUri = new Uri(path);
            //Guarantee that the folder ends with a slash.
            if (!baseDirectory.EndsWith(Path.DirectorySeparatorChar.ToString()))
            {
                baseDirectory += Path.DirectorySeparatorChar;
            }
            var directoryUri = new Uri(baseDirectory);
            return Uri.UnescapeDataString(directoryUri.MakeRelativeUri(pathUri).ToString().Replace('/', Path.DirectorySeparatorChar));
        }


        static void CollectContentPaths(string projectPath, out string workingPath, out List<string> shaderPaths)
        {
            using (var projectCollection = new ProjectCollection())
            {
                var project = projectCollection.LoadProject(projectPath);
                workingPath = project.DirectoryPath + Path.DirectorySeparatorChar;

                var contentItems = project.GetItems("Content");
                shaderPaths = new List<string>();

                foreach (var item in contentItems)
                {
                    var extension = Path.GetExtension(item.EvaluatedInclude);
                    switch (extension)
                    {
                        case ".hlsl":
                            {
                                shaderPaths.Add(workingPath + item.EvaluatedInclude);
                            }
                            break;
                    }
                }
            }
        }

        public static void BuildShaders(string workingPath, string projectName, List<string> shaderPaths, bool debug, int optimizationLevel, out List<ShaderCompilationResult> warnings, out List<ShaderCompilationResult> errors)
        {

            var updatedCache = ShaderCompiler.Compile(workingPath, shaderPaths.ToArray(), workingPath + projectName + "ShaderCompilationCache.scc", out warnings, out errors, debug: debug, optimizationLevel: optimizationLevel);

            var prunedShaders = new Dictionary<SourceShader, byte[]>();
            foreach (var pathShaderPair in updatedCache.CompiledShaders)
            {
                //Prune out all of the extra path bits and save it.
                var relativePath = GetRelativePathFromDirectory(pathShaderPair.Key.Name, workingPath);
                prunedShaders.Add(new SourceShader { Name = relativePath, Defines = pathShaderPair.Key.Defines }, pathShaderPair.Value.Data);
            }

            const int retryCount = 10;
            const int retryDelay = 200;
            var runtimeCachePath = workingPath + projectName + "Shaders.sc";
            for (int i = 0; i < retryCount; ++i)
            {
                try
                {
                    using (var stream = File.OpenWrite(runtimeCachePath))
                    {
                        ShaderCache.Save(prunedShaders, workingPath, stream);
                    }
                    break;
                }
                catch (IOException e)
                {
                    Console.WriteLine($"Failed to write shader cache (attempt {i}): {e.Message}, retrying...");
                    Thread.Sleep(retryDelay);
                }
            }
        }


        public static void Main(string[] args)
        {
            bool debug = false;
            int optimizationLevel = 3;
            var targetPaths = new List<string>();
            for (int i = 0; i < args.Length; ++i)
            {
                //The argument should be either a compilation flag or a project path.
                if (args[i][0] == '-')
                {
                    switch (args[i])
                    {
                        case "-debug":
                            debug = true;
                            break;
                        case "-O0":
                            optimizationLevel = 0;
                            break;
                        case "-O1":
                            optimizationLevel = 1;
                            break;
                        case "-O2":
                            optimizationLevel = 2;
                            break;
                        case "-O3":
                            optimizationLevel = 3;
                            break;
                    }
                }
                else
                {
                    targetPaths.Add(args[i]);
                }
            }
            foreach (var targetPath in targetPaths)
            {
                CollectContentPaths(targetPath, out var workingPath, out var shaderPaths);

                BuildShaders(workingPath, Path.GetFileNameWithoutExtension(targetPath), shaderPaths, debug, debug ? 0 : optimizationLevel, out var warnings, out var errors);
                foreach (var error in errors)
                {
                    Console.WriteLine($"{error.File} ({error.LineNumber},{error.ColumnNumber}): error {error.Code}: {error.Message}");
                }
                foreach (var warning in warnings)
                {
                    Console.WriteLine($"{warning.File} ({warning.LineNumber},{warning.ColumnNumber}): warning {warning.Code}: {warning.Message}");
                }

            }
        }
    }
}
