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
        public static string GetRelativePathFromDirectory(string path, string baseDirectory)
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


        static void CollectContentPaths(string projectPath, out string workingPath,
            out List<string> shaderPaths,
            out List<string> fontPaths)
        {
            using (var projectCollection = new ProjectCollection())
            {
                //TODO: Watch out, this takes a dependency on a specific version of the ms build tools, requiring a user to have it installed.
                //Would be nice to have a better way to do this. Preferably that doesn't rely on registry, since a registry issue is why this got added in the first place.
                //Maybe a modification to the projects to specify a wildcarded version or something.
                projectCollection.DefaultToolsVersion = "14.0";
                var project = projectCollection.LoadProject(projectPath);
                workingPath = project.DirectoryPath + Path.DirectorySeparatorChar;

                var contentItems = project.GetItems("Content");
                shaderPaths = new List<string>();
                fontPaths = new List<string>();

                foreach (var item in contentItems)
                {
                    var extension = Path.GetExtension(item.EvaluatedInclude);
                    var path = workingPath + item.EvaluatedInclude;
                    switch (extension)
                    {
                        case ".hlsl":
                            shaderPaths.Add(path);
                            break;
                        case ".ttf":
                        case ".otf":
                            fontPaths.Add(path);
                            break;
                    }
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
                CollectContentPaths(targetPath, out var workingPath, out var shaderPaths, out var fontPaths);
                var cachePathStart = workingPath + Path.GetFileNameWithoutExtension(targetPath);
                //Shaders are stored a little differently than the rest of content. This is partially for legacy reasons.
                //You could make the argument for bundling them together, but shaders do have some unique macro and dependency management that other kinds of content lack.

                ShaderCompiler.Compile(workingPath,
                    cachePathStart + "ShaderCompilationCache.scc",
                    cachePathStart + "Shaders.sc", shaderPaths.ToArray(), out var shaderWarnings, out var shaderErrors, debug: debug, optimizationLevel: optimizationLevel);
                foreach (var error in shaderErrors)
                {
                    Console.WriteLine($"{error.File} ({error.LineNumber},{error.ColumnNumber}): error {error.Code}: {error.Message}");
                }
                foreach (var warning in shaderWarnings)
                {
                    Console.WriteLine($"{warning.File} ({warning.LineNumber},{warning.ColumnNumber}): warning {warning.Code}: {warning.Message}");
                }
                ContentBuilder.BuildContent(workingPath,
                    cachePathStart + "ContentBuildCache.cbc",
                    cachePathStart + "Content.ca", fontPaths, out var contentWarnings, out var contentErrors);
                foreach (var error in contentErrors)
                {
                    Console.WriteLine($"{error.File}: error: {error.Message}");
                }
                foreach (var warning in contentWarnings)
                {
                    Console.WriteLine($"{warning.File}: warning: {warning.Message}");
                }
            }
        }
    }
}
