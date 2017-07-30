using DemoContentLoader;
using Microsoft.Build.Evaluation;
using Microsoft.VisualStudio.Setup.Configuration;
using System;
using System.Collections;
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


        struct InstalledBuildTools
        {
            public string Version;
            public string Path;
        }

        unsafe static void CollectContentPaths(string projectPath, out string workingPath,
            out List<string> shaderPaths,
            out List<string> fontPaths)
        {
            //NOTE:
            //This project builder takes dependencies on a lot of things which really should not be depended upon.
            //It would be nice to figure out a more solid way to do it. Is there are better way to query the dotnet build tools?
            //A fallback option is to just create our own 'project'- just a little file with references alongside the csproj that triggered this build step.
            //We would have complete control over the format.



            using (var projectCollection = new ProjectCollection())
            {
                //This is very fragile. If the true dotnet path doesn't include dotnet, we won't find it and the build will fail.
                //For now, we'll bite the bullet on this and assume that people probably don't change the default dotnet path too often.
                var environmentVariables = Environment.GetEnvironmentVariables();
                string dotnetPath = null;
                var paths = ((string)environmentVariables["Path"]).Split(';');

                foreach (var candidatePath in paths)
                {
                    if (candidatePath.Contains("dotnet") && File.Exists(candidatePath + "dotnet.exe"))
                    {
                        dotnetPath = candidatePath;
                        break;
                    }
                }
                if (dotnetPath == null)
                {
                    throw new InvalidOperationException("No path in the Path environment variable includes dotnet, or the ones that do don't include the dotnet executable.");
                }
                //This is also fragile. We're relying on the file structure to remain the same, which isn't a good bet in the long term.
                var sdkPaths = new List<string>(Directory.EnumerateDirectories(dotnetPath + "sdk"));
                sdkPaths.Remove(dotnetPath + @"sdk\NuGetFallbackFolder");
                if (sdkPaths.Count == 0)
                {
                    throw new InvalidOperationException("No dotnet versions available.");
                }
                sdkPaths.Sort();
                var sdkPath = sdkPaths[sdkPaths.Count - 1];
                var toolsets = projectCollection.Toolsets;
                var buildProperties = new Dictionary<string, string>();
                //This, too, is fragile. Assumes that the required properties are the same and that they'll keep pointing in the same places in the folder structure.
                //Oh well, we'll probably have to fix this multiple times.
                buildProperties.Add("MSBuildExtensionsPath", sdkPath);
                buildProperties.Add("RoslynTargetsPath", sdkPath + @"\Roslyn");
                //And this is also fragile!
                const string toolsVersion = "15.0";
                var toolset = new Toolset(toolsVersion, sdkPath, buildProperties, projectCollection, null);

                projectCollection.AddToolset(toolset);
                projectCollection.DefaultToolsVersion = toolsVersion;
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
