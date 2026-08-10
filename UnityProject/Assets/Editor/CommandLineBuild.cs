using System;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;

public static class CommandLineBuild
{
    public static void BuildAndroid()
    {
        var args = Environment.GetCommandLineArgs();
        var pathArgument = Array.IndexOf(args, "-metaQuestApkPath");
        var outputPath = pathArgument >= 0 && pathArgument + 1 < args.Length
            ? args[pathArgument + 1]
            : Environment.GetEnvironmentVariable("METAQUEST_APK_PATH");
        if (string.IsNullOrWhiteSpace(outputPath))
            outputPath = "app.apk";

        var scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();

        var report = BuildPipeline.BuildPlayer(
            scenes,
            outputPath,
            BuildTarget.Android,
            BuildOptions.None);

        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception($"Android build failed: {report.summary.result}");
    }
}
