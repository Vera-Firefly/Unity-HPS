using System.Reflection;

namespace UnityHps;

/// <summary>Display version of the loaded HPS SDK, including its prerelease label.</summary>
public static class HpsVersion
{
    public static string Display => typeof(HpsVersion).Assembly
        .GetCustomAttribute<AssemblyInformationalVersionAttribute>()!.InformationalVersion;
}
