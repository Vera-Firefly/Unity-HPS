# Unity-HPS

English | [简体中文](README.zh-CN.md)

Unity-HPS is a standalone Humanoid pose conversion library and command-line application. It converts local bone poses to body position, body rotation and muscle values, and decodes Humanoid poses back to local bone poses. The solver runs without Unity or VRCSDK.

## Features

- Bone pose → Humanoid RootT, RootQ and muscle values.
- Humanoid pose → local bone pose.
- Optional two-bone IK goals before pose conversion.
- Offline FBX skeleton import and Humanoid rig generation.
- Optional Avatar-specific calibration and reusable response profiles.
- Parallel frame processing, JSON input/output and a public C# SDK.

Humanoid conversion uses an approximate response model. Use a rig matching the target skeleton and its Humanoid settings. Avatar-specific calibration is available for custom configurations.

Unity-HPS processes skeletons and Humanoid poses. For PMX/VMD input and Unity `.anim` output, use VMD Converter Core with its Unity-HPS adapter.

## Requirements and distribution

The Windows x64 release runs without a Unity or .NET installation. Keep `UnityHps.Fbx.dll` beside `UnityHps.exe` when reading FBX files. The `managed/` directory contains the .NET 10 SDK; `unity/Editor/` contains optional Unity calibration tools.

## Quick start

Run these commands from the release directory:

```powershell
.\UnityHps.exe --help
.\UnityHps.exe rig --fbx avatar.fbx --output avatar-rig.json
.\UnityHps.exe solve --rig avatar-rig.json --input poses.json --output human-poses.json --threads 8
.\UnityHps.exe decode --rig avatar-rig.json --input human-poses.json --output decoded-poses.json
```

`solve` accepts bone frames in the rig's bone order. Positions are in metres, quaternions use `[x,y,z,w]`, and all data uses normalized Animator root space. Each output frame contains `humanPose.bodyPosition`, `bodyRotation` and `muscles`. `bodyPosition` is normalized by `rig.humanScale`.

See [the usage guide](docs/OFFLINE.md) for FBX mapping and calibration, and [the JSON protocol](docs/PROTOCOL.md) for input/output structures.

## Commands and options

Options use `--name value` pairs, including Boolean values such as `--ignore-meta true`.

| Command | Usage |
| --- | --- |
| `rig` | `--fbx avatar.fbx --output rig.json`; optional `--meta`, `--bone-map`, `--ignore-meta`, `--scale` (default `1`) and `--profile`. |
| `fbx-info` | `--fbx avatar.fbx --output skeleton.json`; optional `--meta`, `--bone-map` and `--ignore-meta`. |
| `solve` | `--rig rig.json --input poses.json --output result.json`; optional `--threads` (default `0`, automatic) and `--passes` (default `2`, range `1..20`). |
| `decode` | `--rig rig.json --input human-poses.json --output poses.json`. |
| `inspect` | `--rig rig.json`; prints skeleton and calibration information. |
| `calibrate` | `--rig rig.json --input training.json --output calibrated-rig.json`. |
| `profile` | `--rig calibrated-rig.json --output profile.json.gz`; optional `--mass-rig` supplies calibrated body-position weights. |
| `validate` | `--rig rig.json --input reference.json --output report.json`; accepts `--threads`, `--passes` and the limits below. |
| `demo` | `--output directory`; creates example rig and pose JSON files. |
| `self-test` | Runs the built-in checks. |
| `--help`, `-h`, `help` | Prints command usage. |

`validate` compares frames containing `reference` data. Limits are `--max-root-mm` (default `0.1` mm), `--max-root-deg` (default `0.1` degrees) and `--max-muscle` (default `0.001`). Exit codes: `0` success, `1` invalid input or runtime failure, `2` validation limits exceeded.

## Rig generation

`rig` combines an FBX skeleton with the built-in Humanoid response profile. It reads adjacent `.fbx.meta` mappings when available; otherwise it uses bone names. Use `--bone-map` for explicit mappings and `--ignore-meta true` to ignore adjacent metadata.

```powershell
.\UnityHps.exe rig --fbx avatar.fbx --ignore-meta true --bone-map bones.json --output rig.json
```

The mapping file is a JSON object from Humanoid names to FBX node names, for example `{"Hips":"pelvis","LeftUpperLeg":"thigh_l"}`. Required mappings must be complete and unique. Rebuild the rig after changing the skeleton, import scale or Humanoid configuration. Bone paths in the generated rig refer to the imported skeleton.

## Optional Unity calibration

Copy the provided `unity/Editor/` scripts into `Assets/Editor/` in a Unity project. Select a valid Humanoid model or scene GameObject, then choose **Tools → Unity HPS → Export selected Humanoid calibration**. Calibrate the exported rig:

```powershell
.\UnityHps.exe calibrate --rig rig.json --input training.json --output calibrated-rig.json
.\UnityHps.exe solve --rig calibrated-rig.json --input poses.json --output human-poses.json
```

Use the calibrated rig for subsequent conversions. Calibration requires Unity once; solving does not require it. Custom muscle limits and unsupported skeleton scaling may require this path.

## C# SDK

Reference `UnityHps.Core.dll` from a .NET 10 application. Keep `UnityHps.Fbx.dll` available when using FBX import.

```csharp
using UnityHps;

Rig rig = OfflineRigFactory.FromFbx("avatar.fbx", new FbxImportOptions());
var solver = new HumanoidSolver(rig);
PoseBatch input = HpsJson.ReadPoses("poses.json");
ResultBatch result = solver.SolveBatch(input, threads: 8, passes: 2);
HpsJson.Write("human-poses.json", result);
```

| API | Purpose |
| --- | --- |
| `HpsJson.ReadRig/ReadPoses/ReadHumanPoses/Write` | Rig, pose and result serialization. |
| `FbxReader.Read` | Import an FBX skeleton, mappings and morph bindings. |
| `OfflineRigFactory.FromFbx/FromSkeleton` | Generate an offline rig. |
| `OfflineRigFactory.LoadProfile/Transfer` | Load and adapt a response profile. |
| `HumanoidSolver.Solve/SolveBatch` | Convert one frame or a batch to Humanoid poses. |
| `HumanoidSolver.Decode` | Decode muscles and body transforms to a skeleton pose. |
| `HumanoidSolver.Compare` | Compare results with reference poses. |
| `RootCalibration.Fit` | Fit a rig using calibration poses. |
| `SkeletonPose.UpdateWorld`, `TwoBoneIk.Solve` | Update world transforms and solve limb IK. |

Do not modify a rig after constructing its solver. A solver can be shared for batch frame processing. Unity 2022.3 callers should use the EXE/JSON interface rather than loading .NET 10 assemblies into the Editor.

## Build

Install .NET 10 SDK, Visual Studio C++ desktop build tools and Windows SDK. From the repository root:

```powershell
.\scripts\Build.ps1
# Publish a self-contained build without NativeAOT:
.\scripts\Build.ps1 -NoAot
```

Both modes build the native FBX reader and require the C++ tools. The output is `artifacts/Unity-HPS-win-x64.zip`, containing the CLI, managed SDK, Unity integration scripts, documentation and licenses. GitHub Actions runs the same build and uploads the ZIP. Default response profiles are included; regular builds do not require Unity.

## License

Unity-HPS is licensed under [Apache-2.0](LICENSE). See [NOTICE](NOTICE) for attribution. The bundled ufbx reader is distributed under its MIT license; its full license is in `third_party/ufbx/LICENSE` in source distributions and `ufbx-LICENSE.txt` in binary distributions.
