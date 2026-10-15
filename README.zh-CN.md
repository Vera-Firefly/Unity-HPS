# Unity-HPS

[English](README.md) | 简体中文

Unity-HPS 是独立运行的 Humanoid 姿态转换库与命令行程序，用于将骨骼局部姿态转换为身体位置、身体旋转和肌肉值，并将 Humanoid 姿态还原为骨骼局部姿态。求解过程无需 Unity 或 VRCSDK。

## 功能

- 骨骼姿态转换为 Humanoid RootT、RootQ 和肌肉值。
- Humanoid 姿态还原为骨骼局部姿态。
- 在姿态转换前应用可选的双骨骼 IK 目标。
- 离线读取 FBX 骨架并生成 Humanoid rig。
- 支持模型专用校准和可复用的肌肉响应模板。
- 提供并行帧处理、JSON 输入输出和公开 C# SDK。

Humanoid 转换采用近似响应模型。使用时应选择与目标骨架及 Humanoid 设置一致的 rig；自定义配置可使用模型专用校准。

Unity-HPS 处理骨架和 Humanoid 姿态。若需读取 PMX/VMD 并生成 Unity `.anim`，请使用 VMD Converter Core 及其 Unity-HPS 适配器。

## 运行要求与发布内容

Windows x64 发布包无需安装 Unity 或 .NET。读取 FBX 时，须将 `UnityHps.Fbx.dll` 保留在 `UnityHps.exe` 同目录。`managed/` 包含 .NET 10 SDK，`unity/Editor/` 包含可选的 Unity 校准工具。

## 快速开始

在发布目录执行：

```powershell
.\UnityHps.exe --help
.\UnityHps.exe rig --fbx avatar.fbx --output avatar-rig.json
.\UnityHps.exe solve --rig avatar-rig.json --input poses.json --output human-poses.json --threads 8
.\UnityHps.exe decode --rig avatar-rig.json --input human-poses.json --output decoded-poses.json
```

`solve` 的输入骨骼顺序须与 rig 一致。位置单位为米，四元数顺序为 `[x,y,z,w]`，数据位于归一化 Animator 根节点空间。输出帧包含 `humanPose.bodyPosition`、`bodyRotation` 和 `muscles`；`bodyPosition` 按 `rig.humanScale` 归一化。

FBX 映射与校准步骤见[使用指南](docs/OFFLINE.md)，输入输出结构见 [JSON 数据协议](docs/PROTOCOL.md)。

## 命令与参数

参数采用 `--名称 值` 格式，布尔参数同样需要值，例如 `--ignore-meta true`。

| 命令 | 用法 |
| --- | --- |
| `rig` | `--fbx avatar.fbx --output rig.json`；可选 `--meta`、`--bone-map`、`--ignore-meta`、`--scale`（默认 `1`）、`--profile`。 |
| `fbx-info` | `--fbx avatar.fbx --output skeleton.json`；可选 `--meta`、`--bone-map`、`--ignore-meta`。 |
| `solve` | `--rig rig.json --input poses.json --output result.json`；可选 `--threads`（默认 `0`，自动选择）、`--passes`（默认 `2`，范围 `1..20`）。 |
| `decode` | `--rig rig.json --input human-poses.json --output poses.json`。 |
| `inspect` | `--rig rig.json`；显示骨架与校准信息。 |
| `calibrate` | `--rig rig.json --input training.json --output calibrated-rig.json`。 |
| `profile` | `--rig calibrated-rig.json --output profile.json.gz`；可选 `--mass-rig` 提供已校准的身体位置权重。 |
| `validate` | `--rig rig.json --input reference.json --output report.json`；支持 `--threads`、`--passes` 和下述误差限值。 |
| `demo` | `--output directory`；生成 rig 和姿态 JSON 示例。 |
| `self-test` | 执行内置检查。 |
| `--help`、`-h`、`help` | 显示命令帮助。 |

`validate` 比较包含 `reference` 数据的输入帧。限值参数为 `--max-root-mm`（默认 `0.1` 毫米）、`--max-root-deg`（默认 `0.1` 度）、`--max-muscle`（默认 `0.001`）。退出码：`0` 成功，`1` 输入或运行错误，`2` 超出比较限值。

## rig 生成

`rig` 将 FBX 骨架与内置 Humanoid 响应模板结合。有相邻 `.fbx.meta` 时读取其中的骨骼映射，否则按骨骼名称识别。可用 `--bone-map` 指定映射，并用 `--ignore-meta true` 忽略相邻元数据。

```powershell
.\UnityHps.exe rig --fbx avatar.fbx --ignore-meta true --bone-map bones.json --output rig.json
```

映射文件是 Humanoid 骨名称到 FBX 节点名的 JSON 对象，例如 `{"Hips":"pelvis","LeftUpperLeg":"thigh_l"}`。必需骨骼映射须完整且唯一。修改骨架、导入比例或 Humanoid 设置后应重新生成 rig；rig 中的骨骼路径对应导入骨架。

## 可选的 Unity 校准

将提供的 `unity/Editor/` 脚本复制到 Unity 工程的 `Assets/Editor/`。选择有效的 Humanoid 模型或场景 GameObject，执行 **Tools → Unity HPS → Export selected Humanoid calibration**，然后校准导出的 rig：

```powershell
.\UnityHps.exe calibrate --rig rig.json --input training.json --output calibrated-rig.json
.\UnityHps.exe solve --rig calibrated-rig.json --input poses.json --output human-poses.json
```

后续转换使用校准后的 rig。校准阶段需要 Unity，求解阶段无需 Unity。自定义肌肉限制或不受离线导入支持的骨架缩放可能需要此方式。

## C# SDK

在 .NET 10 程序中引用 `UnityHps.Core.dll`。调用 FBX 导入时，还须提供 `UnityHps.Fbx.dll`。

```csharp
using UnityHps;

Rig rig = OfflineRigFactory.FromFbx("avatar.fbx", new FbxImportOptions());
var solver = new HumanoidSolver(rig);
PoseBatch input = HpsJson.ReadPoses("poses.json");
ResultBatch result = solver.SolveBatch(input, threads: 8, passes: 2);
HpsJson.Write("human-poses.json", result);
```

| 接口 | 用途 |
| --- | --- |
| `HpsJson.ReadRig/ReadPoses/ReadHumanPoses/Write` | rig、姿态和结果序列化。 |
| `FbxReader.Read` | 读取 FBX 骨架、映射和形态键绑定。 |
| `OfflineRigFactory.FromFbx/FromSkeleton` | 生成离线 rig。 |
| `OfflineRigFactory.LoadProfile/Transfer` | 加载并适配响应模板。 |
| `HumanoidSolver.Solve/SolveBatch` | 转换单帧或批量 Humanoid 姿态。 |
| `HumanoidSolver.Decode` | 将肌肉值和身体变换还原为骨骼姿态。 |
| `HumanoidSolver.Compare` | 与参考姿态比较。 |
| `RootCalibration.Fit` | 用校准姿态拟合 rig。 |
| `SkeletonPose.UpdateWorld`、`TwoBoneIk.Solve` | 更新世界变换、求解肢体 IK。 |

创建 solver 后不要修改其 rig。solver 可供批量帧处理共享。Unity 2022.3 调用方应使用 EXE/JSON 接口，不应在 Editor 中直接加载 .NET 10 程序集。

## 构建

安装 .NET 10 SDK、Visual Studio C++ 桌面开发工具和 Windows SDK，在仓库根目录执行：

```powershell
.\scripts\Build.ps1
# 不使用 NativeAOT，生成自包含版本：
.\scripts\Build.ps1 -NoAot
```

两种方式均会构建原生 FBX 读取器，需要 C++ 工具。产物为 `artifacts/Unity-HPS-win-x64.zip`，包含 CLI、托管 SDK、Unity 集成脚本、文档和许可证。GitHub Actions 执行相同构建并上传 ZIP。默认响应模板已包含在仓库中，常规构建无需 Unity。

## 开源协议

Unity-HPS 使用 [Apache-2.0](LICENSE)，署名信息见 [NOTICE](NOTICE)。随附的 ufbx 读取器按其 MIT 许可分发；完整许可位于源码包的 `third_party/ufbx/LICENSE` 和二进制发布包的 `ufbx-LICENSE.txt`。
