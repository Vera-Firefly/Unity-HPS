# Unity-HPS 使用指南

## FBX 骨架与 rig

```powershell
.\UnityHps.exe rig --fbx avatar.fbx --output avatar-rig.json
.\UnityHps.exe fbx-info --fbx avatar.fbx --output skeleton.json
```

`rig` 生成可供求解器使用的 Humanoid rig；`fbx-info` 输出骨架、人体映射和形态键绑定。读取 FBX 须提供同目录的 `UnityHps.Fbx.dll`。

默认读取相邻 `.fbx.meta` 中的人体映射与参考姿态；没有元数据时按名称识别。显式指定元数据或骨骼映射：

```powershell
.\UnityHps.exe rig --fbx avatar.fbx --meta avatar.fbx.meta --output rig.json
.\UnityHps.exe rig --fbx avatar.fbx --ignore-meta true --bone-map bones.json --output rig.json
```

`bones.json` 为 Humanoid 名称到 FBX 节点名的映射，例如 `{"Hips":"pelvis","LeftUpperLeg":"thigh_l"}`。必需骨骼须完整且唯一。`--scale` 对米制骨架额外缩放，默认 `1`；`--profile` 可指定 JSON 或 JSON.gz 响应模板。

内置模板支持有、无 UpperChest 的骨架。自定义肌肉限制或人体链上的非均匀缩放可能需要专用校准。修改骨架、导入比例或人体设置后，须重新生成 rig。

## 姿态转换

```powershell
.\UnityHps.exe solve --rig rig.json --input poses.json --output human-poses.json --threads 8 --passes 2
.\UnityHps.exe decode --rig rig.json --input human-poses.json --output decoded-poses.json
```

输入的骨骼顺序与局部父子关系须和 rig 一致，位置单位为米，四元数为 xyzw。所有数据采用归一化 Animator 根节点空间。`--threads 0` 自动选择线程数；`--passes` 控制拟合轮数，范围 `1..20`。

`Rig.bindBones` 表示导入参考姿态，`Rig.bones` 表示肌肉中立姿态。`solve` 输出可直接作为 `decode` 的输入。完整结构见 [JSON 数据协议](PROTOCOL.md)。

## 模型专用校准

将发布包中的 `unity/Editor/` 脚本放入 Unity 工程的 `Assets/Editor/`。选择有效的 Humanoid GameObject、FBX 或预制件，执行 **Tools → Unity HPS → Export selected Humanoid calibration**，然后：

```powershell
.\UnityHps.exe calibrate --rig rig.json --input training.json --output calibrated-rig.json
```

后续 `solve` 和 `decode` 使用 `calibrated-rig.json`。仅数据采集需要 Unity，运行求解器无需 Unity。更改模型的 Humanoid 配置后应重新校准。

## 响应模板

将已校准 rig 导出为可复用模板：

```powershell
.\UnityHps.exe profile --rig calibrated-rig.json --output profile.json.gz
```

生成 rig 时以 `--profile profile.json.gz` 使用。`--mass-rig` 可单独提供身体位置权重来源。默认模板随构建内嵌，无需用户重新生成；维护者可用 `scripts/Build-Profiles.ps1 -Unity <Unity.exe路径>` 重建内置模板。

## 结果比较

```powershell
.\UnityHps.exe validate --rig calibrated-rig.json --input reference.json --output report.json --max-root-mm 0.1 --max-root-deg 0.1 --max-muscle 0.001
```

输入帧须包含 `reference` 人体姿态。输出报告包含身体位置、身体旋转和肌肉差值；超出指定限值返回退出码 `2`。限值不改变求解精度。

## 与 VMD Converter Core 配合

VMDCC 负责 PMX/VMD 解析、PMX IK、曲线生成及 `.anim` 输出；Unity-HPS 提供 Humanoid 求解和离线 FBX rig。将 `VmdConverterCore.Hps.dll`、`UnityHps.Core.dll` 放在 VMDCC EXE 同目录，读取 FBX 时另需 `UnityHps.Fbx.dll`。

```powershell
.\VmdConverterCore.exe --pmx model.pmx --vmd motion.vmd --output motion.anim --anim-w
.\VmdConverterCore.exe --pmx model.pmx --fbx avatar.fbx --output-kind target-humanoid --vmd motion.vmd --output motion-target.anim --anim-w
.\VmdConverterCore.exe --pmx model.pmx --fbx avatar.fbx --output-kind target-fk --vmd motion.vmd --output motion-fk.anim --anim-w
.\VmdConverterCore.exe --pmx model.pmx --hps-rig calibrated-rig.json --output-kind target-humanoid --vmd motion.vmd --output motion.anim --anim-w
```

VMDCC 仅在传入 `--anim-w` 时查找 Unity-HPS；缺少 HPS 时使用 VMDCC 原求解器，并要求原格式 `--rig`。`--rig` 和 `--hps-rig` 不能互换。`humanoid-ik` 使用 PMX 源人体骨架；`target-humanoid` 和 `target-fk` 使用目标 FBX/HPS rig，未指定目标时使用 PMX 人体代理骨架。PMX rig 生成由 VMDCC 提供。

多个动作可重复 `--vmd` 并配合 `--output-dir`，或传入 `--jobs` 清单。全部参数见 VMDCC 文档及 `VmdConverterCore.exe --help`。
