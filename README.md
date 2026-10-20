# AdaptorPhysX

Unity 开源体素柔体插件，支持动态切割与半切割，面向虚拟手术等医学仿真场景。
实现基于 Burst + Jobs（CPU 多线程）；Compute Shader 后端在路线图中。

![标准场景总览](docs/images/99_overview.png)

| 全切后拉开 | 预应力皮瓣半切 | 镊子牵拉切口 | 楔形切除 |
|---|---|---|---|
| ![S1](docs/images/S1b_block_pulled.png) | ![S2](docs/images/S2_flap_incision.png) | ![S4](docs/images/S4_flap_retraction_close.png) | ![S5](docs/images/S5b_liver_wedge_lifted.png) |

- 体积表示：稀疏六面体 lattice，每个体素按 Kuhn 剖分拆为 6 个四面体。
- 材料模型：Stable Neo-Hookean XPBD（Macklin & Müller 2021），参数为杨氏模量与泊松比。
- 时间积分：small steps（每帧多个子步，每个子步 1 次迭代），按体素列 4 色并行，结果可逐位复现。
- 切割：刀刃扫掠面与 lattice 棱求交后断开棱，按 virtual node algorithm 复制体素副本。
  全切、半切（切口终止在组织内部）、交叉切与对已分离部分的再次切割使用同一套机制。
- 渲染：原始表面网格以三线性插值嵌入体素副本，并沿切平面拆分；切面由断开棱上的交点与法线生成，
  按原始表面的有符号距离场裁剪到表面以内。切面着色器使用静止坐标的 3D 噪声，不需要 UV。
- 接触：Unity Sphere/Capsule/Box 碰撞体（含摩擦，子步内插值位姿）；切下的块之间、折叠的组织之间的接触。
- 交互：手术刀（`ApxScalpel`）、钉住区域、镊子抓取（不跨越切口）。

## 环境

- Unity 2022.3 LTS（内置渲染管线）
- 依赖包：Burst 1.8、Collections 1.4、Mathematics 1.2、Test Framework 1.1
- `com.coplaydev.unity-mcp`：编辑器 MCP 服务（开发用，可从 `Packages/manifest.json` 删除）

## 快速开始

1. 用 Unity Hub 打开仓库根目录。
2. 打开 `Assets/Samples/SurgeryDemo/SurgeryDemo.unity`（或菜单 `AdaptorPhysX/Build Surgery Demo Scene` 重新生成）。
3. 运行场景。操作：

| 输入 | 功能 |
|---|---|
| `1` / `2` | 手术刀 / 镊子 |
| 鼠标左键拖动 | 切割（手术刀）或抓取牵拉（镊子） |
| `Q` / `E` | 减小 / 增大刀刃深度（深度小于组织厚度时为半切） |
| 鼠标右键拖动、滚轮 | 旋转、缩放相机 |
| `R` | 重新加载场景 |

场景内三个对象：组织块、两端钉住且带预应力的皮瓣、椭球形肝叶。

## 在自己的场景中使用

1. 在带 `MeshFilter` + `MeshRenderer` 的对象上添加 `ApxSoftBody`。网格必须闭合且 Read/Write 已开启。
2. 设置 `resolution`（最长轴上的体素数）、`youngsModulus`、`poissonRatio`、`density`。
3. `cutMaterial` 指定切面材质，可使用 `AdaptorPhysX/Tissue Interior` 着色器。
4. 需要固定的部位放 Collider（可设为 Trigger），拖入 `pinColliders`。
5. 刀具对象添加 `ApxScalpel`，用 `edgeStart` / `edgeEnd` 定义刀刃，`cutting` 打开时运动即切割。
6. 抓取：`ApxSoftBody.BeginGrab(ray, radius, out point)` → `MoveGrab(target)` → `EndGrab()`。

## 目录

```
Assets/AdaptorPhysX/Runtime/        插件运行时（asmdef: AdaptorPhysX.Runtime）
  ApxSoftBody.cs, ApxScalpel.cs     组件
  Core/                             lattice、求解器、接触、拓扑、切割、渲染
Assets/AdaptorPhysX/Shaders/        切面着色器
Assets/AdaptorPhysX/Tests/          EditMode 与 PlayMode 测试
Assets/Samples/SurgeryDemo/         演示场景、脚本、场景生成器、标准场景截图工具
docs/                               设计说明、路线图、截图
```

## 测试与标准场景截图

```bash
"D:/Program Files/Unity/Hub/Editor/2022.3.54f1/Editor/Unity.exe" -batchmode -projectPath . -runTests -testPlatform EditMode -testResults editmode.xml
```

```bash
"D:/Program Files/Unity/Hub/Editor/2022.3.54f1/Editor/Unity.exe" -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.SurgeryDemoCapture.CaptureBatch -apxCaptureDir Captures
```

截图工具按脚本执行 `docs/ROADMAP.md` 中的标准场景 S1–S5，输出 PNG，并在日志中记录每个阶段的块大小与耗时。

## 历史

2026-10 之前的布料切割（C++/CUDA native 库 + Unity 绑定）与绳索、多引擎计划已冻结并从工作树删除，
历史保留在三个 tag：`archive/phase1-unity`、`archive/phase1-native`、`archive/phase1-meta`。

## 许可

Apache License 2.0，见 `LICENSE.txt`。
