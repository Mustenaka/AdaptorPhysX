# AdaptorPhysX

Unity 开源体素柔体插件，支持动态切割与半切割，面向虚拟手术等医学仿真场景。
实现基于 Burst + Jobs（CPU 多线程）；Compute Shader 后端在路线图中。

![标准场景总览](docs/images/99_overview.png)

| 全切后拉开 | 预应力皮瓣半切 | 镊子牵拉切口 | 楔形切除 |
|---|---|---|---|
| ![S1](docs/images/S1b_block_pulled.png) | ![S2](docs/images/S2_flap_incision.png) | ![S4](docs/images/S4_flap_retraction_close.png) | ![S5](docs/images/S5b_liver_wedge_lifted.png) |

| 十字切与再次切割后拉开的 5 块 | 实体组织上的小切口 | 牵开切口，露出真皮与脂肪 | 缝合 3 针后切口闭合 |
|---|---|---|---|
| ![S3b](docs/images/S3b_block_pieces_apart.png) | ![S6a](docs/images/S6a_pad_incision.png) | ![S6b](docs/images/S6b_pad_retracted.png) | ![S7b](docs/images/S7b_pad_sutured_close.png) |

| 软体之间的接触 | 钢球（Rigidbody）落在组织上 | 环形器械（距离场）压入 | 粘附的平板提起组织 |
|---|---|---|---|
| ![S8](docs/images/S8_bodies_stacked.png) | ![S9](docs/images/S9_ball_on_tissue.png) | ![S10](docs/images/S10a_ring_pressed.png) | ![S11c](docs/images/S11c_paddle_lifting.png) |

| 真实网格：扫描版兔子、Fertility、龙 | Fertility 全切后拉开 | 龙横切后拉开 | 镊子提起兔子的耳朵 |
|---|---|---|---|
| ![S12a](docs/images/S12a_real_meshes.png) | ![S12b](docs/images/S12b_fertility_cut.png) | ![S12c](docs/images/S12c_dragon_cut.png) | ![S12d](docs/images/S12d_bunny_lifted.png) |

| 调试显示：lattice 按块着色与断棱 | 受力与包围盒 | 侧倾 30°、前倾 20° 的刀刃 |
|---|---|---|
| ![D1](docs/images/D1_block_lattice_cuts.png) | ![D2](docs/images/D2_contact_forces.png) | ![D3](docs/images/D3_blade_pose.png) |

- 体积表示：稀疏六面体 lattice，每个体素按 Kuhn 剖分拆为 6 个四面体。
- 材料模型：co-rotational XPBD（偏量项 `‖F − R‖` 与体积项 `det F − 1` 联立求解），参数为杨氏模量与泊松比。
  可选皮肤层：表面以下一定厚度内刚度更高，并带沿表面方向的预应力，小切口在张力下张开。
- 时间积分：small steps（每帧多个子步，每个子步 1 次迭代），按体素切片 2 色并行，切片内 4 个体素一组用 SIMD 求解，
  同一机器、同一构建下结果可逐位复现。静止的物体自动休眠，不占用计算。
- 切割：刀刃扫掠面与 lattice 棱求交后断开棱，按 virtual node algorithm 复制体素副本。
  全切、半切（切口终止在组织内部）、交叉切与对已分离部分的再次切割使用同一套机制。
- 渲染：原始表面网格以三线性插值嵌入体素副本，并沿切平面拆分；切面由断开棱上的交点与法线生成，
  按原始表面的有符号距离场裁剪到表面以内。切面着色器使用静止坐标的 3D 噪声，并按表面以下的深度分层着色
  （真皮、脂肪、肌肉）。
- 接触：Unity Sphere/Capsule/Box 碰撞体（含摩擦，子步内插值位姿）；MeshCollider 与任意形状的器械
  （有符号距离场，运行时或在编辑器中烘焙）；切下的块之间、折叠的组织之间、不同 `ApxSoftBody` 之间的接触。
  接触距离按可见表面计算（`collisionMargin` 为 0 时表面贴住碰撞体）；比 2.5 个体素细的器械另推出表面体素，
  不从 lattice 节点之间穿入（`surfaceContact`）。
- 器械与刚体：碰撞体受到的力、力矩与接触中心（`TryGetColliderWrench`），抓取与钉住区域受到的力
  （`TryGetGrabWrench`、`TryGetPinWrench`）；与非运动学 Rigidbody 的双向耦合（组织托住、推开刚体）；
  器械表面粘附（stickiness）。
- 材料：可选塑性（超过屈服应变的形变按速率计入静止形状）。
- 状态快照（含切割拓扑）与 blueprint 资产（预先体素化，Start 不再体素化）。
- 交互：手术刀（`ApxScalpel`）、钉住区域、镊子抓取（不跨越切口；可同时多处，抓取帧随器械移动与转动）；动态附着把材料点连到 Transform（`AddAttachment`，带柔度与断开力，输出受力）；
  两钳口夹钳（`ApxGrasper`）：钳口合拢后夹住其间的组织，随夹钳移动与转动，拉力超过钳口摩擦 2μN 时滑脱。
- 简化缝合：切口两侧两个材料点之间的针脚（`AddStitch`），按速度收紧到两点之间组织的无应力长度，输出张力；
  半切切口两壁之间有切面接触，缝合对合后两壁不相互穿过（DESIGN 9.11 节）。
- 材料点：嵌入体素副本的点（`AddMaterialPoint(世界坐标)`、`MaterialPointPosition`），随组织移动，切割后留在原来的一侧；用作缝合、附着与接触的锚点（DESIGN 4.4 节）。
- 交互工具与调试显示（`AdaptorPhysX.Tools`）：相机旋转、平移、向指针缩放；手术刀的航向、侧倾、前倾与深度可调；
  镊子抓取与提起；测量；参考网格、lattice、节点、断棱、受力与刀刃扫掠面的调试显示；暂停与单步。

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
| `1` / `2` / `3` | 手术刀 / 镊子 / 测量 |
| 鼠标左键拖动 | 切割（在组织上或组织旁按下）、抓取牵拉、选取测量点 |
| 右键拖动（或 Alt + 左键） | 绕中心点旋转相机 |
| 中键拖动（或 Shift + 右键） | 平移相机 |
| 滚轮 | 向指针处缩放 |
| `F` / `Home` | 框选指针下的物体（指针下没有物体时框选全部）/ 回到初始视角 |
| 小键盘 `1` / `3` / `7` / `5` | 前视 / 右视 / 顶视 / 初始角度 |
| `Q` / `E`，Ctrl + 滚轮 | 减小 / 增大刀刃深度。深度为刀尖在刀下组织表面以下的竖直距离，拖动时随表面高度变化；深度小于组织在竖直方向的厚度时为半切 |
| `Z` / `X`，`C` / `V`，`A` / `D` | 刀刃侧倾、前倾、航向 |
| `T` / `B` | 航向跟随刀路（默认开启）/ 刀刃回正 |
| Shift + 左键拖动 | 沿当前航向直线切割 |
| `[` / `]` | 手术刀稳定绳长度（默认 8 mm，0 时刀直接跟随指针）；镊子抓取半径 |
| 夹持时滚轮 | 镊子提起或放低 |
| Shift + 松开左键 | 镊子保持夹持，再次点击松开 |
| `P` / `N` | 暂停 / 单步 |
| `F2`–`F8` | 调试图层：网格、lattice、节点、断棱、受力、刀刃、包围盒 |
| `F9` / `F10` | 调试线绘制在表面之上 / 隐藏表面 |
| `Tab` / `F1` / `R` | 显示面板 / 显示按键说明 / 重新加载场景 |

左上角面板显示当前工具的状态（刀刃角度与深度、本刀断棱数、夹持力、测量距离）与各物体的块数、步进耗时，
工具、视角、调试图层与节点着色方式也可在面板中点选。手术刀按下后移动 3 mm 才开始切割，航向取这段移动的方向；
在组织旁按下时刀刃从侧面进入，全切需要从组织旁起刀。切割时刀以稳定绳跟随指针（黄线为绳，十字为指针），
指针离刀超过绳长时刀才移动，手的抖动不进入切口；松开时刀停在指针后一个绳长处。指针在过按下点的水平面上移动。

场景内四个对象：组织块、两端钉住且带预应力的皮瓣、椭球形肝叶、放在托盘中带皮肤层的组织垫（Tissue Pad，
用于实体组织上的小切口）。

交互演示 `Assets/Samples/InteractionDemo/InteractionDemo.unity`（菜单 `AdaptorPhysX/Build Interaction Demo Scene`
重新生成），操作与手术演示相同：

| 对象 | 内容 |
|---|---|
| Base Block、Dropped Lobe | 肝叶落在组织块上，两个软体相互接触 |
| Hammock、Steel Ball | 钢球（Rigidbody，0.1 kg）落在两端夹住的组织上，组织下垂并托住钢球 |
| Pressed Block、Ring Instrument | 环形器械（MeshCollider，距离场资产 `Ring Field`）循环压入组织 |
| Putty、Putty Press | 塑性材料被球压一次后保留压痕 |
| Wet Tissue、Sticky Paddle | 带粘附的平板压在组织上后抬起，组织表面随之上移，超过粘附距离后脱离 |

真实网格演示 `Assets/Samples/RealMeshDemo/RealMeshDemo.unity`（菜单 `AdaptorPhysX/Build Real Mesh Demo Scene` 重新生成），
操作相同。场景中是 `Assets/AdaptorPhysX/Arts/Models` 中的三个 FBX 测试模型（分辨率 40）：扫描版斯坦福兔子（底部有孔，
按广义缠绕数体素化）、Fertility（亏格 4）、斯坦福龙（87 万三角形，FBX 没有可用的法线，初始化时重新计算）。

## 在自己的场景中使用

1. 在带 `MeshFilter` + `MeshRenderer` 的对象上添加 `ApxSoftBody`。网格需开启 Read/Write，最好闭合：有孔的网格
   按广义缠绕数判定内外，孔只在体积（lattice）中封闭，渲染表面保留孔，初始化时输出警告。网格法线与三角形不符
   （没有法线，或导入时没能算出）时按三角形重新计算，并输出警告。
2. 设置 `resolution`（最长轴上的体素数）、`youngsModulus`、`poissonRatio`、`density`。
   需要皮肤层时设置 `skinThickness`（米）、`skinYoungsModulus`、`skinPreStrain`；整体预应力用 `preStrain`。
   这些参数在运行时修改后下一步生效；修改 `resolution` 或 `pinColliders` 后调用 `Rebuild()`（切割会丢弃）。
3. `cutMaterial` 指定切面材质，可使用 `AdaptorPhysX/Tissue Interior` 着色器。
4. 需要固定的部位放 Collider（可设为 Trigger），拖入 `pinColliders`。
5. 刀具对象添加 `ApxScalpel`，用 `edgeStart` / `edgeEnd` 定义刀刃，`cutting` 打开时运动即切割。
   自己调用 `ApxSoftBody.Cut` 时，同一次连续切割的各次调用传入相同的非 0 `stroke`，组织在切割过程中移动时也能分开。
6. 抓取：`ApxSoftBody.BeginGrab(ray, radius, out point)` → `MoveGrab(target)` → `EndGrab()`。
   抓取受到的力（组织作用在抓取上，含被抓取部分的重量）：`TryGetGrabWrench(out ApxWrench)`；
   钉住区域受到的力：`TryGetPinWrench(pinIndex, out ApxWrench)`，`pinIndex` 为 `pinColliders` 的下标。
7. `allowSleep`（默认开启）：物体静止后停止步进，碰撞体移动、切割、抓取时恢复。直接修改节点数据后调用 `WakeUp()`。
8. 器械：
   - MeshCollider 直接可用（网格需 Read/Write，运行时按网格烘焙距离场）。复杂器械可新建 `ApxDistanceField` 资产
     （菜单 Create → AdaptorPhysX → Distance Field，检视面板中烘焙），在器械的 Collider 上添加 `ApxColliderMaterial`
     并引用该资产；此时 Collider 只用于包围范围查询。
   - `ApxColliderMaterial` 还可设置该碰撞体的摩擦系数与粘附（`stickiness`、`stickDistance`）。
   - 半径（盒为最小半边长）小于 2.5 个体素的碰撞体按表面体素接触，压入时组织表面随器械尖端下陷；一个体素不能形成比它窄的
     凹坑，针杆旁的表面在器械内最深约 0.2 个体素（DESIGN 9.9 节）。`surfaceContact` 关闭时只有节点接触。
   - 碰撞体带非运动学 Rigidbody 时与组织双向耦合（`twoWayCoupling`，默认开启）；此时物体每次 FixedUpdate 都步进，不受 `maxStepsPerFrame` 限制。
   - 器械受力：`TryGetColliderWrench(collider, out ApxWrench)`；多个软体对同一碰撞体的合力用 `ApxHaptics.TrySumColliderWrench`。
   - 力反馈：`TryGetColliderForceModel(collider, out ApxForceModel)` 给出每步发布的局部线性力模型，
     触觉线程在两次物理步之间用 `model.Evaluate(器械位置)` 计算力；多个软体用 `ApxHaptics.TrySumColliderForceModel`。
9. 多个 `ApxSoftBody` 之间默认相互接触（`collideWithBodies`，并受 `collisionLayers` 过滤）。
10. 塑性：`plasticCreep` 大于 0 时启用，`plasticYield` 为屈服应变，`plasticRecovery` 为恢复速率，
    `maxPlasticDeformation` 为塑性变形上限。
11. 快照：`ApxSoftBodyState state = body.CaptureState()`，之后 `body.RestoreState(state)` 回到该状态（含切口）。
12. Blueprint：检视面板“Bake Blueprint”生成资产并赋给 `blueprint`；网格、分辨率或缩放改变后需重新烘焙。
13. 交互工具：`ApxToolbox.CreateDefault(camera, scalpel, view)` 给相机加上 `ApxCameraRig`，并新建对象
    “Apx Tools”，含手术刀、镊子、测量工具与调试显示（`ApxDebugView`）。调试显示也可单独添加，`layers` 选择图层，
    编辑模式下在 Scene 视图中同样可见（内置渲染管线）。工具读取旧输入系统（Input Manager）。

## 目录

```
Assets/AdaptorPhysX/Runtime/        插件运行时（asmdef: AdaptorPhysX.Runtime）
  ApxSoftBody*.cs, ApxScalpel.cs    组件（ApxSoftBody.Interaction.cs：器械、刚体、快照、blueprint）
  ApxColliderMaterial.cs 等         碰撞材质、距离场资产、blueprint 资产、力的类型
  SoftBodyWorld.cs                  软体之间的配对
  Core/                             lattice、求解器、接触、拓扑、切割、渲染、距离场、刚体耦合
Assets/AdaptorPhysX/Tools/          交互工具与调试显示（asmdef: AdaptorPhysX.Tools）
Assets/AdaptorPhysX/Editor/         检视面板（blueprint 与距离场烘焙）
Assets/AdaptorPhysX/Shaders/        切面着色器
Assets/AdaptorPhysX/Tests/          EditMode 与 PlayMode 测试
Assets/AdaptorPhysX/Benchmarks/     基准测试（编辑器，JSON 输出）
Assets/Samples/SurgeryDemo/         演示场景、脚本、场景生成器、标准场景截图工具
Assets/Samples/InteractionDemo/     交互演示场景（S8–S11）、场景生成器与截图工具
Assets/Samples/RealMeshDemo/        真实网格演示场景（S12）、场景生成器与截图工具
Benchmarks/                         基准网格下载脚本；网格与结果不提交
docs/                               设计说明、路线图、待办清单（TODO.md）、截图
```

## 测试与标准场景截图

```bash
"D:/Program Files/Unity/Hub/Editor/2022.3.54f1/Editor/Unity.exe" -batchmode -projectPath . -runTests -testPlatform EditMode -testResults editmode.xml
```

```bash
"D:/Program Files/Unity/Hub/Editor/2022.3.54f1/Editor/Unity.exe" -batchmode -projectPath . -executeMethod AdaptorPhysX.Samples.Editor.SurgeryDemoCapture.CaptureBatch -apxCaptureDir Captures
```

截图工具按脚本执行 `docs/ROADMAP.md` 中的标准场景 S1–S7，输出 PNG，并在日志中记录每个阶段的块大小与耗时。
交互场景 S8–S11 用 `AdaptorPhysX.Samples.Editor.InteractionDemoCapture.CaptureBatch`（参数相同），
编辑器内为菜单 `AdaptorPhysX/Capture Interaction Scenarios`；真实网格场景 S12 用
`AdaptorPhysX.Samples.Editor.RealMeshDemoCapture.CaptureBatch`，菜单 `AdaptorPhysX/Capture Real Mesh Scenarios`。

## 基准测试

网格：组织块（box）与肝形网格（liver）由代码生成；斯坦福兔子（bunny）与龙（dragon）用下面的脚本下载到
`Benchmarks/Meshes/`（不提交，斯坦福模型仅限研究用途）。放入闭合的 `Benchmarks/Meshes/liver.obj` 时改用该网格。

```bash
sh Benchmarks/fetch-meshes.sh
```

batchmode 运行（工程不能同时在编辑器中打开）。参数均可省略，默认值为全部四个网格、分辨率 16/32/48/64、16 子步、
每阶段 60 步，结果写入 `Benchmarks/Results/apx-benchmark-<时间>.json`：

```bash
"D:/Program Files/Unity/Hub/Editor/2022.3.54f1/Editor/Unity.exe" -batchmode -projectPath . -executeMethod AdaptorPhysX.Benchmarks.Editor.ApxBenchmark.RunBatch -apxBenchmarkOut Benchmarks/Results/bench.json -apxMeshes box,liver,bunny,dragon -apxResolutions 16,32,48,64
```

编辑器内使用菜单 `AdaptorPhysX/Run Benchmarks`。测量内容与 JSON 字段见 `docs/DESIGN.md` 第 8 节。
运行结束时与入库的 `Benchmarks/baseline.json` 比较各阶段的节点位置哈希、块数与断棱数，日志列出不一致的组
（哈希只在同一机器、同一构建下可比）。改动有意改变仿真结果时加 `-apxWriteBaseline`
（或菜单 `AdaptorPhysX/Run Benchmarks and Write Baseline`）重写基线。

## 历史

2026-10 之前的布料切割（C++/CUDA native 库 + Unity 绑定）与绳索、多引擎计划已冻结并从工作树删除，
历史保留在三个 tag：`archive/phase1-unity`、`archive/phase1-native`、`archive/phase1-meta`。

## 许可

Apache License 2.0，见 `LICENSE.txt`。
