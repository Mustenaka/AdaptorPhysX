# 待办清单

本文件列出与 Obi 生态、医疗仿真需求相比尚缺的功能，以及基准测试与代码检查中发现的缺陷，并逐项对照当前实现。
每个条目有固定编号，新对话中直接引用编号（例如“处理 BUG-2”）。

- 对照基准：提交 `17fde10`（2026-10-26），分支 `dev/softbody`。
- 里程碑编号（M1–M5）见 `docs/ROADMAP.md`；算法与数据结构见 `docs/DESIGN.md`。

## 状态定义

| 状态 | 含义 |
|---|---|
| 已完成 | 已实现并有测试或基准数据 |
| 部分完成 | 已实现一部分，未完成的部分在“现状”中列出 |
| 未开始 | 没有实现 |
| 原因未查明 | 缺陷，有复现数据，原因未查明 |
| 原因已确认 | 缺陷，原因已确认，未修复 |

## 新对话的工作方式

1. 读 `README.md`、本文件中对应条目、`docs/DESIGN.md` 中条目引用的章节。
2. 按条目的“验收”完成；按 `docs/ROADMAP.md` 的工作约定附 EditMode 测试，影响效果的改动附标准场景截图。
3. 涉及性能或切割行为的改动，运行基准测试并与 `docs/DESIGN.md` 6.1 节对比：
   - 网格：`sh Benchmarks/fetch-meshes.sh`（兔子、龙；box 与肝形网格由代码生成）。
   - 编辑器未打开工程时用 batchmode（命令见 README）。
   - 编辑器已打开工程时，通过 MCP 调用 `ApxBenchmark.ScheduleKeepingScenes(settings)`。
     不要同步调用 `RunKeepingScenes`：完整运行需要约 150 s，超过 MCP 应答超时后命令会被重发，基准测试会重复运行。
   - 同一机器、同一构建下各阶段的 `positionsHash` 应一致，可用于判断改动是否改变了仿真结果。
4. 完成后更新本文件中条目的状态与现状，并在“变更记录”中加一行。

## 1. 缺陷

| 编号 | 问题 | 现状与证据 | 状态 | 下一步 | 验收 |
|---|---|---|---|---|---|
| BUG-1 | 放在碰撞体上的物体全切后不分离 | DESIGN 6.1：box 64、bunny 32/48/64、dragon 32 的全切断开了棱，块数仍为 1（不计节点数低于 2% 的碎块）。box 64 在无重力、无碰撞体时同一刀路分成两块。有重力时，刀到达之前已有 1259 个节点越过刀面（刀面距最近节点 0.5h） | 原因未查明 | 在切割帧记录刀面经过、但未被断开的棱，以及这些棱所在体素的副本划分；与 BUG-2 修复后的结果对比 | 6.1 节 16 组全切结果均为 2 块；新增一个 EditMode 测试（box 分辨率 64，落到碰撞盒上后全切） |
| BUG-2 | 摩擦使竖直落下的物体整体侧向漂移 | box 从 5 mm 高处落到 Box 碰撞体上，180 步后节点平均 z 位移：摩擦 0.6 时 +1.35 mm（分辨率 32）、+1.75 mm（分辨率 64，x 方向 +0.6 mm）；摩擦 0 时约为 0。摩擦实现见 `SolverJobs.cs:61`–`76` | 原因未查明 | 分别改变切片内求解顺序、子步数、碰撞体类型（Sphere、Capsule），记录漂移方向与大小 | 摩擦 0.6 时平均水平位移小于 0.05 mm；新增 EditMode 测试 |
| BUG-3 | 落下后的体积损失随分辨率增大 | 6.1 节落下 1.5 s 后表面体积比：box 0.996（16）→ 0.972（64），bunny 64 为 0.946。ν = 0.45 | 原因未查明 | 分别用 `-apxSubsteps 8`、`16`、`32` 运行基准测试，对比各分辨率的体积比；在 `SelfWeightCompressionMatchesYoungsModulus` 中加入分辨率 64 | 分辨率 16–64 之间体积比差异小于 1% |
| BUG-4 | 刀面经过一层 lattice 节点时不分离 | 棱与刀面只在端点相交或位于刀面内，`CutJob`（`TopologyJobs.cs`）不断开这些棱。复现：box 分辨率 16，刀面取节点包围盒中心（恰为一层节点） | 原因已确认 | `CutJob` 中对端点位于刀面上的情况统一归到刀面一侧 | 新增 EditMode 测试：沿一层节点全切，得到 2 块 |
| BUG-5 | 参数的生效时机不一致 | `BuildParameters()` 每步传入全部参数（`ApxSoftBody.cs:244`）。`preStrain` 与三个 `skin*` 参数只在构造时使用（`SoftBodySimulation.cs:232`），运行时修改无效且无提示。`density` 只在拓扑重建中使用（`SoftBodySimulation.cs:692`），运行时修改后要到下一次切割才生效 | 原因已确认 | 与 CFG-2 一起处理 | 见 CFG-2 |

## 2. 与 Obi 生态的差距

| 编号 | 能力 | Obi | AdaptorPhysX 现状 | 状态 | 验收 |
|---|---|---|---|---|---|
| ECO-1 | 不同物体之间的碰撞 | 同一 solver 内全部物体互相碰撞 | 没有（DESIGN 第 7 节）。切下的块只与同一物体的其他块接触 | 未开始 | 两个 `ApxSoftBody` 相互挤压不穿透；属于 M3 |
| ECO-2 | 碰撞体类型 | 支持 MeshCollider 与距离场 | 只有 Sphere、Capsule、Box（`ApxSoftBody.cs:401`–`418`） | 未开始 | MeshCollider 或 SDF 形状的器械（钳口、弯剪）与组织接触 |
| ECO-3 | 与刚体的双向耦合 | 有 | 单向：组织不对 Rigidbody 施加力 | 未开始 | 组织能推动带 Rigidbody 的器械 |
| ECO-4 | GPU 后端 | Obi 7 有 Compute Shader 后端 | 没有 | 未开始 | 属于 M4 |
| ECO-5 | 布料、绳、流体 | 有 | 已冻结（ROADMAP 决策 3）。医疗场景用途：膜与筋膜（布料）、缝线与血管（绳）、出血（流体） | 未开始 | 需先修改 ROADMAP 决策 3 |
| ECO-6 | 粘附与塑性 | 碰撞材质有 stickiness，softbody 有 plasticity | 没有 | 未开始 | 湿组织粘附器械；按压后保留残余形变 |
| ECO-7 | 编辑器工具 | blueprint 预计算；用笔刷绘制质量、钉住区域 | 每次 Start 重新体素化（M5 列出“blueprint 资产缓存体素化结果”）；钉住区域只能用 Collider 选取 | 未开始 | 体素化结果存为资产，Start 不再体素化；在编辑器中绘制钉住区域与材料 |
| ECO-8 | 状态快照与撤销 | 有 | 没有 API | 未开始 | 保存与恢复仿真状态（含切割拓扑），恢复后 `positionsHash` 与保存时一致 |

## 3. 医疗仿真需求

### 3.1 材料

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-1 | 应变硬化（J 形应力-应变曲线） | co-rotational 线弹性，小应变线性化（DESIGN 第 3 节） | 未开始 | 单轴拉伸曲线与所选本构模型的解析解一致 |
| MED-2 | 粘弹性（应力松弛、蠕变） | 没有 | 未开始 | 恒定应变下应力随时间衰减，衰减时间常数可配置 |
| MED-3 | 各向异性（肌纤维方向） | 没有 | 未开始 | 沿纤维方向与垂直方向的刚度比可配置并可测量 |
| MED-4 | 阻尼模型 | 全局速度阻尼（`SolverJobs.cs:95`），刚体运动也被衰减 | 未开始 | 自由下落与匀速平移不受阻尼影响 |
| MED-5 | 多材料与体素材料标签 | 两层：皮肤层与主体（DESIGN 第 3 节）。M5 列出脂肪、肌肉不同材料 | 部分完成 | 每个体素一个材料 ID；从 CT/MRI 分割标签（NIfTI）读取材料 ID |
| MED-6 | 超阈值撕裂 | 没有（M5） | 未开始 | 应变超过阈值时断棱，撕裂沿受力方向扩展 |

### 3.2 切割精度与器械

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-7 | 切口精度 | 分辨率上限 64（`ApxSoftBody.cs:22` 的 Inspector 范围），0.15 m 的物体约 2.3 mm 一个体素；薄于一个体素的结构无法分离（DESIGN 4.3 节）；刀路在体素内弯曲时切口为折线（DESIGN 第 7 节） | 未开始 | 切口附近局部加密 lattice；弯曲刀路的切口误差小于设定值 |
| MED-8 | 切割阻力与切割力 | 扫掠面与棱相交即断开，组织不先压陷；没有切割力输出 | 未开始 | 切入前组织压陷，切入条件取决于刀锋参数与组织韧性；输出刀刃受力 |
| MED-9 | 器械种类 | 只有线段刀刃（`ApxScalpel`）。缺：剪刀（两刃闭合）、电刀与超声刀（删除体素）、穿刺针（插入不切开）、缝合（重新连接断棱，当前拓扑连通只减少）、夹闭器与吻合器 | 未开始 | 每种器械一个标准场景截图与 EditMode 测试 |
| MED-10 | 镊子夹持 | 半径内节点设为运动学节点（`BeginGrab`），等同固定连接 | 未开始 | 两钳口夹持，接触带摩擦，拉力超过阈值时滑脱 |

### 3.3 力反馈

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-11 | 器械受力输出 | 没有。需要从约束的拉格朗日乘子 λ 与碰撞投影量汇总器械受力 | 未开始 | 压痕实验中输出的力-位移曲线与线弹性解析解一致 |
| MED-12 | 1 kHz 触觉更新 | 没有。物理更新为 60–100 Hz，触觉设备（Touch、Force Dimension）需要 1 kHz，两者需要解耦（例如 virtual coupling） | 未开始 | 触觉线程以 1 kHz 输出力，物理帧率下降时输出连续 |

### 3.4 训练系统接口

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-13 | 评分事件与指标 | 没有。需要：切口长度、深度、位置，组织损伤量，超力事件 | 未开始 | 事件 API 与一个标准场景的评分输出 |
| MED-14 | 输入录制与回放 | 同一机器、同一构建下结果逐位一致（`SameInputsGiveBitIdenticalResults`）；基准测试输出节点位置哈希。没有录制与回放 API | 部分完成 | 录制器械位姿与操作序列，回放后 `positionsHash` 与录制时一致 |

### 3.5 平台

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-15 | URP 支持 | 只支持内置渲染管线（切面着色器 `ApxTissueInterior.shader`） | 未开始 | URP 下标准场景 S1–S6 截图正常 |
| MED-16 | Player 与 ARM 平台 | 性能数据均在编辑器中测得（见 PERF-4） | 未开始 | Quest（ARM、IL2CPP）上运行演示场景并记录基准数据 |

## 4. 参数化与 AI 可配置

目标：参数以数据形式存在并带元信息（单位、范围、生效时机），可由 AI 通过 schema 读取、通过 MCP 工具修改，并通过 JSON 指标验证修改结果。

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| CFG-1 | 参数元数据特性，例如 `[ApxParam(Unit = "Pa", Min, Max, Typical, Apply = Hot / Rebuild / Init)]` | 只有 Unity 的 `Range`、`Tooltip` | 未开始 | `ApxSoftBody` 全部公开参数带元数据 |
| CFG-2 | 修正生效时机，提供 `Rebuild()` | 见 BUG-5；改 `resolution` 需要重建，没有 API | 未开始 | `density` 修改后下一步生效；Init 类参数修改时触发重建或输出警告；`Rebuild()` 保留钉住设置 |
| CFG-3 | 从元数据生成 JSON Schema 与参数文档 | 没有 | 未开始 | 生成的 schema 覆盖 CFG-1 的全部参数 |
| CFG-4 | 材料预设（JSON，附文献出处），组件引用预设并局部覆盖 | 没有 | 未开始 | liver、skin、fat 三个预设；预设文件可文本比较 |
| CFG-5 | `GetParams` / `SetParams` 接口 | 没有 | 未开始 | `SetParams` 返回每个参数的结果：已生效、需重建、超出范围 |
| CFG-6 | 只读派生量 | 已有：`CellSize`、`LastStepMilliseconds`、`LastTopologyMilliseconds`、`LastInitializeTimings`。缺：子步 dt、总质量、稳定性数 μΔt²/(ρh²)（DESIGN 第 3 节） | 部分完成 | 派生量可通过 CFG-5 的接口读取 |
| CFG-7 | 硬编码常量改为参数 | `SleepSteps`、`SleepSpeed`（`ApxSoftBody.cs:90`、`92`）；接触搜索半径 1.8h、最小距离 0.8h（`SoftBodySimulation.cs:417`、`436`、`437`）；扫掠延伸 0.5h / 0.1h（`SoftBodySimulation.cs:507`）；`ContactInterval`（`SoftBodySimulation.cs:61`）；`MaxKinematicFrames`（`SoftBodySimulation.cs:58`） | 未开始 | 常量带 CFG-1 元数据，默认值不变时 `positionsHash` 不变 |
| CFG-8 | unity-mcp 自定义工具：`apx_get_params`、`apx_set_params`、`apx_run_scenario`、`apx_benchmark` | 基准测试可通过 MCP 的代码执行调用 `ApxBenchmark.ScheduleKeepingScenes`；没有自定义工具 | 未开始 | 四个工具可用；长时间任务异步执行并可查询结果 |
| CFG-9 | 文本描述的实验场景（刀路、参数、期望指标），batchmode 运行并输出 JSON | 基准测试输出 JSON，场景固定在代码中；标准场景截图工具（S1–S6）只输出 PNG 与日志 | 部分完成 | 场景文件定义刀路与期望指标；S1–S6 输出 JSON 指标 |
| CFG-10 | 闭环调参示例 | 没有。依赖 MED-11 的力输出 | 未开始 | 调整材料参数，使压痕的力-位移曲线与给定文献数据的误差小于设定值 |

## 5. 性能与基准测试

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| PERF-1 | 基准测试框架 | `AdaptorPhysX/Run Benchmarks`、`ApxBenchmark.RunBatch`，网格 box、liver、bunny、dragon，分辨率 16/32/48/64，JSON 输出（DESIGN 第 8 节），结果见 DESIGN 6.1 节 | 已完成 | — |
| PERF-2 | 基准矩阵的其余维度 | 未覆盖：分辨率 96/128（`resolution` 可由代码设置，Inspector 范围为 4–64）；子步 8/32（`-apxSubsteps` 已支持，未运行）；抓取状态；Armadillo、Thingi10K、真实器官分割网格 | 部分完成 | 上述维度各有一组结果 |
| PERF-3 | 指标缺口 | Native 内存只有 Unity 总分配量的差值（编辑器中为近似值）；托管分配为 Mono 堆增长量，期间发生 GC 时记为 -1；首次 JIT 耗时在预热中，未单独记录 | 部分完成 | 记录仿真与表面网格的 Native 内存；首次切割帧耗时单独记录 |
| PERF-4 | batchmode、Player（IL2CPP）与 ARM 数据 | 6.1 节数据来自交互式编辑器（Debug、Jobs Debugger 开启）。batchmode 入口未实际运行过 | 未开始 | batchmode 运行一次；Windows Player（IL2CPP、Release）与 Quest 各一组结果 |
| PERF-5 | 仿真拓扑局部重建 | 每个切割帧全量重建：1428 个体素 1.2 ms，77220 个体素 51 ms（6.1 节） | 未开始 | 77220 个体素时切割帧扫掠小于 5 ms；属于 M3 |
| PERF-6 | 表面更新与渲染拓扑重建 | 龙（43.5 万顶点）每帧表面更新 8–11 ms，切割帧表面重建 p95 200–460 ms；box、liver、bunny 分辨率 64 时 p95 约 160 ms | 未开始 | 切割帧表面重建 p95 小于 16 ms；顶点在 GPU 上写入（M4） |
| PERF-7 | 初始化耗时 | box 64 的 4.78 s 中表面细分 3.67 s（`MeshUtil.Subdivide`，托管 List）；龙 64 的 3.93 s 中体素化 2.43 s（`InsideParityJob` 每行遍历全部三角形） | 未开始 | 两项各降低一个数量级；或由 ECO-7 的 blueprint 缓存替代 |
| PERF-8 | 自适应 lattice | 分辨率上限 64，lattice 均匀 | 未开始 | 与 MED-7 一起验收 |
| PERF-9 | 减少子步数的求解方法 | 16 子步、每子步 1 次迭代。候选：Chebyshev 加速、Vertex Block Descent | 未开始 | 评估报告：相同精度下的耗时对比（与 BUG-3 的体积损失一起比较） |
| PERF-10 | 开放网格的内外判定 | 体素化要求闭合网格（射线奇偶测试）。当前 bunny 为闭合版本，不受影响 | 未开始 | 广义缠绕数判定；原始扫描版兔子（底部有孔）体素化结果正确 |

## 6. 建议顺序

| 顺序 | 条目 | 状态 |
|---|---|---|
| 1 | 基准测试框架（PERF-1） | 已完成 |
| 2 | 切割与接触缺陷（BUG-1、BUG-2、BUG-4），之后 BUG-3 | 未开始 |
| 3 | 参数元数据、生效时机、MCP 工具（CFG-1、CFG-2、CFG-5、CFG-6、CFG-8；同时处理 BUG-5） | 未开始 |
| 4 | 跨物体碰撞、MeshCollider 与 SDF 器械、局部重建（ECO-1、ECO-2、PERF-5、PERF-6） | 未开始 |
| 5 | 力反馈（MED-11、MED-12） | 未开始 |
| 6 | 体素材料标签 → 非线性与粘弹性材料 → 撕裂（MED-5、MED-1、MED-2、MED-6） | 未开始 |

第 2 项排在参数化之前：BUG-1 与 BUG-2 影响切割结果与基准测试中的“全切结果”列，后续改动需要以此为对照。

## 变更记录

| 日期 | 提交 | 内容 |
|---|---|---|
| 2026-10-26 | `17fde10` | 基准测试框架（PERF-1）；基准测试与代码检查发现 BUG-1 至 BUG-5 |
| 2026-10-26 | — | 建立本文件 |
