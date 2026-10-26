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
| MED-9 | 器械种类 | 只有线段刀刃（`ApxScalpel`）。缺：剪刀（两刃闭合）、电刀与超声刀（删除体素）、穿刺针（插入不切开）、夹闭器与吻合器。缝合见 3.3 节 | 未开始 | 每种器械一个标准场景截图与 EditMode 测试 |
| MED-10 | 镊子夹持 | 半径内节点设为运动学节点（`BeginGrab`），等同固定连接 | 未开始 | 两钳口夹持，接触带摩擦，拉力超过阈值时滑脱 |

### 3.3 缝合

缝合由三部分组成：缝针穿刺组织、缝线在穿刺通道内滑动、收紧缝线使切口两侧对合。SUT-1 是其他条目的基础；
SUT-5 不模拟缝线，可以在缝针与缝线之前实现，用于验证切口闭合与组织张力。

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| SUT-1 | 材料点锚定 | 抓取与钉住通过节点列表绑定到运动学帧（`Attach`，最多 32 帧，`SoftBodySimulation.cs:58`）；没有嵌入在体素副本内的材料点 | 未开始 | 材料点由（体素副本，局部坐标）定义，位置按三线性插值计算；切割后随副本继承；可作为穿刺点、缝线锚点、力反馈接触点使用 |
| SUT-2 | 缝针穿刺 | 没有（直针穿刺见 MED-9） | 未开始 | 弧形缝针沿轨迹刺入并穿出组织，不产生切口；进针力超过阈值才刺入；针与组织之间有摩擦；记录进针点与出针点（SUT-1 材料点） |
| SUT-3 | 缝线 | 没有。绳的仿真已冻结（ECO-5，ROADMAP 决策 3） | 未开始 | 缝线可弯曲、长度近似不变，与组织、器械和自身碰撞。开始前需修改 ROADMAP 决策 3 |
| SUT-4 | 缝线穿过组织 | 没有 | 未开始 | 缝线在穿刺通道内滑动，带摩擦；收紧缝线时切口两侧对合 |
| SUT-5 | 简化缝合（不模拟缝线） | 没有。另外，按 DESIGN 3.1 节，半切切口两侧属于同一块且静止距离小，不产生接触，对合后切面可能相互穿透 | 未开始 | 每一针为连接切口两侧两个材料点的距离约束，目标长度逐步减小到 0；S6 的切口在皮肤张力下被拉拢闭合；对合后两侧切面不相互穿透 |
| SUT-6 | 打结 | 没有 | 未开始 | 判定结的形成；成结后缝线两段之间改为固定约束，松开器械后缝线张力保持 |
| SUT-7 | 缝合后的拓扑 | 切割只减少连通（DESIGN 4.2 节），没有重新连接断棱的机制 | 未开始 | 单次训练的时长内一般不需要愈合；如需要，定义断棱重新连接与体素副本合并的规则，并保持 `positionsHash` 可复现 |
| SUT-8 | 缝合评分指标 | 没有 | 未开始 | 针距、边距（进针点到切口边缘的距离）、进针深度、对合程度、缝线张力；与 MED-13 使用同一事件接口 |
| SUT-9 | 标准场景 S7 | 没有 | 未开始 | 在 S6 的小切口上缝 3 针并收紧，切口闭合；加入 ROADMAP 标准场景表，截图工具输出 S7 |

### 3.4 力反馈（计算接口）

范围：只提供力的计算接口与线程间交换接口，不包含设备驱动（OpenHaptics、Force Dimension SDK 等）。
没有实际设备，所有条目的验收都以 EditMode 测试或脚本设备完成。按编号顺序实现，每个条目可单独在一次对话中完成。

约定：
- 单位为 N、N·m、m、s，坐标为世界坐标。力的方向为组织作用在器械上的力。
- 一个物理步内各子步结果的平均值为该步的力；可选输出逐子步序列。16 子步、60 Hz 时子步频率为 960 Hz，
  但这些值在一帧内一次计算完成，对设备有一帧的延迟。
- 求和不使用浮点原子操作：按批次求部分和，再按固定顺序归约，保持同一机器、同一构建下逐位可复现。

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| HAP-1 | 碰撞体受力：器械（Collider）受到的力、力矩、接触中心 | 没有。碰撞投影在 `NodeStepJob`（`SolverJobs.cs:61`–`76`）中直接修改节点位置，修正量未记录 | 未开始 | 节点 i 对碰撞体 s 的位置修正 Δx（法向投影加摩擦）换算为力 `m_i Δx / h²`（h 为子步长），对节点求和取反；力矩相对碰撞体 Transform 位置。静止物体放在 Box 碰撞体上，碰撞体受力等于总重量（误差 < 2%） |
| HAP-2 | 运动学帧受力：抓取帧、钉住区域的支座反力 | 没有。运动学节点在求解中逆质量为 0（`SolverJobs.cs:164`），作用在它们上的约束力不计算 | 未开始 | 两种候选方法：在 `SolveCellsJob` 中累加约束对运动学节点的 `λ∇C / h²`；或在步末对运动学节点计算弹性力 `−∂E/∂x`。按下面两项测试选择：用抓取提起整个物体静止后，抓取受力等于总重量（误差 < 2%）；两端钉住的物体静止时，钉住反力之和等于总重量 |
| HAP-3 | 力的物理校验测试 | 没有 | 未开始 | EditMode 测试：自重（HAP-1、HAP-2 中的两项）；力平衡（器械力 + 钉住反力 + 重力 = 0，物体静止时）；球形压头压入组织块，力随深度单调增加，小深度下与 Hertz 接触解比较（容差 30%，与 `SelfWeightCompressionMatchesYoungsModulus` 同级）；相同输入时力逐位一致 |
| HAP-4 | 局部线性力模型：供 1 kHz 线程在两次物理步之间计算力 | 没有 | 未开始 | 每步发布 `F(x) = F0 + K (x − x0)`，K 为 3×3 刚度（N/m）。K 的计算方法待定，候选：各接触节点法向与等效刚度的和 `Σ k n nᵀ`；相邻两步的有限差分（秩一更新）。验收：压头匀速压入时，用模型预测下一步的力，相对误差 < 10% |
| HAP-5 | 虚拟耦合（纯计算） | 没有 | 未开始 | `F = k_c (x_proxy − x_device) + b_c (v_proxy − v_device)`，带力上限；参数由调用方按设备给出。单元测试：脚本设备沿闭合轨迹往返时，设备对用户做的净功不为正（无能量注入） |
| HAP-6 | 线程间交换与设备抽象 | 没有 | 未开始 | 设备位姿由触觉线程写入、物理步读取；力模型由物理步发布、触觉线程读取；双缓冲，无锁，读写不阻塞主线程。`IApxHapticDevice` 预留给设备适配 |
| HAP-7 | 无设备测试 | 没有 | 未开始 | 脚本设备（`ApxScriptedHapticDevice`）在测试线程中以 1 kHz 回放录制或脚本轨迹并记录收到的力；EditMode 测试覆盖：物理帧率从 60 Hz 降到 20 Hz 时输出力连续（相邻 1 ms 的变化小于设定值） |

切割阻力与切割力输出见 MED-8，实现时复用 HAP-1 的累加方式。

接口草案（名称与签名在实现时可调整）：

```csharp
// 一个物理步的受力，组织作用在器械上
public struct ApxWrench
{
    public float3 Force;            // N
    public float3 Torque;           // N·m，相对 Origin
    public float3 Origin;           // 力矩参考点：碰撞体 Transform 位置，或运动学帧原点
    public float3 ContactCentroid;  // 按力大小加权的接触点
    public int ContactNodes;
}

public partial class ApxSoftBody
{
    // HAP-1：上一步中该碰撞体受到的力；本步未接触时返回 false
    public bool TryGetColliderWrench(Collider collider, out ApxWrench wrench);
    // HAP-2：抓取帧与钉住区域（pinColliders 的下标）受到的力
    public ApxWrench GrabWrench { get; }
    public ApxWrench GetPinWrench(int pinIndex);
}

public static class ApxHaptics
{
    // 所有 ApxSoftBody 对同一碰撞体的力之和
    public static ApxWrench SumColliderWrench(Collider collider);
}

// HAP-4：局部线性力模型，每个物理步发布一次
public struct ApxForceModel
{
    public double Time;        // 仿真时间，s
    public float3 Position0;   // 发布时的器械位置
    public float3 Force0;      // 发布时的力
    public float3x3 Stiffness; // dF/dx，N/m
    public float MaxForce;     // 力上限，N
    public float3 Evaluate(float3 position);
}

// HAP-5：虚拟耦合
public struct ApxVirtualCoupling
{
    public float Stiffness;    // N/m
    public float Damping;      // N·s/m
    public float MaxForce;     // N
    public float3 DeviceForce(float3 proxyPosition, float3 proxyVelocity, float3 devicePosition, float3 deviceVelocity);
}

// HAP-6：设备抽象与线程间交换
public interface IApxHapticDevice
{
    bool TryReadPose(out float3 position, out quaternion rotation, out double time);
    void WriteForce(float3 force, float3 torque);
}

public sealed class ApxHapticBridge
{
    public void PublishModel(ApxForceModel model);            // 物理步结束后，主线程
    public ApxForceModel LatestModel { get; }                 // 触觉线程
    public void PublishPose(float3 position, quaternion rotation, double time); // 触觉线程
    public bool TryGetLatestPose(out float3 position, out quaternion rotation, out double time); // 物理步开始前
}
```

### 3.5 训练系统接口

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-13 | 评分事件与指标 | 没有。需要：切口长度、深度、位置，组织损伤量，超力事件，缝合指标（SUT-8） | 未开始 | 事件 API 与一个标准场景的评分输出 |
| MED-14 | 输入录制与回放 | 同一机器、同一构建下结果逐位一致（`SameInputsGiveBitIdenticalResults`）；基准测试输出节点位置哈希。没有录制与回放 API | 部分完成 | 录制器械位姿与操作序列，回放后 `positionsHash` 与录制时一致 |

### 3.6 平台

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
| CFG-10 | 闭环调参示例 | 没有。依赖 HAP-1 的器械受力与 HAP-3 的校验 | 未开始 | 调整材料参数，使压痕的力-位移曲线与给定文献数据的误差小于设定值 |

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
| 5 | 力反馈计算接口（HAP-1 至 HAP-7，按编号顺序） | 未开始 |
| 6 | 体素材料标签 → 非线性与粘弹性材料 → 撕裂（MED-5、MED-1、MED-2、MED-6） | 未开始 |
| 7 | 缝合：SUT-1、SUT-5、SUT-9 先行；SUT-2 至 SUT-4、SUT-6 依赖穿刺（MED-9）与缝线（ECO-5） | 未开始 |

第 2 项排在参数化之前：BUG-1 与 BUG-2 影响切割结果与基准测试中的“全切结果”列，后续改动需要以此为对照。

## 变更记录

| 日期 | 提交 | 内容 |
|---|---|---|
| 2026-10-26 | `17fde10` | 基准测试框架（PERF-1）；基准测试与代码检查发现 BUG-1 至 BUG-5 |
| 2026-10-26 | `7ee1147` | 建立本文件 |
| 2026-10-26 | — | 新增 3.3 节缝合（SUT-1 至 SUT-9）；力反馈改为计算接口范围（3.4 节，HAP-1 至 HAP-7 与接口草案），替代原 MED-11、MED-12 |
